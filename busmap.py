import tkinter as tk
import folium
import pyproj
from folium.plugins import HeatMap
from folium.plugins import MarkerCluster
from folium.plugins import MousePosition

grs80_params = {
    'proj': 'tmerc',
    'lat_0': 38,
    'lon_0': 127.0028902777778,
    'k': 1,
    'x_0': 200000,
    'y_0': 600000,
    'ellps': 'GRS80',
    'units': 'm',
    'no_defs': True,
}

wgs84_params = {
    'proj': 'longlat',
    'ellps': 'WGS84',
    'datum': 'WGS84',
    'no_defs': True,
}

# 좌표 변환을 위한 Transformer 생성
transformer = pyproj.Transformer.from_proj(grs80_params, wgs84_params)

# GPS 좌표
lon_c = 127.0361  # 여기에 실제 GPS X 좌표를 넣으세요
lat_c = 37.5004  # 여기에 실제 GPS Y 좌표를 넣으세요

min_lon, max_lon = lon_c-0.0055, lon_c+0.0055
min_lat, max_lat = lat_c-0.005, lat_c+0.005
coa_lon, coa_lat = (550 /500)*0.0055+127.0306, (400 /500)*0.005 +37.4954
lcoa_min_lon, lcoa_max_lon = (350 /500)*0.0055+127.0306, (750 /500)*0.0055+127.0306
lcoa_min_lat, lcoa_max_lat = (200 /500)*0.005 +37.4954, (600 /500)*0.005 +37.4954


# m = folium.Map(location=[37.5004, 127.0361], zoom_start=15)

m = folium.Map(
    control_scale=True,
    location=[lat_c, lon_c],
    zoom_start=15,
    min_lat=min_lat,
    max_lat=max_lat,
    min_lon=min_lon,
    max_lon=max_lon,
    #tiles="stamentoner"
    #tiles='https://tiles.stadiamaps.com/tiles/stamen_terrain_lines/{z}/{x}/{y}{r}.png', attr='my'
)

geojson = {
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "properties": {"lines": [0,1]},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [127.02758, 37.49795],
                    [127.03693, 37.50081],

                    #강남역-역삼역
                ],
            },
        },
{
            "type": "Feature",
            "properties": {"lines": [0,3]},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    #[127.02758, 37.49795],
                    [127.03693, 37.50081],
                    [127.04272, 37.5025],
                    #역삼역-선릉역
                ],
            },
        },
        {
            "type": "Feature",
            "properties": {"lines": [1,2]},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [127.03693, 37.50081],
                    [127.0339, 37.50727],
                    #역삼역-학동역
                ],
            },
        },
        {
            "type": "Feature",
            "properties": {"lines": [2,3]},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [127.03693, 37.50081],
                    [127.03928, 37.49585],
                    #역삼역-한티역
                ],
            },
        },
{
            "type": "Feature",
            "properties": {"lines": [3]},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [127.03928, 37.49585],
                    [127.044772, 37.49848],
                ],
            },
        },
    ],
}

# manage overlays in groups to ease superposition order
outlines = folium.FeatureGroup("outlines")
line_bg = folium.FeatureGroup("lineBg")
bus_lines = folium.FeatureGroup("busLines")
bus_stops = folium.FeatureGroup("busStops")

line_weight = 6
line_colors = ["red", "green", "blue", "orange", "#f80"]
stops = []
for line_segment in geojson["features"]:
    # Get every bus line coordinates
    segment_coords = [[x[1], x[0]] for x in line_segment["geometry"]["coordinates"]]
    # Get bus stops coordinates
    stops.append(segment_coords[0])
    stops.append(segment_coords[-1])
    # Get number of bus lines sharing the same coordinates
    lines_on_segment = line_segment["properties"]["lines"]
    # Width of segment proportional to the number of bus lines
    segment_width = len(lines_on_segment) * (line_weight + 1)
    # For the white and black outline effect
    folium.PolyLine(
        segment_coords, color="#000", weight=segment_width + 5, opacity=1
    ).add_to(outlines)
    folium.PolyLine(
        segment_coords, color="#fff", weight=segment_width + 3, opacity=1
    ).add_to(line_bg)
    # Draw parallel bus lines with different color and offset
    for j, line_number in enumerate(lines_on_segment):
        folium.plugins.PolyLineOffset(
            segment_coords,
            color=line_colors[line_number],
            weight=line_weight,
            opacity=1,
            offset=j * (line_weight + 1) - (segment_width / 2) + ((line_weight + 1) / 2),
        ).add_to(bus_lines)

# Draw bus stops
for stop in stops:
    folium.CircleMarker(
        stop,
        color="#000",
        fill_color="#ccc",
        fill_opacity=1,
        radius=10,
        weight=4,
        opacity=1,
    ).add_to(bus_stops)

outlines.add_to(m)
line_bg.add_to(m)
bus_lines.add_to(m)
bus_stops.add_to(m)


# 지도를 해당 GPS 좌표로 초기화

MousePosition().add_to(m)

ls = folium.PolyLine(
    locations=[[min_lat, min_lon], [min_lat, max_lon], [max_lat, max_lon], [max_lat, min_lon], [min_lat, min_lon]], color="green"
)
ls.add_to(m)

folium.map.Marker(
    [min_lat, min_lon],
    icon=folium.DivIcon(
        icon_size=(300,50),
        icon_anchor=(0,0),
        html='<div style="font-size: 20pt"><b>POI(1km x 1km)</div>',
        )
    ).add_to(m)


gj = folium.GeoJson(
    data={"type": "Polygon", "coordinates": [[[lcoa_min_lon, lcoa_min_lat], [lcoa_max_lon, lcoa_min_lat], [lcoa_max_lon, lcoa_max_lat], [lcoa_min_lon, lcoa_max_lat]]]}
)
gj.add_to(m)

folium.map.Marker(
    [lcoa_min_lat, lcoa_min_lon],
    icon=folium.DivIcon(
        icon_size=(300,50),
        icon_anchor=(0,0),
        html='<div style="font-size: 12pt"><b>LCoA(400m x 400m)</div>',
        )
    ).add_to(m)

folium.Marker([coa_lat, coa_lon],icon=folium.Icon(color='red', prefix='fa',icon='fire')).add_to(m)
folium.map.Marker(
    [coa_lat, (coa_lon+lcoa_min_lon) / 2],
    icon=folium.DivIcon(
        icon_size=(300,50),
        icon_anchor=(0,0),
        html='<div style="font-size: 10pt"><b>CoA(550,400)</div>',
        )
    ).add_to(m)



with open('./busdata/20231010-1.txt', 'r') as file:
    lines = file.readlines()

count = 1
heatmap = []

for line in lines:

    bus_coordinates = []

    bus_info = line.strip().split('/')

    for info in bus_info:

        bus_route, bus_id, xx, yy = info.split(',')

        x= float(xx)
        y= float(yy)
        temp_x = (x /500)*0.0055+127.0306
        temp_y = (y /500)*0.005 +37.4954

        bus_coordinates.append((bus_id, temp_x,temp_y))
        heatmap.append((temp_y, temp_x))

    fg = folium.FeatureGroup(name="Time:%d" % (count), show=True).add_to(m)
    for i, (bus_id, busX, busY) in enumerate(bus_coordinates, start=1):
        # folium.Marker([busY, busX], icon=folium.Icon(color='blue', prefix='fa',icon='bus'), size='5', tooltip=f'Bus {bus_id}').add_to(marker_cluster)
        folium.Marker([busY, busX], icon=folium.Icon(color='blue', prefix='fa',icon='bus'), size='5', tooltip=f'Bus {bus_id}').add_to(fg)
        pass

    count+=1


#HeatMap(heatmap).add_to(m)
folium.LayerControl().add_to(m)

m.save('bus_map.html')