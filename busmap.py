import tkinter as tk
import folium
import pyproj

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

# 지도를 해당 GPS 좌표로 초기화
m = folium.Map(
    max_bounds=True,
    control_scale=True,
    location=[lat_c, lon_c],
    zoom_start=15,
    min_lat=min_lat,
    max_lat=max_lat,
    min_lon=min_lon,
    max_lon=max_lon,
)

ls = folium.PolyLine(
    locations=[[min_lat, min_lon], [min_lat, max_lon], [max_lat, max_lon], [max_lat, min_lon], [min_lat, min_lon]], color="blue"
)
ls.add_to(m)

with open('realbus2.txt', 'r') as file:
    lines = file.readlines()

count = 1

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

    fg = folium.FeatureGroup(name="Time:%d" % (count), show=True).add_to(m)
    for i, (bus_id, busX, busY) in enumerate(bus_coordinates, start=1):
        folium.Marker([busY, busX], tooltip=f'Bus {bus_id}').add_to(fg)

    count+=1

folium.LayerControl().add_to(m)

# 지도를 HTML 파일로 저장
m.save('bus_map.html')