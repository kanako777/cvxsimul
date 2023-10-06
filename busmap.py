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

# 윈도우 생성
window = tk.Tk()
window.title("버스 위치 표시")

# 캔버스 생성
canvas = tk.Canvas(window, width=1000, height=1000)
canvas.pack()


bus_coordinates = []

# 파일에서 버스 위치 정보 읽기
with open('buspos.txt', 'r') as file:
    lines = file.readlines()

for line in lines:
    bus_info = line.strip().split('/')
    for info in bus_info:

        bus_id, xy = info.split(':')
        xx, yy = xy.split(',')

        x= float(xx)
        y= float(yy)
        temp_x = (x /500)*0.0055+127.0306
        temp_y = (y /500)*0.005 +37.4954

        bus_coordinates.append((temp_x,temp_y))
        # 화면에 버스 위치 표시
        canvas.create_oval(x-3, y-3, x + 3, y + 3, fill='blue')
        #canvas.create_text(x + 1, y, text=f'Bus {bus_id}')

# 윈도우 실행
window.mainloop()


# GPS 좌표
lon_c = 127.0361  # 여기에 실제 GPS X 좌표를 넣으세요
lat_c = 37.5004  # 여기에 실제 GPS Y 좌표를 넣으세요

# 지도를 해당 GPS 좌표로 초기화
m = folium.Map(location=[lat_c, lon_c], zoom_start=15)

print(bus_coordinates)
# 버스 위치를 지도에 표시
for i, (busX, busY) in enumerate(bus_coordinates, start=1):
    folium.Marker([busY, busX], tooltip=f'Bus {i}').add_to(m)

# 지도를 HTML 파일로 저장
m.save('bus_map.html')
