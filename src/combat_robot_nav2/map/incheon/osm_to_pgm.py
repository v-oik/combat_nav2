import xml.etree.ElementTree as ET
import pyproj
import cv2
import numpy as np
import os

def convert_osm_to_nav2(osm_file, output_name, resolution=0.1, road_width_m=5.0):
    print(f"🗺️ [{osm_file}] 변환 시작 (GPS-RViz 완벽 동기화 모드)...")

    # 1. OSM 파일 로드
    tree = ET.parse(osm_file)
    root = tree.getroot()

    # UTM 52N (한국 기준 좌표계) 설정
    proj = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
    
    nodes = {}
    x_coords, y_coords = [], []

    for node in root.findall('node'):
        lon, lat = float(node.get('lon')), float(node.get('lat'))
        x, y = proj(lon, lat)
        nodes[node.get('id')] = (x, y)
        x_coords.append(x)
        y_coords.append(y)

    # 맵의 중심 UTM 좌표 계산
    avg_x = sum(x_coords) / len(x_coords)
    avg_y = sum(y_coords) / len(y_coords)

    # 🚀 핵심: 맵 중심의 UTM 좌표를 다시 위경도(Lat, Lon)로 변환
    center_lon, center_lat = proj(avg_x, avg_y, inverse=True)

    # 맵 크기 (중심 기준 +- 200m)
    range_m = 400 
    min_x, max_x = avg_x - range_m, avg_x + range_m
    min_y, max_y = avg_y - range_m, avg_y + range_m

    width_px = int((max_x - min_x) / resolution)
    height_px = int((max_y - min_y) / resolution)
    
    # 배경: 회색(205, 자유 주행 가능)
    img = np.full((height_px, width_px), 205, dtype=np.uint8)
    thickness_px = int(road_width_m / resolution)

    for way in root.findall('way'):
        is_building = False
        for tag in way.findall('tag'):
            if tag.get('k') == 'building':
                is_building = True
                break

        pts = []
        for nd in way.findall('nd'):
            ref = nd.get('ref')
            if ref in nodes:
                x, y = nodes[ref]
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    px = int((x - min_x) / resolution)
                    py = height_px - int((y - min_y) / resolution) # Y축 반전
                    pts.append([px, py])
        
        if len(pts) > 1:
            pts = np.array(pts, np.int32).reshape((-1, 1, 2))
            if is_building:
                cv2.fillPoly(img, [pts], color=0)
            else:
                cv2.polylines(img, [pts], isClosed=False, color=255, thickness=thickness_px)

    pgm_name = f"{output_name}.pgm"
    yaml_name = f"{output_name}.yaml"
    cv2.imwrite(pgm_name, img)

    # 지도의 정중앙을 RViz (0,0)으로 배치
    origin_x = -(width_px * resolution) / 2.0
    origin_y = -(height_px * resolution) / 2.0

    yaml_content = f"""image: {pgm_name}
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_name, 'w') as f:
        f.write(yaml_content)

    print(f"✅ 맵 생성 완료: {pgm_name}, {yaml_name}")
    print(f"====================================================")
    print(f"🚨 [매우 중요] ekf.yaml 파일을 열고 아래 값을 수정하세요! 🚨")
    print(f"wait_for_datum: true")
    print(f"datum: [{center_lat:.8f}, {center_lon:.8f}, 0.0]")
    print(f"====================================================")

if __name__ == "__main__":
    # 파일명이 map.osm 이라면 그대로 두고, 아니라면 실제 파일명으로 수정하세요.
    convert_osm_to_nav2("incheon.osm", "incheon", resolution=0.05, road_width_m=6.0)