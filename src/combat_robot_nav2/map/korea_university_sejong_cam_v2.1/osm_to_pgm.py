import xml.etree.ElementTree as ET
import pyproj
import cv2
import numpy as np
import os

def convert_osm_to_nav2(osm_file, output_name, resolution=0.1, road_width_m=5.0):
    print(f"🗺️ [{osm_file}] 변환 시작 (전술로봇 최적화 모드)...")

    # 1. OSM 파일 로드
    tree = ET.parse(osm_file)
    root = tree.getroot()

    # UTM 52N (한국 기준 좌표계) 설정
    proj = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
    
    nodes = {}
    x_coords, y_coords = [], []

    # 모든 노드 좌표 추출
    for node in root.findall('node'):
        lon, lat = float(node.get('lon')), float(node.get('lat'))
        x, y = proj(lon, lat)
        nodes[node.get('id')] = (x, y)
        x_coords.append(x)
        y_coords.append(y)

    # 🚀 핵심: 맵의 중심점 계산
    avg_x = sum(x_coords) / len(x_coords)
    avg_y = sum(y_coords) / len(y_coords)

    # 🚀 맵 크기 제한: 중심에서 +- 150m (총 300m x 300m)
    # 너무 크면 RViz 메모리 에러가 발생하므로 적절히 조절됨
    range_m = 150 
    min_x, max_x = avg_x - range_m, avg_x + range_m
    min_y, max_y = avg_y - range_m, avg_y + range_m

    width_px = int((max_x - min_x) / resolution)
    height_px = int((max_y - min_y) / resolution)
    
    # 배경: 검은색 (0 = 장애물)
    img = np.zeros((height_px, width_px), dtype=np.uint8)
    thickness_px = int(road_width_m / resolution)

    # 2. 도로 그리기 (Way 추출)
    for way in root.findall('way'):
        pts = []
        for nd in way.findall('nd'):
            ref = nd.get('ref')
            if ref in nodes:
                x, y = nodes[ref]
                # 맵 범위 안의 데이터만 처리
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    px = int((x - min_x) / resolution)
                    py = height_px - int((y - min_y) / resolution) # Y축 반전
                    pts.append([px, py])
        
        if len(pts) > 1:
            pts = np.array(pts, np.int32).reshape((-1, 1, 2))
            # 하얀색 도로 그리기 (255 = 주행 가능)
            cv2.polylines(img, [pts], isClosed=False, color=255, thickness=thickness_px)

    # 3. 파일 저장
    pgm_name = f"{output_name}.pgm"
    yaml_name = f"{output_name}.yaml"
    cv2.imwrite(pgm_name, img)

    # 🚀 Origin 설정: 로봇(0,0)이 지도의 한가운데 오도록 설정
    origin_x = -(width_px * resolution) / 2
    origin_y = -(height_px * resolution) / 2

    yaml_content = f"""image: {pgm_name}
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_name, 'w') as f:
        f.write(yaml_content)

    print(f"✅ 생성 완료: {pgm_name}, {yaml_name}")
    print(f"📍 맵 크기: {width_px}x{height_px} px ({range_m*2}m x {range_m*2}m)")
    print(f"📍 설정된 Origin: {origin_x}, {origin_y}")

if __name__ == "__main__":
    # 파일명이 다르면 아래 이름을 수정하세요
    convert_osm_to_nav2("lanelet2_map.osm", "sejong_clean_map", resolution=0.05, road_width_m=6.0)