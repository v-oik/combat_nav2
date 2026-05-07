import os

# 해상도 0.05m 기준, 200m x 200m 크기
WIDTH = 4000
HEIGHT = 4000
FILENAME = "incheon.pgm"

print(f"🚀 {WIDTH}x{HEIGHT} (200m x 200m) 크기의 하얀색 맵({FILENAME})을 생성합니다...")

with open(FILENAME, 'wb') as f:
    # 1. PGM(P5) 헤더 작성 (255: 빈 공간 / 흰색)
    header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
    f.write(header.encode('ascii'))
    
    # 2. 픽셀 데이터 채우기
    row = bytearray([255] * WIDTH)
    for _ in range(HEIGHT):
        f.write(row)

file_size_mb = os.path.getsize(FILENAME) / (1024 * 1024)
print(f"✅ 생성 완료! 파일 크기: 약 {file_size_mb:.2f} MB")