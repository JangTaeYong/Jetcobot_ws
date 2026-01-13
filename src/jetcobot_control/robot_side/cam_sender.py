import cv2
import socket
import numpy as np
import sys

# ==========================================
# 1. 설정 (이 부분을 본인 환경에 맞게 수정)
# ==========================================
PC_IP = "192.168.0.13"  # 리눅스 PC의 IP 주소 (hostname -I로 확인)
PORT = 9505             # GUI 수신 포트와 일치해야 함
QUALITY = 70            # JPEG 압축 품질 (0~100, 낮을수록 빠르지만 화질 저하)

# UDP 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 2. 카메라 설정 (최신 OS용 V4L2 백엔드 사용)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# 해상도 및 프레임 속도 최적화 (640x480은 속도와 화질의 베스트 밸런스)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ 에러: 카메라를 열 수 없습니다.")
    print("팁: ls /dev/video* 명령어로 카메라 번호를 확인하거나 케이블을 점검하세요.")
    sys.exit()

print(f"🚀 UDP 스트리밍 시작 -> PC 주소: {PC_IP}:{PORT}")
print("중단하려면 Ctrl+C를 누르세요.")

# [Image of UDP packet structure for video streaming showing header and JPEG payload]

try:
    while True:
        # 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 읽지 못했습니다.")
            break

        # 3. 데이터 압축 (JPEG)
        # 원본 대비 용량을 1/10 이하로 줄여 전송 속도를 극대화합니다.
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
        
        # 4. 데이터 전송
        # UDP 한계 크기(65507 바이트)를 넘지 않는지 확인
        data = buffer.tobytes()
        if len(data) < 65507:
            sock.sendto(data, (PC_IP, PORT))
        else:
            print(f"⚠️ 경고: 프레임이 너무 큼 ({len(data)} bytes)")

except KeyboardInterrupt:
    print("\n👋 사용자에 의해 스트리밍이 중단되었습니다.")
finally:
    # 리소스 해제
    cap.release()
    sock.close()
    print("🔌 카메라 및 소켓 연결 종료.")