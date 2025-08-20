# import socket
# import json
# import time

# HOST = '192.168.1.234'  # 로봇의 IP 주소
# PORT = 5001  # 로봇의 서버 포트

# client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# client.connect((HOST, PORT))

# print(f"Connected to {HOST}:{PORT}")
# while True:
#     # 보낼 pose 데이터 (10.0 고정)
#     pose_data = "10.0"

#     # 로봇(클라이언트)로 전송
#     client.sendall(pose_data.encode() + b'\n')  # 개행 문자 추가

#     print(f"Sent pose: {pose_data}")

#     time.sleep(1)  # 60 FPS로 전송

import socket
import time

HOST = "192.168.1.234"  # 서버 IP 주소
PORT = 5005  # 서버와 동일한 포트

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 서버가 실행될 때까지 연결 시도
while True:
    try:
        client.connect((HOST, PORT))
        print("Connected to server!")
        break  # 연결 성공 시 while 루프 종료
    except ConnectionRefusedError:
        print("Server not available, retrying in 2 seconds...")
        time.sleep(2)  # 2초 후 다시 시도
