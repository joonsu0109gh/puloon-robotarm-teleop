import socket
import time

from pyRobotiqGripper import pyRobotiqGripper

grip = pyRobotiqGripper.RobotiqGripper("/dev/ttyUSB0")
grip.resetActivate()
# grip.printInfo()  # 현재 상태 출력


# grip.open()
# grip.close()
grip.goTo(0, 255)  # 0~255 사이의 값
  # goto(position, speed, force, blocking) # 0 = open, 255 = close
# grip.goTo(0, 100) 

def format_message(x, y, z, rx, ry, rz):
    """ 주어진 값들을 형식에 맞게 변환 """
    message = f"{x:+04d},{y:+04d},{z:+04d},{rx:+04d},{ry:+04d},{rz:+04d}"
    return message

HOST = '0.0.0.0'  # 서버가 실행될 IP 주소
PORT = 5004  # 서버 포트

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 포트 재사용 옵션 추가
server.bind((HOST, PORT))
server.listen(1)
print("Waiting for client connection...")

conn, addr = server.accept()
print(f"Connected to client: {addr}")
x, y, z, rx, ry, rz = -47, 444, 237, 180, 0, 180

try:
    message = format_message(x, y, z, rx, ry, rz)
    conn.sendall(message.encode())  # 문자열 전송
    print(f"Sent: {message}")
except Exception as e:
    print(f"Error: {e}")

time.sleep(5)
grip.goTo(255, 255, 1)

x, y, z, rx, ry, rz = -47, 444, 437, 180, 0, 180

try:
    message = format_message(x, y, z, rx, ry, rz)
    conn.sendall(message.encode())  # 문자열 전송
    print(f"Sent: {message}")
    time.sleep(1)  # 1초 대기 후 다시 전송
except Exception as e:
    print(f"Error: {e}")

conn.close()
server.close()