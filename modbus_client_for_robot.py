from pymodbus.client.sync import ModbusTcpClient

# 서버에 연결 (localhost, 포트 1502)
client = ModbusTcpClient("192.168.1.20", port=1502)
# target_pose = [423, -88, 205, 176, 3, -86, 0]
# target_pose = [439, -94, 213, -176,4, -93, 0]
target_pose = [2,2,3,4,5,6,7]

if client.connect():
    print("Modbus TCP 서버 연결 성공!")

    # Holding Register(3번 기능 코드)에서 7개 값 읽기
    response = client.read_holding_registers(500, 7)
    print(f"서버에서 읽은 값: {response.registers}")

    # update values
    client.write_registers(500, target_pose)
else:
    print("Modbus TCP 서버 연결 실패!")


if response[-1] == 1:
    print("Gripper is open")