from pymodbus.client.sync import ModbusTcpClient

# 서버에 연결 (localhost, 포트 1502)
client = ModbusTcpClient("localhost", port=1502)

if client.connect():
    print("Modbus TCP 서버 연결 성공!")

    # Holding Register(3번 기능 코드)에서 7개 값 읽기
    response = client.read_holding_registers(1, 7)
    print(f"서버에서 읽은 값: {response.registers}")

    # update values
    new_values = [10, 20, 30, 40, 50, 60, 70]
    client.write_registers(0, new_values)
else:
    print("Modbus TCP 서버 연결 실패!")
