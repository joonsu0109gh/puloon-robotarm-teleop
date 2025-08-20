import time
from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.device import ModbusDeviceIdentification

# Modbus 서버 설정
def setup_modbus_server():
    store = ModbusSlaveContext(
        hr=ModbusSequentialDataBlock(301, [0] * 100)  # Holding Register 100개 할당
    )
    context = ModbusServerContext(slaves=store, single=True)
    return context

context = setup_modbus_server()

identity = ModbusDeviceIdentification()
identity.VendorName = "MyCompany"
identity.ProductCode = "MyRobot"
identity.ProductName = "Robotic Arm"

# 초깃값 설정 (x, y, z, rx, ry, rz, grip)
x, y, z, rx, ry, rz, grip = -47, 444, 237, 180, 0, 180, 0
context[0].setValues(3, int(0x01), [x, y, z, rx, ry, rz, grip])  # 주소 0번부터 값 저장

print("Starting Modbus TCP Server...")
from threading import Thread
server_thread = Thread(target=StartTcpServer, args=(context,), kwargs={"identity": identity, "address": ("0.0.0.0", 1505)})
server_thread.start()

# 값 업데이트 루프
try:
    while True:
        time.sleep(1)

        # 새로운 값 확인 및 업데이트
        new_values = context[0].getValues(3, int(0x01), count=7)  # 7개의 값(x, y, z, rx, ry, rz, grip) 읽기
        new_x, new_y, new_z, new_rx, new_ry, new_rz, new_grip = new_values

        if (new_x, new_y, new_z, new_rx, new_ry, new_rz) != (x, y, z, rx, ry, rz):
            x, y, z, rx, ry, rz = new_x, new_y, new_z, new_rx, new_ry, new_rz
            print(f"Updated Position: X={x}, Y={y}, Z={z}, RX={rx}, RY={ry}, RZ={rz}")

        if new_grip != grip:
            grip = new_grip
            print(f"Gripper updated: {grip}")

except KeyboardInterrupt:
    print("Shutting down Modbus server...")
    server_thread.join()

finally:
    server_thread.join()