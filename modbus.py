import time
from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.device import ModbusDeviceIdentification
from threading import Thread


def setup_modbus_server():
    store = ModbusSlaveContext(
        hr=ModbusSequentialDataBlock(0, [0] * 100)
    )
    context = ModbusServerContext(slaves=store, single=True)
    return context

context = setup_modbus_server()

identity = ModbusDeviceIdentification()
identity.VendorName = "MyCompany"
identity.ProductCode = "MyRobot"
identity.ProductName = "Robotic Arm"


x, y, z, rx, ry, rz, grip = -47, 444, 237, 180, 0, 180, 0
context[1].setValues(3, 0, [x, y, z, rx, ry, rz, grip]) 

print("Starting Modbus TCP Server...")

def run_server():
    try:
        StartTcpServer(context, identity=identity, address=("0.0.0.0", 1502))
    except Exception as e:
        print(f"서버 실행 중 오류 발생: {e}")

server_thread = Thread(target=run_server, daemon=True)
server_thread.start()

print("Modbus TCP Server Started. Waiting for client connections...")

try:
    while True:
        time.sleep(1)
        new_values = context[1].getValues(3, 0, count=7)
        print(f"현재 저장된 값: {new_values}")
except KeyboardInterrupt:
    print("\nShutting down Modbus server...")
finally:
    print("Modbus 서버 종료 완료.")

server_thread.join()