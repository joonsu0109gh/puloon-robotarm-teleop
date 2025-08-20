from pyRobotiqGripper import RobotiqGripper

try:
    grip = RobotiqGripper("/dev/ttyUSB0")
    grip.readAll()
    print("그리퍼 오류 코드:", grip.paramDic["gFLT"])
except Exception as e:
    print("오류 발생:", e)
