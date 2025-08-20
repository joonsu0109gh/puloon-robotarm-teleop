from pyRobotiqGripper import RobotiqGripper

grip = RobotiqGripper("/dev/ttyUSB0")
# grip.resetActivate()
# grip.printInfo()  # 현재 상태 출력


# grip.open()
# grip.close()
grip.goTo(0)  # 0~255 사이의 값
grip.goTo(255, 100)  # goto(position, speed, force, blocking) # 0 = open, 255 = close
# grip.goTo(0, 100) 
position_in_bit = grip.getPosition()
print(position_in_bit)