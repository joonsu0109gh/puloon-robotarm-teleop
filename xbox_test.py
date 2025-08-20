import pygame

# Pygame 초기화
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() < 1:
    raise RuntimeError("Xbox 컨트롤러를 찾을 수 없습니다.")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected to: {joystick.get_name()}")

# Xbox 컨트롤러 입력을 지속적으로 읽음
running = True
while running:
    pygame.event.pump()  # 이벤트 처리 (필수)

    # 🎮 스틱 값 (좌우, 상하)
    left_stick_x = joystick.get_axis(0) # left x
    left_stick_y = joystick.get_axis(1) # left y
    right_stick_x = joystick.get_axis(2)       # LT
    right_stick_y = joystick.get_axis(3)    # right x

    # 🎮 트리거 값 (LT, RT)
    left_trigger = joystick.get_axis(4) #  right x
    right_trigger = joystick.get_axis(5)    # RT

    # 🎮 버튼 값 (A, B, X, Y, LB, RB, START 등)
    A = joystick.get_button(0)
    B = joystick.get_button(1)
    X = joystick.get_button(2)
    Y = joystick.get_button(3)
    LB = joystick.get_button(4)
    RB = joystick.get_button(5)
    BACK = joystick.get_button(6)
    START = joystick.get_button(7)
    XBOX = joystick.get_button(8)
    LS = joystick.get_button(9)  # 왼쪽 스틱 클릭
    RS = joystick.get_button(10) # 오른쪽 스틱 클릭

    # 🎮 방향 패드 (D-Pad)
    dpad_x, dpad_y = joystick.get_hat(0)

    # 값 출력
    print(f"""
    Left Stick:  X={left_stick_x:.2f}, Y={left_stick_y:.2f}
    Right Stick: X={right_stick_x:.2f}, Y={right_stick_y:.2f}
    LT: {left_trigger:.2f}, RT: {right_trigger:.2f}
    A={A}, B={B}, X={X}, Y={Y}, LB={LB}, RB={RB}
    BACK={BACK}, START={START}, XBOX={XBOX}
    LS={LS}, RS={RS}
    D-Pad: X={dpad_x}, Y={dpad_y}
    """)

    pygame.time.wait(100)  # 100ms마다 갱신 (10Hz)

    # START 버튼을 누르면 종료
    if START:
        print("Exiting...")
        running = False

pygame.quit()
