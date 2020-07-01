# 这个文件读取键盘并控制vrep中的小车进行移动

import pygame as pg  # pygame 包来读取键盘
import vrep

SET_LINE_SPEED = 3.0
SET_ROTATE_SPEED = 1.0

if True:  # 初始化
    # vrep初始化
    # vrep.simxFinish(-1)  # 断开所有已经连接的remote，这里不执行，避免把matlab中的连接断开
    dr20ID = vrep.simxStart("127.0.0.1", 56666, True, True, 5000, 5)
    if dr20ID < 0:
        print("vrep_connection_failed")
        exit()
    else:
        print("vrep_connection_success")
        handle_left_wheel = vrep.simxGetObjectHandle(dr20ID, "dr20_leftWheelJoint_", vrep.simx_opmode_blocking)
        handle_right_wheel = vrep.simxGetObjectHandle(dr20ID, "dr20_rightWheelJoint_", vrep.simx_opmode_blocking)
        vrep.simxAddStatusbarMessage(dr20ID, "python_remote_connected\n", vrep.simx_opmode_oneshot)

    # pygame初始化
    pg.init()
    window = pg.display.set_mode((320, 240))
    pg.display.set_caption("vrep_car_control_window")

# 正常运行
end_flag = False
while not end_flag:
    pg.time.delay(30)

    # 显示，背景色
    window.fill((100, 100, 100))

    # pygame处理
    for event in pg.event.get():
        if event.type == pg.QUIT:
            end_flag = True

    # 检测按键，并遥控小车，并更新显示
    key_status = pg.key.get_pressed()
    window_block_position = [160 - 25, 120 - 25]
    v_line = 0.0
    v_rotate = 0.0
    if key_status[pg.K_UP] == 1:
        window_block_position[1] -= 100
        v_line += SET_LINE_SPEED
    if key_status[pg.K_DOWN] == 1:
        window_block_position[1] += 100
        v_line -= SET_LINE_SPEED
    if key_status[pg.K_LEFT] == 1:
        window_block_position[0] -= 100
        v_rotate -= SET_ROTATE_SPEED
    if key_status[pg.K_RIGHT] == 1:
        window_block_position[0] += 100
        v_rotate += SET_ROTATE_SPEED

    # 显示矩形
    pg.draw.rect(window, (255, 0, 0), (window_block_position[0], window_block_position[1], 50, 50))
    # 发送速度
    vrep.simxSetJointTargetVelocity(dr20ID, handle_left_wheel[1], v_line + v_rotate, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(dr20ID, handle_right_wheel[1], v_line - v_rotate, vrep.simx_opmode_streaming)

    pg.display.update()

pg.quit()
vrep.simxFinish(dr20ID)
print("python exit")
