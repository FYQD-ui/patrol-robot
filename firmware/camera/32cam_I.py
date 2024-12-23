# esp32cam代码
import socket
import network
import camera
import time
import machine
import select

# 连接wifi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print('connecting to network...')
    wlan.connect('YYB_107', '123456798A')
    
    while not wlan.isconnected():
        pass
print('Network configuration:', wlan.ifconfig())

# 摄像头初始化
try:
    camera.init(0, format=camera.JPEG)
except Exception as e:
    camera.deinit()
    camera.init(0, format=camera.JPEG)

# 远程操作的舵机，初始化PWM控制
servo_h = machine.PWM(machine.Pin(15), freq=50)  # 水平方向舵机控制，PWM控制、GPIO12
servo_v = machine.PWM(machine.Pin(2), freq=50)  # 垂直方向舵机控制，PWM控制、GPIO14

# 定义占空比范围
min_duty = 40  # 0度时的占空比
max_duty = 115  # 180度时的占空比

def set_servo_angle(servo, angle):
    # 将角度转换为占空比
    duty = min_duty + (max_duty - min_duty) * angle // 180
    servo.duty(duty)

# 初始化舵机到中位
servo_h_angle = 90
servo_v_angle = 90
set_servo_angle(servo_h, servo_h_angle)  # 水平中位
set_servo_angle(servo_v, servo_v_angle)  # 垂直中位

# 舵机调节程序
angle_step = 15
min_angle_h = 0   # 水平最小角度 0 度
max_angle_h = 180 # 水平最大角度 180 度
min_angle_v = 15  # 垂直最小角度 15 度
max_angle_v = 90  # 垂直最大角度 90 度

# 其他设置：
camera.flip(1)
camera.mirror(1)
camera.framesize(camera.FRAME_HVGA)
camera.speffect(camera.EFFECT_NONE)
camera.saturation(0)
camera.brightness(0)
camera.contrast(0)
camera.quality(5)

# socket UDP 的创建
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
s.bind(("0.0.0.0", 9092))  # 为控制指令创建的端口
s.setblocking(False)  # 设置为非阻塞模式，避免recvfrom阻塞

# socket 用于发送视频流
video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

try:
    while True:
        # 获取图像数据并发送
        buf = camera.capture()
        video_socket.sendto(buf, ("192.168.137.1", 9091))  # 向服务器发送图像数据
        
        # 使用select检查是否有控制命令
        rlist, _, _ = select.select([s], [], [], 0)
        if rlist:
            cmd_data, addr = s.recvfrom(1024)
            if cmd_data:
                print("指令已接收: ", cmd_data.decode())
                cmd = cmd_data.decode()
                if cmd == "w" and servo_v_angle + angle_step <= max_angle_v:
                    servo_v_angle += angle_step
                    servo_v_angle = min(max(servo_v_angle, min_angle_v), max_angle_v)  # 限制范围
                    set_servo_angle(servo_v, servo_v_angle)
                elif cmd == "s" and servo_v_angle - angle_step >= min_angle_v:
                    servo_v_angle -= angle_step
                    servo_v_angle = min(max(servo_v_angle, min_angle_v), max_angle_v)  # 限制范围
                    set_servo_angle(servo_v, servo_v_angle)
                elif cmd == "a" and servo_h_angle - angle_step >= min_angle_h:
                    servo_h_angle -= angle_step
                    servo_h_angle = min(max(servo_h_angle, min_angle_h), max_angle_h)  # 限制范围
                    set_servo_angle(servo_h, servo_h_angle)
                elif cmd == "d" and servo_h_angle + angle_step <= max_angle_h:
                    servo_h_angle += angle_step
                    servo_h_angle = min(max(servo_h_angle, min_angle_h), max_angle_h)  # 限制范围
                    set_servo_angle(servo_h, servo_h_angle)
        
        time.sleep(0.05)  # 减少CPU占用率

except Exception as e:
    print(str(e))
finally:
    camera.deinit()
