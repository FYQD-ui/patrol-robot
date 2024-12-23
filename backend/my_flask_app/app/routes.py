from flask import Blueprint, jsonify, request, Response
import socket
import threading
import time
from PIL import Image
import io
import cv2
import numpy as np
from ultralytics import YOLO
import mysql.connector
from mysql.connector import pooling, Error
from Crypto.Cipher import AES
import base64

main_blueprint = Blueprint('main', __name__)

model = YOLO("app/static/models/best.pt")  # 加载自定义YOLO模型
secret_key = 'MySecret_key_123'  # AES密钥

# 数据库配置
db_config = {
    "host": "192.168.210.128",
    "user": "yyb",
    "password": "123456",
    "database": "sat_system",
    "pool_name": "mypool",
    "pool_size": 5
}

db_pool = None  # 延迟初始化数据库连接池

# 全局变量，用于存储最新的帧和线程锁
latest_frame = None
frame_lock = threading.Lock()  # 初始化线程锁
streaming = False  # 控制流是否开启
frame_thread = None

# UDP 控制指令 socket 初始化
control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_target_ip = None
control_target_port = None

# 摄像头控制的 socket
camera_control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 标志变量，用于区分当前控制页面是否正在使用
is_control_page_active = False

def get_db_pool():
    global db_pool
    if db_pool is None:
        try:
            db_pool = pooling.MySQLConnectionPool(**db_config)
        except Error as err:
            print(f"数据库连接池初始化错误：{err}")
    return db_pool

def decrypt(encrypted_data):
    encrypted_data = base64.b64decode(encrypted_data)
    cipher = AES.new(secret_key.encode('utf-8'), AES.MODE_ECB)
    decrypted_data = cipher.decrypt(encrypted_data)
    padding_len = decrypted_data[-1]
    decrypted_data = decrypted_data[:-padding_len]
    return decrypted_data.decode('utf-8')

def receive_frames():
    global latest_frame, streaming
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
    s.bind(("0.0.0.0", 9091))
    print("Socket 已绑定到 0.0.0.0:9091")

    streaming = True
    try:
        while streaming:
            data, addr = s.recvfrom(65536)
            if data:
                try:
                    # 将接收到的字节数据转换为图像
                    img = Image.open(io.BytesIO(data))
                    img = np.array(img)
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                except Exception as e:
                    print(f"图像处理错误：{e}")
                    continue

                try:
                    # 执行 YOLO 目标检测
                    results = model(img)
                    result_img = results[0].plot()

                    # 将处理后的帧存储在全局变量中
                    with frame_lock:
                        latest_frame = result_img
                except Exception as e:
                    print(f"YOLO 模型推理错误：{e}")
                    continue
    except Exception as e:
        print(f"发生异常错误：{e}")
    finally:
        s.close()
        print("Socket 已关闭。")

def gen_frames():
    while streaming:
        with frame_lock:
            frame = latest_frame
        if frame is not None:
            # 将帧编码为 JPEG 格式
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # 使用 multipart/x-mixed-replace 格式发送帧
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.1)  # 等待下一帧

def send_command_to_robot(ip, port, cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ip, port))
            s.sendall(cmd.encode())
            print(f"指令 {cmd} 成功发送到 {ip}:{port}")
            return '命令已成功发送'
    except Exception as e:
        error_message = f'错误：{str(e)}'
        print(error_message)
        return error_message

@main_blueprint.route('/command', methods=['POST'])
def command():
    data = request.json
    ip = data.get('ip', '')
    port = data.get('port', '')
    cmd = data.get('cmd', '')
    control_page = data.get('control_page', False)

    # 检查是否提供了 IP、端口和命令
    if not ip or not port or not cmd:
        return jsonify({'status': 'error', 'message': 'IP、端口或命令缺失'}), 400

    try:
        port = int(port)
    except ValueError:
        return jsonify({'status': 'error', 'message': '无效的端口号'}), 400

    global is_control_page_active
    if control_page:
        # 如果控制页面正在使用，阻止其他页面发送命令
        if is_control_page_active:
            return jsonify({'status': 'error', 'message': '控制页面正在使用，请稍后再试'}), 400
        is_control_page_active = True
    else:
        if is_control_page_active:
            return jsonify({'status': 'error', 'message': '当前控制页面正在使用，无法发送命令'}), 400

    # 调用函数发送指令至机器人
    result_message = send_command_to_robot(ip, port, cmd)
    print(f"发送指令: {cmd} 至 {ip}:{port}, 结果: {result_message}")

    # 更新控制页面状态
    if control_page:
        is_control_page_active = False

    return jsonify({'status': 'success', 'message': result_message})

@main_blueprint.route('/camera_control', methods=['POST'])
def camera_control():
    data = request.json
    camera_ip = data.get('ip')
    camera_port = data.get('port')
    cmd = data.get('cmd')

    if not camera_ip or not camera_port or not cmd:
        return jsonify({'status': 'error', 'message': '缺少摄像头 IP、端口或命令'}), 400

    try:
        camera_port = int(camera_port)
    except ValueError:
        return jsonify({'status': 'error', 'message': '无效的端口号'}), 400

    try:
        # 使用 UDP 发送摄像头控制命令
        camera_control_socket.sendto(cmd.encode("utf-8"), (camera_ip, camera_port))
        return jsonify({'status': 'success', 'message': '摄像头控制指令已发送'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'发送控制指令时发生错误：{e}'}), 500

@main_blueprint.route('/video_feed')
def video_feed():
    global streaming, frame_thread

    # 确保视频流线程正在运行
    if not streaming:
        streaming = True
        frame_thread = threading.Thread(target=receive_frames)
        frame_thread.daemon = True
        frame_thread.start()

    # 返回 MJPEG 格式的响应
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@main_blueprint.route('/stop_feed')
def stop_feed():
    global streaming, frame_thread

    if streaming:
        streaming = False
        if frame_thread is not None:
            frame_thread.join(timeout=5)
            frame_thread = None
        print("视频流已停止，资源已释放。")

    return jsonify({"message": "视频流已停止"})

@main_blueprint.route('/signup', methods=['POST'])
def signup():
    data = request.get_json()
    encrypted_username = data.get('username')
    encrypted_email = data.get('email')
    encrypted_password = data.get('password')

    try:
        username = decrypt(encrypted_username)
        email = decrypt(encrypted_email)
        password = decrypt(encrypted_password)
    except Exception as e:
        print(f"解密错误：{e}")
        return jsonify({"error": "解密失败"}), 500

    try:
        db_pool = get_db_pool()
        if db_pool is None:
            return jsonify({"error": "无法连接到数据库，请稍后再试"}), 500
        db = db_pool.get_connection()
        cursor = db.cursor()
        insert_query = "INSERT INTO usersinfo (username, email, password) VALUES (%s, %s, %s)"
        cursor.execute(insert_query, (username, email, password))
        db.commit()
    except Error as err:
        print(f"数据库错误：{err}")
        return jsonify({"error": "无法连接到数据库，请稍后再试"}), 500
    finally:
        if 'cursor' in locals():
            cursor.close()
        if 'db' in locals():
            db.close()

    return jsonify({"message": "用户信息已成功存入数据库"})

@main_blueprint.route('/login', methods=['POST'])
def login():
    data = request.get_json()
    encrypted_email = data.get('email')
    encrypted_password = data.get('password')

    if encrypted_email is None or encrypted_password is None:
        return jsonify({"error": "缺少 email 或 password 参数"}), 400

    try:
        email = decrypt(encrypted_email)
        password = decrypt(encrypted_password)
    except Exception as e:
        print(f"解密错误：{e}")
        return jsonify({"error": "解密失败"}), 500

    try:
        db_pool = get_db_pool()
        if db_pool is None:
            return jsonify({"error": "无法连接到数据库，请稍后再试"}), 500
        db = db_pool.get_connection()
        cursor = db.cursor(dictionary=True)
        query = "SELECT password FROM usersinfo WHERE email = %s"
        cursor.execute(query, (email,))
        result = cursor.fetchone()
    except Error as err:
        print(f"数据库错误：{err}")
        return jsonify({"error": "无法连接到数据库，请稍后再试"}), 500
    finally:
        if 'cursor' in locals():
            cursor.close()
        if 'db' in locals():
            db.close()

    if result is None:
        return jsonify({"error": "账号不存在"}), 401
    elif result['password'] == password:
        return jsonify({"message": "允许进入主页面"}), 200
    else:
        return jsonify({"error": "密码错误"}), 401