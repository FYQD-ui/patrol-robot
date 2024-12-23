import logging

from flask import Flask
from flask_cors import CORS
from app.routes import main_blueprint


def create_app():
    app = Flask(__name__)
    app.config['DEBUG'] = True  # 启用 Flask 调试模式
    CORS(app)  # 启用 CORS，允许跨域请求

    # 配置日志格式和级别
    logging.basicConfig(
        level=logging.DEBUG,  # 日志级别
        format='%(asctime)s - %(levelname)s - %(message)s'  # 日志格式
    )

    # 配置应用的设置
    app.config.from_object('config.Config')

    # 注册蓝图
    app.register_blueprint(main_blueprint)

    return app
