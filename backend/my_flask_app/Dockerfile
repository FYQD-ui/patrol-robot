# 使用正式版 Python 基础镜像
FROM python:3.10-slim

# 安装 libgl1 以满足 OpenCV 依赖
RUN apt-get update && apt-get install -y libgl1

# 设置工作目录
WORKDIR /app

# 复制当前目录所有文件到容器中
COPY . .

# 安装依赖项
RUN pip install --no-cache-dir -r requirements.txt

# 暴露端口 5000 供外部访问
EXPOSE 5000

# 设置环境变量
ENV FLASK_APP=app.py
ENV FLASK_ENV=development

# 启动 Flask 应用
CMD ["flask", "run", "--host=0.0.0.0", "--port=5000"]
