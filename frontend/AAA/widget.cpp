#include "widget.h"
#include "ui_widget.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QVBoxLayout>
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>
#include <QWebEngineView>  // 添加 QWebEngineView 的头文件
#include <QLabel>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , networkManager(new QNetworkAccessManager(this))
    , videoTimer(new QTimer(this))
    , isCameraConnected(false)
    , isStreaming(false)
{
    ui->setupUi(this);
    this->setWindowTitle("小车控制系统");

    // 使用 QLabel 设置背景图片
    backgroundLabel = new QLabel(this);
    backgroundLabel->setPixmap(QPixmap(":/images/background.jpg"));
    backgroundLabel->setScaledContents(true); // 让图片自动适应 QLabel 大小
    backgroundLabel->resize(this->size());
    backgroundLabel->lower(); // 将背景标签放在最底层



    // 初始化 webEngineView
    webEngineView = new QWebEngineView(this);

    // 设置 webEngineView 用于显示 HTML 视频流
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(webEngineView);
    ui->videoContainer->setLayout(layout);  // 假设 ui 界面中有一个 videoContainer 用于放置视频流

    // 初始显示连接页面
    ui->stackedWidget->setCurrentIndex(0);
    ui->carStackedWidget->setCurrentIndex(0);

    // 连接按钮与对应的槽函数
    connect(ui->connectCameraButton, &QPushButton::clicked, this, &Widget::connectCamera);
    connect(ui->toggleVideoStreamButton, &QPushButton::clicked, this, &Widget::toggleVideoStream);
    connect(ui->backButton, &QPushButton::clicked, this, &Widget::disconnectCamera);
    connect(ui->upButton, &QPushButton::clicked, [this]() { sendCameraCommand("w"); });
    connect(ui->downButton, &QPushButton::clicked, [this]() { sendCameraCommand("s"); });
    connect(ui->leftButton, &QPushButton::clicked, [this]() { sendCameraCommand("a"); });
    connect(ui->rightButton, &QPushButton::clicked, [this]() { sendCameraCommand("d"); });

    // 连接小车的按钮与对应的槽函数
    connect(ui->connectCarButton, &QPushButton::clicked, this, &Widget::connectCar);
    connect(ui->sendCarCommandButton, &QPushButton::clicked, [this]() {
        sendCarCommand(ui->carCommandLineEdit->text());
    });
    connect(ui->go, &QPushButton::clicked, [this]() { sendCarCommand("8"); });
    connect(ui->back, &QPushButton::clicked, [this]() { sendCarCommand("2"); });
    connect(ui->leftgo, &QPushButton::clicked, [this]() { sendCarCommand("7"); });
    connect(ui->leftback, &QPushButton::clicked, [this]() { sendCarCommand("1"); });
    connect(ui->rightgo, &QPushButton::clicked, [this]() { sendCarCommand("9"); });
    connect(ui->rightback, &QPushButton::clicked, [this]() { sendCarCommand("3"); });
    connect(ui->change, &QPushButton::clicked, [this]() { sendCarCommand("x"); });
    connect(ui->stop, &QPushButton::clicked, [this]() { sendCarCommand("0"); });
}

Widget::~Widget()
{
    delete ui;
    delete webEngineView;
}

void Widget::connectCamera()
{
    QString ip = ui->cameraIP->text();
    QString port = ui->cameraPort->text();

    if (ip.isEmpty() || port.isEmpty()) {
        qDebug() << "IP 或端口不能为空";
        return;
    }

    QUrl url("http://127.0.0.1:5000/camera_control");  // 替换为您的后端地址
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QJsonObject json;
    json["ip"] = ip;
    json["port"] = port;
    json["cmd"] = "connect";

    QJsonDocument doc(json);
    QByteArray data = doc.toJson();

    QNetworkReply *reply = networkManager->post(request, data);
    connect(reply, &QNetworkReply::finished, [this, reply, ip, port]() {
        if (reply->error() == QNetworkReply::NoError) {
            qDebug() << "摄像头连接请求成功：";
            isCameraConnected = true;

            // 如果连接成功，切换到控制页面
            ui->stackedWidget->setCurrentIndex(1);

            // 使用 QWebEngineView 显示视频流
            QString videoPageUrl = "http://127.0.0.1:5000/video_feed";  // 使用一个 HTML 页面包含 <video> 标签
            webEngineView->setUrl(QUrl(videoPageUrl));
            webEngineView->show();

            isStreaming = true;
            ui->toggleVideoStreamButton->setText("关闭视频流");
        } else {
            qDebug() << "摄像头连接请求失败：" << reply->errorString();
            isCameraConnected = false;
        }
        reply->deleteLater();
    });
}

void Widget::toggleVideoStream()
{
    if (isStreaming)
    {

        // 使用 QNetworkAccessManager 发送停止视频流的请求
        QNetworkRequest request(QUrl("http://127.0.0.1:5000/stop_feed"));
        networkManager->get(request);

        // 更新按钮文本和界面
        ui->toggleVideoStreamButton->setText("打开视频流");
        webEngineView->setUrl(QUrl("about:blank"));
        webEngineView->hide();

        isStreaming = false;

    }
    else
    {

        // 清除缓存，防止缓存问题
        webEngineView->page()->profile()->clearHttpCache();
        webEngineView->page()->profile()->clearAllVisitedLinks();

        // 使用 QNetworkAccessManager 发送启动视频流的请求
        QNetworkRequest request(QUrl("http://127.0.0.1:5000/video_feed"));
        networkManager->get(request);

        // 设置视频流的 URL 并显示
        webEngineView->setUrl(QUrl("http://127.0.0.1:5000/video_feed"));
        webEngineView->show();

        ui->toggleVideoStreamButton->setText("关闭视频流");
        isStreaming = true;

    }
}

void Widget::disconnectCamera()
{
    // 停止视频流（如果正在运行）
    if (isStreaming) {
        toggleVideoStream();  // 停止视频流
    }

    // 重置连接状态
    isCameraConnected = false;
    isStreaming = false;

    // 切换回连接页面
    ui->stackedWidget->setCurrentIndex(0);

    qDebug() << "已断开摄像头连接，返回连接页面";
}

void Widget::sendCameraCommand(const QString &command)
{
    if (!isCameraConnected) {
        qDebug() << "请先连接摄像头";
        return;
    }

    // 获取用户输入的 IP 和端口
    QString ip = ui->cameraIP->text();
    QString port = ui->cameraPort->text();

    // 构造请求的 URL 指向后端的 '/camera_control' 路由
    QString backendUrl = "http://127.0.0.1:5000/camera_control";
    QUrl url(backendUrl);
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    // 创建 JSON 数据
    QJsonObject json;
    json["ip"] = ip;
    json["port"] = port;
    json["cmd"] = command;

    QJsonDocument doc(json);
    QByteArray data = doc.toJson();

    // 发送 POST 请求到后端
    QNetworkReply *reply = networkManager->post(request, data);
    connect(reply, &QNetworkReply::finished, [reply]() {
        if (reply->error() == QNetworkReply::NoError) {
            qDebug() << "指令发送成功：" << reply->readAll();
        } else {
            qDebug() << "指令发送失败：" << reply->errorString();
            qDebug() << "URL：" << reply->url().toString();
        }
        reply->deleteLater();
    });
}

void Widget::connectCar()
{
    // 获取小车的IP和端口
    QString carIp = ui->robotIP->text();
    QString carPort = ui->robotPort->text();

    if (carIp.isEmpty() || carPort.isEmpty()) {
        qDebug() << "请填写小车的 IP 和端口。";
        return;
    }

    // 切换到小车控制页面
    ui->carStackedWidget->setCurrentIndex(1);
    qDebug() << "已连接到小车 " << carIp << ":" << carPort;
}

void Widget::sendCarCommand(const QString &command)
{
    // 获取小车的IP、端口和控制指令
    QString carIp = ui->robotIP->text();
    QString carPort = ui->robotPort->text();
    QString carCommand = command.isEmpty() ? ui->carCommandLineEdit->text() : command;

    if (carIp.isEmpty() || carPort.isEmpty() || carCommand.isEmpty()) {
        qDebug() << "请填写完整的小车信息和控制指令。";
        return;
    }

    // 构造请求的 URL 指向后端的小车控制接口
    QString backendUrl = "http://127.0.0.1:5000/command";
    QUrl url(backendUrl);
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    // 创建 JSON 数据
    QJsonObject json;
    json["ip"] = carIp;
    json["port"] = carPort;
    json["cmd"] = carCommand;

    QJsonDocument doc(json);
    QByteArray data = doc.toJson();

    // 发送 POST 请求到后端
    QNetworkReply *reply = networkManager->post(request, data);
    connect(reply, &QNetworkReply::finished, [reply]() {
        if (reply->error() == QNetworkReply::NoError) {
            qDebug() << "小车指令发送成功：" << reply->readAll();
        } else {
            qDebug() << "小车指令发送失败：" << reply->errorString();
            qDebug() << "URL：" << reply->url().toString();
        }
        reply->deleteLater();
    });
}
