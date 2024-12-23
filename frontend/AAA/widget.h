#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QTimer>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QWebEngineView>
#include <QWebEngineProfile>
#include <QLabel>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void connectCamera();  // 连接摄像头
    void toggleVideoStream(); // 开启/关闭视频流
    void disconnectCamera(); // 断开摄像头连接并返回连接页面
    void sendCameraCommand(const QString &command); // 发送摄像头控制指令
    void connectCar();     // 连接小车
    void sendCarCommand(const QString &command); // 发送小车控制指令


private:
    Ui::Widget *ui;
    QNetworkAccessManager *networkManager; // 用于发送HTTP请求的网络管理器
    QTimer *videoTimer; // 用于定时刷新摄像头画面
    QMediaPlayer *mediaPlayer; // 用于播放视频流
    QVideoWidget *videoWidget; // 显示视频的组件
    QWebEngineView *webEngineView; // 声明 webEngineView
    QLabel *backgroundLabel; // 声明 backgroundLabel 作为类的成员变量
    bool isCameraConnected; // 摄像头是否已连接
    bool isStreaming; // 视频流是否在播放
};
#endif // WIDGET_H
