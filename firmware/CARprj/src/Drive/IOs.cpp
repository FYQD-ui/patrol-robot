#include "IOs.h"

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE,22,21);  // OLED -> 2页缓存
Adafruit_NeoPixel pixels(WS_COUNT,WS_PIN, NEO_GRB + NEO_KHZ800);        // WS2812B

void InitIOs(){
    Serial.println(F("** Configure IOs..."));

    //旋转编码器
    pinMode(EC_A,INPUT);
    pinMode(EC_K,INPUT);
    pinMode(EC_B,INPUT);
    Serial.println(F("  -----EC Init Success!"));

    //蜂鸣器
    pinMode(Buzzerpin,OUTPUT);
    digitalWrite(Buzzerpin,0);
    Serial.println(F("  -----BEEP Init Success!"));

    //WS2812B
    pixels.begin();
    // pixels.show();            // Turn OFF all pixels ASAP
    // pixels.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    Serial.println(F("  -----WS2812B Init Success!"));

    //OLED
    u8g2.begin();                           //u8g2初始化
    u8g2.setFontDirection(2);               //设置字体方向：180  
    u8g2.enableUTF8Print();                 //编码设置
    u8g2.setFont(u8g2_font_8x13_me);        //设置字体
    Serial.println(F("  -----OLED Init Success!"));

    Serial.println(F("[IOs]\tInit Success!"));
}

void EC_Debug(){
  Serial.print(digitalRead(EC_A));Serial.print(F(" - "));
  Serial.print(digitalRead(EC_K));Serial.print(F(" - "));
  Serial.print(digitalRead(EC_B));Serial.println();
}

void OLED_Debug(){
  // 已知旋转180°
  u8g2.firstPage();        // 标志图像循环的开始
  do{
    u8g2.setCursor(128,48);  u8g2.print("Hello World");

  }while(u8g2.nextPage());  // 标志图像循环的结束
}

void BlinkLed(uint8_t lelpin,uint8_t blinknum){
  uint8_t i;
  for(i = 0;i < blinknum;i++){
    digitalWrite(lelpin,0);delay(80);
    digitalWrite(lelpin,1);delay(80);
  }
}

int Motor_freq_PWM        = 2000;         // PWM频率
int Motor_resolution_PWM  = 10;           // PWM分辨率，取值为 0-20 之间，10即2^10,1024

//频道0-16，不要和其他timer冲突，绑定io
void PWM_Init(int PWM_Channel, int PWM_IO){
  pinMode(PWM_IO, OUTPUT);
  ledcSetup(PWM_Channel, Motor_freq_PWM, Motor_resolution_PWM);   // 设置通道
  ledcAttachPin(PWM_IO, PWM_Channel);                             // 将通道绑定到指定IO口上
}

//pwm的占空比
void PWM_Control(int PWM_Channel, int DutyA){

  ledcWrite(PWM_Channel, DutyA);
}