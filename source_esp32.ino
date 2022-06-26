/*
  Quadruped robot arduino sketch.
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.

  [source] This is the main file that manages [kinematics] & [hardware]
  all the important parameters are set in this file.

  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/

#include "datatypes.h" //資料型態標頭黨

#include <Adafruit_PWMServoDriver.h>//利用PWM來控制轉向(利用戰空比不同的方波來輸出軸機轉向)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*#include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
  MPU6050 mpu;*/

#include <PS4Controller.h> //使用PS4做控制 

/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//> stores the frequency of the loop function
const float frequency = 440.0; // ## Hz 宣告常數浮點數頻率，單位Hz

/// Kinematics Parameters 運動學參數

//: stores the location, rotation and scale of the main [body]
//儲存主體的位置、角度和大小
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## 位置單位{mm, mm, mm}
  {0, 0, 0},   // ## 角度單位{deg, deg, deg}
  {300, 40, 180} // ## 大小單位{mm, mm, mm}
};

//: stores the parent joint location relative to the [body]
//儲存對於主體的父關節位置
const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0}, // ## 位置單位{mm, mm, mm}
  { +50, 0, 0}, // ## 位置單位{mm, mm, mm}
  { +50, 0, 0}, // ## 位置單位{mm, mm, mm}
  { -50, 0, 0}  // ## 位置單位{mm, mm, mm}
};
const float bone_length = 105; //宣告浮點數骨骼長度 ##單位 mm

//: high level parameters for the step function
//階躍函數的高級參數
const datatypes::Vector step_extent = {40, 40, 26}; // ##單位 {mm, mm}
float vrt_offset = - 16.50; //垂直位移 ##單位 mm
float hrz_offset = - 6.00; //水平位移 ##單位 mm

float base_offset[] = { 0, -1, 0, -2};//基礎位移
const float precision = 0.001; //宣告償付浮點數精確值 ## 單位mm

void setup() {
  Serial.begin(115200);//鮑率

  init_hardware();//初始化硬件
  init_input();//初始化輸入
}

//: those local variables control the step direction and period
//局部變量控制步進方向和周期
datatypes::Vector2D _direction = {0, 0};
float turn = 0; //> indicates the direction of rotation 旋轉的方向
float height = 0; //> indicates the leg extension 腿部的伸長量

int state = 0; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
// 表示類型，(0) 空閒，(1) 小跑，(2) 橫擺，(3) 俯仰滾動，(4) 物體檢測
float _period = 10.0; //> indicates the number of steps every second
//每秒的步數

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body
//儲存了本體的相對旋轉

unsigned long duration;//宣告無正負號的長整數(時間變數)
int sample_sum, sample_num = 10,//宣告樣本和樣本數
                sample_index;//樣本索引
float freq;//浮點數 頻率

void loop() {
  duration = millis();//回傳持續時間千分之一秒

  handle_hardware();//句柄硬件
  handle_kinematics(_direction, turn, height, _period);//設定(位置,旋轉方向,腿部伸展,每秒步數)

  handle_input();//句柄輸入

  if (Serial.available())//檢查序列阜有無數據(如果有則執行)
    handle_serial();

  // this code gets the frequency of the loop function
  // 獲取循環函數的頻率
  /*sample_sum += 1000.0 / (millis() - duration);
    sample_index++;

    if (sample_index > sample_num) {
    freq = sample_sum / sample_num;
    Serial.println(freq);
    sample_sum = 0;
    sample_index = 0;
    }*/
}

float vo, ho;//宣告浮點數
void init_input() {
  PS4.begin("F8:C3:9E:3F:F8:10"); // !! replace with your own DualShock4 Controller Bluetooth MAC address
  //替換成自己的 DualShock4 控制器藍牙 MAC 位址 
  vo = vrt_offset;//指定vo為垂直位移
  ho = hrz_offset;//指定ho為水平位移
}

bool _tb = false;//判斷狀態是否執行
float stick_min = 6.f;
float lx, ly, rx, ry;
void handle_input() {
  if (PS4.isConnected()) {//如果PS4控制有連上
    lx = inter(lx, PS4.data.analog.stick.lx / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick  獲取左模擬搖桿的插值 x 位置
    ly = inter(ly, PS4.data.analog.stick.ly / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick  獲取左模擬搖桿的插值 y 位置
    rx = inter(rx, PS4.data.analog.stick.rx / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick  獲取右側模擬搖桿的插值 x 位置
    ry = inter(ry, PS4.data.analog.stick.ry / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick  獲取右側模擬搖桿的插值 y 位置

    if (abs(lx) > stick_min) { //> checks whether the stick position is out of the deadzone  檢查左邊的X位置是否在死角
      float x0 = lx - stick_min * sign(lx); //> subtracts the deadzone  減去死角角度做修正
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> checks whether the stick position is out of the deadzone  檢查左邊的Y位置是否在死角
      float y0 = ly - stick_min * sign(ly); //> subtracts the deadzone  減去死角角度做修正
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> checks whether the stick position is out of the deadzone  檢查右邊的X位置是否在死角
      float x1 = rx - stick_min * sign(rx); //> subtracts the deadzone  減去死角角度做修正
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> checks whether the stick position is out of the deadzone  檢查右邊的Y位置是否在死角
      float y1 = ry - stick_min * sign(ry); //> subtracts the deadzone  減去死角角度做修正
      height = y1;
    } else height = 0;
  }

  if (PS4.data.button.touchpad) { //> checks the touchpad state  檢查觸控板狀態
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;//如果state超過4，則跳回0
    }
  } else _tb = true;
}

// !! make sure you have enabled Newline or Carriage return
// 確保已啟用換行或回車
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input
//模式 (0) 用於校準和測試，(1) 使用序列阜作為輸入
void handle_serial() {
  //: reads and stores the serial data  讀取並儲存數據
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 13 || c == 32 || c == '\n') {//如果c=13或是c=32或是c是換行則
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0)//如果模式在0，則執行修正和測試
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)//如果模式在1，則使用序列阜當作輸入
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

//: this is an interpolation function used to smooth
// 用於平滑的插值函數
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) {
  //: properties 0 is used to calibrate the joints
  //  狀態0用於校準關節
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo);
  }
  //: properties 1 is used for small adjustments to balance the weight
  //  狀態1用於小調整以平衡權重
  else if (properties == 1) {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount;
  }
}
