#include <Arduino.h>
#include <Servo.h>

const int SENSOR_R2 = A0;
const int SENSOR_R1 = A1;
const int SENSOR_M = A2;
const int SENSOR_L1 = A3;
const int SENSOR_L2 = A4;

uint16_t sensorValue_L2;
uint16_t sensorValue_L1;
uint16_t sensorValue_M;
uint16_t sensorValue_R1;
uint16_t sensorValue_R2;

const int THRESHOLD = 740; //传感器阈值 白色均值约980 黑色约350
const int SERVO_CENTER = 90; // 你的舵机中值
const int BASE_SPEED = 246; // 你的基础速度
const int SPEED_DIFF = 9; // 你的直线修正在这里：A电机比B电机快9

float Kp = 20.0; // 比例系数：主要控制转向大小
float Ki = 0.0; // 积分系数：循线通常为0
float Kd = 12.0; // 微分系数：越大越“阻尼”，防止抖动，太大会反应迟钝

float lastError = 0;
float errorIntegral = 0;
int lastDirectionMemory = 0;

const int servoIO = 8; //舵机控制引脚

//A组电机驱动
int A_PWM = 6; //控制速度
int A_DIR = 7; //控制方向
//B组电机驱动
int B_PWM = 5; //控制速度
int B_DIR = 4; //控制方向

Servo myservo;

//A组电机驱动控制函数
void A_Motor(int dir, int speed)
{
    digitalWrite(A_DIR, dir);
    analogWrite(A_PWM, speed);
}

//B组电机驱动控制函数
void B_Motor(int dir, int speed)
{
    digitalWrite(B_DIR, dir);
    analogWrite(B_PWM, speed);
}

void SensorInit()
{
    pinMode(SENSOR_L2, INPUT);
    pinMode(SENSOR_L1, INPUT);
    pinMode(SENSOR_M, INPUT);
    pinMode(SENSOR_R1, INPUT);
    pinMode(SENSOR_R2, INPUT);
}

void MotorInit()
{
    pinMode(A_DIR, OUTPUT);
    pinMode(A_PWM, OUTPUT);
    pinMode(B_DIR, OUTPUT);
    pinMode(B_PWM, OUTPUT);
}

void SensorRead()
{
    sensorValue_L2 = analogRead(SENSOR_L2);
    sensorValue_L1 = analogRead(SENSOR_L1);
    sensorValue_M = analogRead(SENSOR_M);
    sensorValue_R1 = analogRead(SENSOR_R1);
    sensorValue_R2 = analogRead(SENSOR_R2);
}

void SensorPrint()
{
    Serial.print(sensorValue_L2);
    Serial.print(",");
    Serial.print(sensorValue_L1);
    Serial.print(",");
    Serial.print(sensorValue_M);
    Serial.print(",");
    Serial.print(sensorValue_R1);
    Serial.print(",");
    Serial.println(sensorValue_R2);
}

void MotorTest()
{
    // NOTE: A HIGH 电平为正转 B LOW 电平为正转

    int8_t baseSpeed = 200; //基础速度值
    int8_t speedDiff = 9; //速度差值
    // 当前差速可以走直线
    A_Motor(HIGH, baseSpeed + speedDiff);
    B_Motor(LOW, baseSpeed - speedDiff);
}

void ServoTest()
{
    // 测试舵机转动并确定中间位置
    int middle = 90;
    int Min = 50;
    int Max = 130;

    myservo.write(Min);
    delay(1000);
    myservo.write(Max);
    delay(1000);
    myservo.write(middle);
    delay(1000);
}

void setup()
{
    Serial.begin(9600);
    myservo.attach(servoIO);

    SensorInit();
    MotorInit();

    myservo.write(90);
    delay(500);
}

float calculateError()
{
    // 读取传感器状态 (1表示压线/黑, 0表示白)
    int s_l2 = analogRead(SENSOR_L2) < THRESHOLD ? 1 : 0;
    int s_l1 = analogRead(SENSOR_L1) < THRESHOLD ? 1 : 0;
    int s_m = analogRead(SENSOR_M) < THRESHOLD ? 1 : 0;
    int s_r1 = analogRead(SENSOR_R1) < THRESHOLD ? 1 : 0;
    int s_r2 = analogRead(SENSOR_R2) < THRESHOLD ? 1 : 0;

    int activeSensors = s_l2 + s_l1 + s_m + s_r1 + s_r2;

    // 1. 特殊情况：丢失目标 (全白)
    // 使用记忆变量，让车继续往上次偏离的方向转
    if (activeSensors == 0)
    {
        // 如果上次偏左，这就返回一个稍大的左偏误差，迫使它转回去
        if (lastDirectionMemory > 0)
            return 2.5;
        if (lastDirectionMemory < 0)
            return -2.5;
        return 0;
    }

    // 2. 加权平均算法
    // 权重定义：L2(-2), L1(-1), M(0), R1(1), R2(2)
    // 公式：(总分 / 总人数)
    float weightedSum = (s_l2 * 2.0) + (s_l1 * 1.0) + (s_m * 0) + (s_r1 * -1.0) + (s_r2 * -2.0);
    // 注意：我根据你原代码的逻辑调整了正负号
    // 原代码：L1触发 -> steerAngle > 0 (正数) -> 舵机增加 -> 向左转
    // 所以这里 L侧 给正权重，R侧 给负权重

    float error = weightedSum / activeSensors;

    // 更新记忆变量
    if (error > 0.1)
        lastDirectionMemory = 1; // 记住偏左了
    else if (error < -0.1)
        lastDirectionMemory = -1; // 记住偏右了

    return error;
}

void loop()
{
    // 1. 获取误差 (范围 -2.0 ~ 2.0)
    float error = calculateError();

    // 2. PID 计算
    // P项
    float P = error * Kp;

    // I项 (通常不需要，限制幅度防止饱和)
    errorIntegral += error;
    errorIntegral = constrain(errorIntegral, -50, 50); // 简单的抗饱和
    float I = errorIntegral * Ki;

    // D项 (当前误差 - 上次误差)
    float D = (error - lastError) * Kd;
    lastError = error; // 更新历史

    // PID 总输出
    float pidOutput = P + I + D;

    // 3. 执行控制

    // [舵机控制]
    // 基础中心 + PID修正量
    int servoAngle = SERVO_CENTER + pidOutput * 1.25;
    servoAngle = constrain(servoAngle, 40, 140); // 物理限位
    myservo.write(servoAngle);

    // [差速控制]
    // 你的原逻辑：Left转(Output>0) -> A减速, B加速
    // 考虑到 BASE_SPEED 很大 (246)，加速空间极小(到255)，主要靠减速

    int speedAdj = abs(pidOutput); // 差速力度取绝对值
    // 这里使用一个系数来把舵机角度映射到电机速度变化
    // 假设转30度，速度变化约 50-80
    speedAdj = speedAdj * 1.5;

    int speedA = BASE_SPEED + SPEED_DIFF;
    int speedB = BASE_SPEED - SPEED_DIFF;

    if (pidOutput > 0)
    { // 向左转
        speedA -= speedAdj; // A 减速
        // speedB += speedAdj; // B 已经在高速了，再加就饱和了，所以只减速内侧轮最稳
    }
    else
    { // 向右转
        // speedA += speedAdj;
        speedB -= speedAdj; // B 减速
    }

    // 最终限幅
    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    A_Motor(HIGH, speedA);
    B_Motor(LOW, speedB);

    // PID 循环需要稳定的时间间隔，或者使用 millis() 计算 dt
    // 这里保留 delay 但改小一点，增加采样率
    delay(5);
}