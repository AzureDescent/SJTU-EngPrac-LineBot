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
const int SERVO_CENTER = 90;   // 舵机物理中值 (根据你的ServoTest调整)
const int BASE_SPEED = 180;    // 基础巡航速度 (不要设太快，推荐150-200)
const int TURN_LIMIT = 35;     // 舵机最大转向修正角度
const float KP = 1.5;          // 差速系数: 越大转弯时两侧轮速差越大

int lastErrorDirection = 0;

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

    // 测试电机正转转速差
    A_Motor(HIGH, 200); //A电机正转，速度值200
    B_Motor(LOW, 200); //B电机正转，速度值200

    // 根据实际情况调整小车转速差，使得小车能直线前进
    int8_t speedDiff = 0; //速度差值
    int8_t baseSpeed = -8.5; //基础速度值
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
}

void loop()
{
    // int32_t ticks = millis();

    // 测试传感器读取和打印
    // SensorRead();
    // SensorPrint();
    // delay(50);

    // 测试电机驱动
    MotorTest();

    // 测试舵机转动
    // ServoTest();
    // digitalRead(servoIO);
}