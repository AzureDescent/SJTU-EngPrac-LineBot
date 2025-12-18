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

const int PIN_L_PWM = 5;

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

void MotorInit(){
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
    int8_t baseSpeed = 200; //基础速度值
    A_Motor(HIGH, baseSpeed + speedDiff);
    B_Motor(LOW, baseSpeed - speedDiff);
}

void setup()
{
    Serial.begin(9600);

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
}