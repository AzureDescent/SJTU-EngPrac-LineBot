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
const int BASE_SPEED = 200; // 你的基础速度
const int SPEED_DIFF = 9; // 【关键】你的直线修正在这里：A电机比B电机快9

// 转向参数
const int TURN_LIMIT = 40; // 最大转向角度 (40度)
const float KP = 1.8; // 转向灵敏度 (1.0~2.0之间调整)
// 差速力度 = 角度 * KP。例如转30度时，电机速度改变 36

// 记忆变量 (-1:偏左, 1:偏右, 0:直行)
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
    // A_Motor(HIGH, 200); //A电机正转，速度值200
    // B_Motor(LOW, 200); //B电机正转，速度值200

    // 根据实际情况调整小车转速差，使得小车能直线前进
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

void loop()
{
    bool l2 = analogRead(SENSOR_L2) < THRESHOLD;
    bool l1 = analogRead(SENSOR_L1) < THRESHOLD;
    bool m = analogRead(SENSOR_M) < THRESHOLD;
    bool r1 = analogRead(SENSOR_R1) < THRESHOLD;
    bool r2 = analogRead(SENSOR_R2) < THRESHOLD;

    double steerAngle = 0;

    if (m)
    {
        steerAngle = 0;
        lastErrorDirection = 0;
    }
    else if (l1)
    {
        steerAngle = TURN_LIMIT * 0.6;
        lastErrorDirection = 1;
    }
    else if (r1)
    {
        steerAngle = -TURN_LIMIT * 0.6;
        lastErrorDirection = -1;
    }
    else if (l2)
    {
        steerAngle = TURN_LIMIT;
        lastErrorDirection = 1;
    }
    else if (r2)
    {
        steerAngle = -TURN_LIMIT;
        lastErrorDirection = -1;
    }
    else
    {
        steerAngle = TURN_LIMIT * 1.2 * lastErrorDirection;
    }

    int finalServoAngle = SERVO_CENTER + steerAngle;
    finalServoAngle = constrain(finalServoAngle, 45, 135);
    myservo.write(finalServoAngle);

    int turnAdj = steerAngle * KP;

    int speedA = (BASE_SPEED + SPEED_DIFF) - turnAdj;

    int speedB = (BASE_SPEED - SPEED_DIFF) + turnAdj;

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    A_Motor(HIGH, speedA);
    B_Motor(LOW, speedB);

    delay(10);
}