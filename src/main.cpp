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

// --- 原始参数 ---
const int THRESHOLD = 740;
const int SERVO_CENTER = 90;
const int SPEED_DIFF = 9;

// --- 速度策略 (U弯特化) ---
const int MAX_SPEED = 240;     // 直道可以快
const int CORNER_SPEED = 90;   // [关键] 弯道基准速度进一步降低，保证抓地力
const int BRAKE_SENSITIVITY = 25;

// --- PID 参数 ---
float Kp = 16.0;  // 稍微加大一点P，配合反转
float Ki = 0.0;
float Kd = 55.0;  // [关键] 巨量 D，防止反转带来的瞬间抖动

// PID 变量
float lastError = 0;
int lastDirectionMemory = 0;

const int servoIO = 8; //舵机控制引脚

//A组电机驱动
int A_PWM = 6; //控制速度
int A_DIR = 7; //控制方向
//B组电机驱动
int B_PWM = 5; //控制速度
int B_DIR = 4; //控制方向

Servo myservo;

void SmartMotorA(int speed) {
    if (speed >= 0) {
        digitalWrite(A_DIR, HIGH); // 正转方向
        analogWrite(A_PWM, constrain(speed, 0, 255));
    } else {
        digitalWrite(A_DIR, LOW);  // 反转方向
        analogWrite(A_PWM, constrain(abs(speed), 0, 255));
    }
}

void SmartMotorB(int speed) {
    if (speed >= 0) {
        digitalWrite(B_DIR, LOW);  // 正转方向
        analogWrite(B_PWM, constrain(speed, 0, 255));
    } else {
        digitalWrite(B_DIR, HIGH); // 反转方向
        analogWrite(B_PWM, constrain(abs(speed), 0, 255));
    }
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

float calculateError() {
    int s_l2 = analogRead(SENSOR_L2) < THRESHOLD ? 1 : 0;
    int s_l1 = analogRead(SENSOR_L1) < THRESHOLD ? 1 : 0;
    int s_m  = analogRead(SENSOR_M)  < THRESHOLD ? 1 : 0;
    int s_r1 = analogRead(SENSOR_R1) < THRESHOLD ? 1 : 0;
    int s_r2 = analogRead(SENSOR_R2) < THRESHOLD ? 1 : 0;

    int activeSensors = s_l2 + s_l1 + s_m + s_r1 + s_r2;

    if (activeSensors == 0) {
        if (lastDirectionMemory > 0) return 3.5; // 加大丢失后的回正力度
        if (lastDirectionMemory < 0) return -3.5;
        return 0;
    }

    // [关键修改] 加大最外侧传感器的权重
    // 让 L2/R2 的一旦触发，Error 直接飙升，触发急刹和反转
    // 权重: L2(3.5), L1(1.5), M(0), R1(-1.5), R2(-3.5)
    float weightedSum = (s_l2 * 3.5) + (s_l1 * 1.5) + (s_m * 0) + (s_r1 * -1.5) + (s_r2 * -3.5);
    float error = weightedSum / activeSensors;

    if (error > 0.1) lastDirectionMemory = 1;
    else if (error < -0.1) lastDirectionMemory = -1;

    return error;
}

void loop() {
    float error = calculateError();

    float P = error * Kp;
    float D = (error - lastError) * Kd;
    lastError = error;

    float pidOutput = P + D;

    // 1. 舵机控制
    int servoAngle = SERVO_CENTER + pidOutput;
    servoAngle = constrain(servoAngle, 35, 145);
    myservo.write(servoAngle);

    // 2. 动态基准速度
    int currentBaseSpeed = MAX_SPEED;
    float absOutput = abs(pidOutput);

    // 更加激进的减速策略
    if (absOutput > BRAKE_SENSITIVITY) {
        float brakeFactor = (absOutput - BRAKE_SENSITIVITY) / 35.0; // 分母改小，刹车更灵敏
        brakeFactor = constrain(brakeFactor, 0.0, 1.0);
        currentBaseSpeed = MAX_SPEED - (MAX_SPEED - CORNER_SPEED) * brakeFactor;
    }

    // 3. 强力差速控制 (允许反转)
    // 这里的系数 3.5 意味着：如果 pidOutput 是 40 (急弯)，速度调整量就是 140
    // 如果基准速度降到了 90，内侧轮就会变成 90 - 140 = -50 (反转!)
    int speedAdj = absOutput * 3.2;

    // A是右轮，B是左轮
    // 基础修正 SPEED_DIFF (9)
    int speedA = currentBaseSpeed + SPEED_DIFF;
    int speedB = currentBaseSpeed - SPEED_DIFF;

    if (pidOutput > 0) { // 向左转 (Left Turn)
        // 外侧轮(A/左) 加速不宜过多，防止冲出去
        speedA -= speedAdj * 0.3;
        // 内侧轮(B/右) 疯狂减速甚至反转
        speedB += speedAdj * 1.2;
    } else { // 向右转 (Right Turn)
        // 内侧轮(A/左) 疯狂减速甚至反转
        speedA += speedAdj * 1.2;
        // 外侧轮(B/右) 加速不宜过多
        speedB -= speedAdj * 0.3;
    }

    // 这里不再使用 constrain(0, 255)，而是允许负数传入 SmartMotor
    // SmartMotor 内部会处理 abs() 和方向
    SmartMotorA(speedA);
    SmartMotorB(speedB);

    delay(5);
}