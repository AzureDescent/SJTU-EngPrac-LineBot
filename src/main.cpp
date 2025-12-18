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

// --- 全局变量 ---
const int THRESHOLD = 740;
const int SERVO_CENTER = 90;
const int SPEED_DIFF = 9;      // 直线修正

// --- 速度策略 (动态速度) ---
const int MAX_SPEED = 246;     // [狂飙模式] 直道最高速
const int CORNER_SPEED = 120;  // [过弯模式] 弯道基准速 (过U形弯必须慢)
const int BRAKE_SENSITIVITY = 30; // 刹车灵敏度：PID输出超过此值即判定为弯道

// --- PID 参数 (针对"直道扭动"优化) ---
// 之前的 Kp=20 太大了，导致直道震荡。
float Kp = 14.0;  // 降低 P，减小直道上的过激反应
float Ki = 0.0;   // 保持 0
float Kd = 40.0;  // [关键] 大幅提升 D。D是"阻尼器"，能抑制左右扭动，让车"粘"在线上

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

float calculateError() {
    int s_l2 = analogRead(SENSOR_L2) < THRESHOLD ? 1 : 0;
    int s_l1 = analogRead(SENSOR_L1) < THRESHOLD ? 1 : 0;
    int s_m  = analogRead(SENSOR_M)  < THRESHOLD ? 1 : 0;
    int s_r1 = analogRead(SENSOR_R1) < THRESHOLD ? 1 : 0;
    int s_r2 = analogRead(SENSOR_R2) < THRESHOLD ? 1 : 0;

    int activeSensors = s_l2 + s_l1 + s_m + s_r1 + s_r2;

    if (activeSensors == 0) {
        if (lastDirectionMemory > 0) return 2.5;
        if (lastDirectionMemory < 0) return -2.5;
        return 0;
    }

    // 权重: L2(2), L1(1), M(0), R1(-1), R2(-2)
    float weightedSum = (s_l2 * 2.0) + (s_l1 * 1.0) + (s_m * 0) + (s_r1 * -1.0) + (s_r2 * -2.0);
    float error = weightedSum / activeSensors;

    if (error > 0.1) lastDirectionMemory = 1;
    else if (error < -0.1) lastDirectionMemory = -1;

    return error;
}

void loop() {
    // 1. 获取误差
    float error = calculateError();

    // 2. PID 计算
    float P = error * Kp;
    float D = (error - lastError) * Kd;
    lastError = error;

    // PID 总输出 (代表转向猛烈程度)
    float pidOutput = P + D;

    // 3. 执行控制

    // [舵机控制]
    // 直接使用 pidOutput，不要乘系数，靠调大 Kp 来解决
    int servoAngle = SERVO_CENTER + pidOutput;
    servoAngle = constrain(servoAngle, 35, 145);
    myservo.write(servoAngle);

    // [动态速度策略] - 解决弯道冲出 + 提升整体圈速
    // 逻辑：如果 pidOutput 很大（说明在急转弯），则降低基础速度；如果在直道，全速前进

    int currentBaseSpeed = MAX_SPEED;
    float absOutput = abs(pidOutput);

    // 如果转向幅度超过阈值，开始线性减速
    if (absOutput > BRAKE_SENSITIVITY) {
        // 这是一个简单的线性映射：转向越猛，速度越慢
        // 比如 pidOutput = 50 (急弯)，速度降到 CORNER_SPEED
        float brakeFactor = (absOutput - BRAKE_SENSITIVITY) / 20.0;
        brakeFactor = constrain(brakeFactor, 0.0, 1.0);

        // 在 MAX_SPEED 和 CORNER_SPEED 之间动态切换
        currentBaseSpeed = MAX_SPEED - (MAX_SPEED - CORNER_SPEED) * brakeFactor;
    }

    // [差速控制]
    // 转向越猛，差速越大。
    // 对于 U 形弯，我们需要内侧轮极慢，甚至轻微反转

    int speedAdj = absOutput * 3.0; // 差速系数，决定了转弯时左右轮速差多大

    int speedA = currentBaseSpeed + SPEED_DIFF;
    int speedB = currentBaseSpeed - SPEED_DIFF;

    if (pidOutput > 0) { // 向左转
        speedA -= speedAdj * 0.5; // 外侧轮少加点，防止超速
        speedB += speedAdj * 1.2; // 【关键】内侧轮狠减速
    } else { // 向右转
        speedA += speedAdj * 1.2; // 【关键】内侧轮狠减速
        speedB -= speedAdj * 0.5;
    }

    // 限制范围
    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    A_Motor(HIGH, speedA);
    B_Motor(LOW, speedB);

    delay(5);
}