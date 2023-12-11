#include <MsTimer2.h>
#include <Wire.h>

// 오른쪽 모터 핀
#define motorRightEnc 2
#define motorRightEnable 5
#define motorRightPwm 6
#define motorRightDir 7
// 오른쪽 모터 핀
#define motorLeftEnc 3
#define motorLeftEnable 10
#define motorLeftPwm 11
#define motorLeftDir 12
// 모터 관련 변수
int enc_pulse_right = 0;
int enc_cnt_right = 0;
int enc_pulse_left = 0;
int enc_cnt_left = 0;

// pid제어기 변수
// 오른쪽용 PID제어기 제어 이득
const double right_proportion = 3;
const double right_integral = 1.5;
const double right_derivative = 1;
// 오른쪽용 PID제어기 제어 이득
const double left_proportion = 3;
const double left_integral = 1.5;
const double left_derivative = 1;

// PID 오차값 전역변수로 선언
// 오른쪽 모터용
float err_right_P = 0;
float err_right_I = 0;
float err_right_D = 0;
float err_right_B = 0;
// 왼쪽 모터용
float err_left_P = 0;
float err_left_I = 0;
float err_left_D = 0;
float err_left_B = 0;

// sw 변수
#define sw_down 22
#define sw_up 23

// 조이스틱 핀
#define joy_stick_x 1
#define joy_stick_y 0

// 초음파 센서 핀
#define trig 26
#define echo 27
int duration, distance;

// 자이로 센서
const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;

const double RADIAN_TO_DEGREE = 180 / 3.141592;
const double DEGREE_PER_SECOND = 32767 / 250;
const double ALPHA = 1 / (1 + 0.04);

unsigned long now = 0;
unsigned long past = 0;
double dt = 0;

double baseAcX, baseAcY, baseAcZ;
double baseGyX, baseGyY, baseGyZ;

// 무게 센서
#define weights 2
int is_weight = 0

// 기타 변수
int speed = 0;
int target_right = 0;
int target_left = 0;
int right_pid_val = 0;
int left_pid_val = 0;

// 외부 인터럽스 서비스 루틴
// 오른쪽 모터 인코더 펄스 카운트 함수
void motorRightEncoderPulseRead()
{
  enc_pulse_right++;
}
// 왼쪽 모터 인코더 펄스 카운트 함수
void motorLeftEncoderPulseRead()
{
  enc_pulse_left++;
}

// 펄스 초기화 함수
void encoderRightInitial()
{
  enc_cnt_right = enc_pulse_right;
  enc_pulse_right = 0;
}
void encoderLeftInitial()
{
  enc_cnt_left = enc_pulse_left;
  enc_pulse_left = 0;
}

// 스피드-목표값 함수
void speedToTargetR(int speed)
{
  target_right = speed;
}
void speedToTargetL(int speed)
{
  target_left = speed;
}

// PID 함수
void motorRightEncoderToPID()
{
  // PID 에러 계산
  err_right_P = enc_cnt_right - target_right;
  err_right_I += err_right_P;
  err_right_D = err_right_B - err_right_P;
  err_right_B = err_right_P;

  // PID 제어기
  right_pid_val = ((err_right_P * right_proportion) + (err_right_I * right_integral) + (err_right_D * right_derivative));

  // PID 제어기 한계값 지정
  // -255 ~ 255
  if (right_pid_val >= 255)
    right_pid_val = 255;
  if (right_pid_val <= -255)
    right_pid_val = -255;

  // PID값을 이용하여 모터 제어

  PIDToMotorR(right_pid_val);
}
void motorLeftEncoderToPID()
{
  // PID 에러 계산
  err_left_P = enc_cnt_left - target_left;
  err_left_I += err_left_P;
  err_left_D = err_left_B - err_left_P;
  err_left_B = err_left_P;

  // PID 제어기
  left_pid_val = ((err_left_P * left_proportion) + (err_left_I * left_integral) + (err_left_D * left_derivative));

  // PID 제어기 한계값 지정
  // -255 ~ 255
  if (left_pid_val >= 255)
    left_pid_val = 255;
  if (left_pid_val <= -255)
    left_pid_val = -255;

  // PID값을 이용하여 모터 제어

  PIDToMotorL(left_pid_val);
}

// 모터 제어 함수
void PIDToMotorR(int motor_right_speed)
{
  if (motor_right_speed < 0)
  {
    digitalWrite(motorRightEnable, LOW);
    analogWrite(motorRightPwm, abs(motor_right_speed));
  }
  else
  {
    digitalWrite(motorRightEnable, LOW);
    analogWrite(motorRightPwm, 0);
  }
}
void PIDToMotorL(int motor_left_speed)
{
  if (motor_left_speed < 0)
  {
    digitalWrite(motorLeftEnable, LOW);
    analogWrite(motorLeftPwm, abs(motor_left_speed));
  }
  else
  {
    digitalWrite(motorLeftEnable, HIGH);
    analogWrite(motorLeftPwm, 0);
  }
}

// 타이머 인터럽트 서비스 루틴 함수
void timerInterrupt()
{
  encoderRightInitial();
  encoderLeftInitial();
  motorRightEncoderToPID();
  motorLeftEncoderToPID();
}

// 조이스틱 제어 함수
void joystickToSpeed()
{
  // 전진
  if (analogRead(joy_stick_y) > 640)
  {
    digitalWrite(motorRightDir, LOW);
    digitalWrite(motorLeftDir, LOW);
    if ((analogRead(joy_stick_y) >= 640 && analogRead(joy_stick_y) <= 768))
    {
      speed = 10;
    }
    else if ((analogRead(joy_stick_y) > 768 && analogRead(joy_stick_y) <= 896))
    {
      speed = 20;
    }
    else if ((analogRead(joy_stick_y) > 896 && analogRead(joy_stick_y) <= 1023))
    {
      speed = 30;
    }
  }
  // 후진
  else if (analogRead(joy_stick_y) < 384)
  {
    digitalWrite(motorRightDir, HIGH);
    digitalWrite(motorLeftDir, HIGH);
    if (analogRead(joy_stick_y) > 256 && analogRead(joy_stick_y) <= 384)
    {
      speed = 30;
    }
    else if (analogRead(joy_stick_y) > 128 && analogRead(joy_stick_y) <= 256)
    {
      speed = 60;
    }
    else if (analogRead(joy_stick_y) >= 0 && analogRead(joy_stick_y) <= 128)
    {
      speed = 90;
    }
  }
  else
  {
    speed = 0;
    digitalWrite(motorRightEnable, HIGH);
    analogWrite(motorRightPwm, 0);
    digitalWrite(motorLeftEnable, HIGH);
    analogWrite(motorLeftPwm, 0);
  }
}

// 초음파 센서 제어 함수
void sonarToRobot()
{
  digitalWrite(trig, LOW);
  delay(2);
  digitalWrite(trig, HIGH);
  delay(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);

  distance = duration * 17 / 1000;

  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= 15)
  {
    speed = 0;
    analogWrite(motorRightPwm, 0);
    analogWrite(motorLeftPwm, 0);
    speedToTargetR(speed);
    speedToTargetL(speed);
  }
  else
  {
    speedToTargetR(speed);
    speedToTargetL(speed);
  }
}

// MPU6050 초기화 함수
void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// MPU6050 데이터 저장 함수
void getMpuData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// 시간 측정 함수
void getElapsedTime() {
  now = millis();
  dt = (now - past) / 1000.0;
  past = now;
}

// MPU6050 초기값 저장 함수 
void calibrateSensor() {
  double sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  getMpuData();
  for (int i = 0; i < 10 ; i++) {
    getMpuData();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }
  baseAcX = sumAcX / 10;
  baseAcY = sumAcY / 10;
  baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10;
  baseGyY = sumGyY / 10;
  baseGyZ = sumGyZ / 10;
}

// 가속도 -> 각도 변환 함수
void calcAccel()
{
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
  angleAcY *= RADIAN_TO_DEGREE;
}

// 회전 각속도 -> 각도 변환 함수
void calcGyroToAngle()
{
  angleGyY = ((GyY - baseGyY) / DEGREE_PER_SECOND) * dt;
}

// 상보 필터
void highLowfillter()
{
  // 자이로 각도 적분
  double angleTmp = angleFiY + angleGyY;
  // 상보 필터
  angleFiY = ALPHA * angleTmp + (1.0 - ALPHA) * angleAcY;
}

// 무게 센서 데이터 측정 함수
void getWeightData()
{
  is_weight = analogRead(weights);

  // 감지되는 무게가 있을 때
  if (is_weight < 1023)
  {
    speedToTargetR(0);
    speedToTargetL(0);
  }

  // 감지되는 무게가 없어 졌을 때
  else
  {
    delay(3000);
  }
}

// 전진
void forward()
{
  digitalWrite(motorRightDir, LOW);
  digitalWrite(motorLeftDir, LOW);
  speedToTargetR(speed);
  speedToTargetL(speed);

// 후진
void backward()
{
  digitalWrite(motorRightDir, HIGH);
  digitalWrite(motorLeftDir, HIGH);
  speedToTargetR(speed);
  speedToTargetL(speed);
}

// 좌회전
turnLeft(int wantAngle)
{
  digitalWrite(motorRightDir, LOW);
  digitalWrite(motorLeftDir, HIGH);
  speedToTargetR(speed);
  speedToTargetL(speed);

  // 회전 각도가 원하는 각도가 되면 정지
  if (angleFiY == wantAngle)
  {
    speedToTargetR(0);
    speedToTargetL(0);
  }
}

// 우회전
void turnRight(int wantAngle)
{
  digitalWrite(motorRightDir, HIGH);
  digitalWrite(motorLeftDir, LOW);
  speedToTargetR(speed);
  speedToTargetL(speed);

  // 회전 각도가 90도가 되면 정지
  if (angleFiY == wantAngle)
  {
    speedToTargetR(0);
    speedToTargetL(0);
  }
}

// 정지
void stop()
{
  digitalWrite(motorRightDir, LOW);
  digitalWrite(motorLeftDir, LOW);
  speedToTargetR(0);
  speedToTargetL(0);
}


// 로봇 루트 설정 함수
void setRobotRoute(int route)
{
  // 루트 1번
  if (route == 1)
  {
    forward(0);
    delay(1000);
    turnLeft();
    forward();
    delay(5000);
    turnRight();
    forward();
    delay(2500);
    turnRight();
    forward();
    delay(500);
    stop();
    
    getWeightData();

    backward();
    delay(500);
    turnRight();
    forward();
    delay(2500);
    turnLeft();
    forward();
    delay(5000);
    turnRight();
    forward();
    delay(1000);
    stop();
    turnRight(180)
  }
  // 루트 2번
  else if (route == 1)
  {
    forward(0);
    delay(1000);
    turnLeft();
    forward();
    delay(5000);
    turnRight();
    forward();
    delay(1500);
    turnRight();
    forward();
    delay(500);
    stop();
    
    getWeightData();

    backward();
    delay(500);
    turnRight();
    forward();
    delay(1500);
    turnLeft();
    forward();
    delay(5000);
    turnRight();
    forward();
    delay(1000);
    stop();
    turnRight(180)
  }
  // 루트 3번
  else if (route == 1)
  {
    forward(0);
    delay(1000);
    turnRight();
    forward();
    delay(2000);
    turnLeft();
    forward();
    delay(2500);
    turnRight();
    forward();
    delay(500);
    stop();
    
    getWeightData();

    backward();
    delay(500);
    turnRight();
    forward();
    delay(2500);
    turnRight();
    forward();
    delay(2000);
    turnLeft();
    forward();
    delay(1000);
    stop();
    turnRight(180)
  }
  // 루트 4번
  else if (route == 1)
  {
    forward(0);
    delay(1000);
    turnRight();
    forward();
    delay(2000);
    turnLeft();
    forward();
    delay(1500);
    turnRight();
    forward();
    delay(500);
    stop();
    
    getWeightData();

    backward();
    delay(500);
    turnRight();
    forward();
    delay(1500);
    turnRight();
    forward();
    delay(2000);
    turnLeft();
    forward();
    delay(1000);
    stop();
    turnRight(180)
  }
}

// 셋업
void setup()
{
  // 시리얼 115200
  Serial.begin(115200);
  Serial.println("speed, enc_cnt_right, enc_cnt_left, right_pid, left_pid");

  // 외부 인터럽트 0, 1 설정
  // 라이징 엣지 마다 인터럽트 발생
  // 외부 인터럽트 서비스 루틴 실행 -> motorRightEncoderPulseRead/motorLeftEncoderPulseRead
  attachInterrupt(digitalPinToInterrupt(motorRightEnc), motorRightEncoderPulseRead, RISING);
  attachInterrupt(digitalPinToInterrupt(motorLeftEnc), motorLeftEncoderPulseRead, RISING);

  // 타이머 2 설정
  // 0.1초마다 타이머 인터럽트 발생
  // 타이머 인터럽트 서비스 루틴 실행 -> timerInterrupt
  MsTimer2::set(100, timerInterrupt);
  MsTimer2::start();

  // 핀모드 설정
  // R 모터 핀절성
  pinMode(motorRightPwm, OUTPUT);
  pinMode(motorRightEnable, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  pinMode(motorRightEnc, INPUT);
  // L 모터 핀절성
  pinMode(motorLeftPwm, OUTPUT);
  pinMode(motorLeftEnable, OUTPUT);
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorLeftEnc, INPUT);

  // 택트 버튼 설정
  pinMode(sw_up, INPUT_PULLUP);
  pinMode(sw_down, INPUT_PULLUP);

  // 조이스틱 설정
  pinMode(joy_stick_x, INPUT);
  pinMode(joy_stick_y, INPUT);

  // 초음파 센서 설정
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // MPU6050 설정
  initSensor();
  calibrateSensor();
  past = millis();

  // 무게센서 설정
  pinMode(weights, INPUT)
}

// 루프
void loop()
{
  sonarToRobot();
  getMpuData();
  getElapsedTime();
}