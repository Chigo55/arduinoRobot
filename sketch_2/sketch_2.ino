#include <MsTimer2.h>

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
double right_proportion = 2;
double right_integral = 1;
double right_derivative = 0;
// 오른쪽용 PID제어기 제어 이득
double left_proportion = 2;
double left_integral = 1;
double left_derivative = 0;

// PID 오차값 전역변수로 선언
// 오른쪽 모터용
float err_right_P = 0;
float err_right_I = 0;
float err_right_D = 0;
float err_right_B = 0 ;
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

// 기타 변수
int speed = 0;
int target_right = 0;
int target_left = 0;
int right_pid_val = 0;
int left_pid_val = 0;


// 외부 인터럽스 서비스 루틴
void motorRightEncoderPulseRead()
{
    enc_pulse_right++;
}
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

// 셋업
void setup()
{
    // 시리얼 9600
    // 시리얼 9600
    Serial.begin(9600);
    Serial.println("speed, enc_cnt_right, enc_cnt_left, right_pid, left_pid");

    // 외부 인터럽트 0, 1 설정
    // 라이징 엣지 마다 인터럽트 발생
    // 외부 인터럽트 서비스 루틴 실행 -> motorRightEncoderPulseRead/motorLeftEncoderPulseRead
    attachInterrupt(digitalPinToInterrupt(motorRightEnc), motorRightEncoderPulseRead, RISING);
    attachInterrupt(digitalPinToInterrupt(motorLeftEnc), motorLeftEncoderPulseRead, RISING);

    // 타이머 2 설정
    // 0.5초마다 타이머 인터럽트 발생
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
}

// 루프
void loop()
{
    joystickToSpeed();

    speedToTargetR(speed);
    speedToTargetL(speed);

    Serial.print(speed);
    Serial.print(", ");
    Serial.print(enc_cnt_right);
    Serial.print(", ");
    Serial.print(enc_cnt_left);
    Serial.print(", ");
    Serial.print(right_pid_val);
    Serial.print(", ");
    Serial.println(left_pid_val);
    delay(100);
}
