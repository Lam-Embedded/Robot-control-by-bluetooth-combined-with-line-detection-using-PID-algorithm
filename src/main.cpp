#include <Arduino.h>
#include "SoftwareSerial.h"
#include <PID_v1.h>

SoftwareSerial HC06(3, 2); //TX, RX

#define VCC 5
#define GND 4
int ENA = 6;
int IN1 = 7;
int IN2 = 8;
int IN3 = 9;
int IN4 = 10;
int ENB = 11;
/// define sensor pinout
#define line_1      A0 // trái (hoặc ngược lại)
#define line_2      A1
#define line_3      A2 // giữa
#define line_4      A3 
#define line_5      A4 // phải

// define ultrasonic senso
const int trig = 12;
const int echo = 13;

char dieu_khien;
uint8_t speed_robot = 150;
int8_t check_out = 0;
// Thong so cho doline PID
double Setpoint = 0, Input, Output;
uint8_t flag_zero = 0;
double Kp = 16.5, Ki = 0.0291, Kd = 3.499999; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int8_t sensor;

void tien();
void lui();
void trai();
void phai();
void tien_trai();
void tien_phai();
void lui_trai();
void lui_phai();
void Stop();
void motorControl(int16_t duty_value);
void scan_sensor();
void follow_line();
void forward(); 
int distance();

void setup() {
    HC06.begin(9600);
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    // Cap nguon cho bluetooth
    pinMode(VCC, OUTPUT);
    pinMode(GND, OUTPUT);
    // Khai vao cam bien do line
    pinMode(line_1, INPUT);
    pinMode(line_2, INPUT);
    pinMode(line_3, INPUT);
    pinMode(line_4, INPUT);
    pinMode(line_5, INPUT);
    // Thiet lap PID
    Input = 0; 
    myPID.SetSampleTime(1); // thời gian lấy mẫu phụ thuộc tốc độ xe, lấy mẫu càng nhanh càng tốt
    myPID.SetMode(AUTOMATIC); 
    myPID.SetOutputLimits(-speed_robot, speed_robot);
    // setup ultrasonic
    pinMode(trig,OUTPUT);
    pinMode(echo,INPUT);
    // analogWrite(ENA, speed_robot);
    // analogWrite(ENB, speed_robot);
    digitalWrite(VCC, HIGH);
    digitalWrite(GND, LOW);
}

void loop()  {
    if (HC06.available() > 0) {
        dieu_khien = HC06.read();
        Serial.println(dieu_khien);
        switch (dieu_khien) {
            case 'U':
                while (HC06.read() != 'u') {
                    follow_line();
                }
                break;
            case 'F':
                tien();
                break;
            case 'B':
                lui();
                break;
            case 'L':
                trai();
                break;
            case 'R':
                phai();
                break;
            case 'I':
                tien_phai();
                break;
            case 'G':
                tien_trai();
                break;
            case 'J':
                lui_phai();
                break;
            case 'H':
                lui_trai();
                break;
            case 'S':
                Stop();
                break;
        }
    }
}   

void scan_sensor() {
    if(digitalRead(line_5) == 0) {
      sensor = 4;
    }
    else if((digitalRead(line_4) == 0)&&(digitalRead(line_5) == 0)) {
      sensor = 3;
    }
    else if(digitalRead(line_4) == 0) {
      sensor = 2;
    }
    else if((digitalRead(line_3) == 0)&&(digitalRead(line_4) == 0)) {
      sensor = 1;
    }
    else if(digitalRead(line_3) == 0) {
      sensor = 0;
    }
    else if((digitalRead(line_2) == 0)&&(digitalRead(line_3) == 0)) {
      sensor = -1;
    }
    else if(digitalRead(line_2) == 0) {
      sensor = -2;
    }
    else if((digitalRead(line_1) == 0)&&(digitalRead(line_2) == 0)) {
      sensor = -3;
    }
    else if(digitalRead(line_1) == 0) {
      sensor = -4;
    }
}

void motorControl(int16_t duty_value) {
    int16_t speed_left, speed_right;
    int speed_base = speed_robot / 2;

    // Nếu line lệch phải → cần rẽ trái → bánh trái nhanh, bánh phải chậm/dừng
    if (duty_value > 1) {
        speed_left = duty_value;
        speed_right = -speed_base;
    }
    // Nếu line lệch trái → cần rẽ phải → bánh phải nhanh, bánh trái chậm/dừng
    else if (duty_value < -1) {
        speed_right = -duty_value;
        speed_left = -speed_base;
    }
    else {
        speed_left = speed_right = 0;
    }

    analogWrite(ENA, speed_left + speed_base);
    analogWrite(ENB, speed_right + speed_base);
    forward();
}

void forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void lui() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void tien() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void phai() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void trai() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void tien_trai() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

}
void tien_phai() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

}
void lui_phai() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

}
void lui_trai() {
    analogWrite(ENA, speed_robot + 105);
    analogWrite(ENB, speed_robot + 105);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}   
void Stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void follow_line() {
    Setpoint = 0;
    scan_sensor();
    Input = -sensor;
    myPID.Compute();
    motorControl(Output);
}

int distance() {
    unsigned long duration; 
    int distance;        
    
    digitalWrite(trig,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trig,1);   // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(trig,0);   // tắt chân trig
     
    duration = pulseIn(echo,HIGH);   
    distance = int(duration/2/29.412);
    delay(200);
    return distance;
}
