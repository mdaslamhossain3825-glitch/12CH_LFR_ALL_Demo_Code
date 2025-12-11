#include <EEPROM.h>
//motorpins
#define right_motor_forward A2  //IN1(L298) || TB6612 -- AIN2
#define right_motor_backward 4  //IN2(L298) || TB6612 -- AIN1
#define left_motor_forward 5    //IN3(L298)  || TB6612 --- BIN1
#define left_motor_backward 6   //IN4(L298)  || TB6612 --- BIN2
//speed control pins (PWM)
#define right_motor_speed 3  //EnA (L298) //TB6612 --- PWMA
#define left_motor_speed 9   //EnB (L298) //TB6612 --- PWMB

//MUX PIN
#define S0 11
#define S1 10
#define S2 8
#define S3 7
#define SIG_PIN A7

//Sensor Variables
#define sensorNumber 12
int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];
int bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };
int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
int theshold = 500;
int sumOnSensor;
int sensorWight;
int bitSensor;
int Max_ADC[sensorNumber];
int Min_ADC[sensorNumber];
int Reference_ADC[sensorNumber];
//PID Variables
float line_position;
float error;
float center_position = 45;
float derivative, previous_error;
int base_speed = 150;
int kp = 8;
int kd = 100;
//inverse parameter
bool inverseON = 0;  //0 = normal line, 1 = inverse line
//button
#define button1 A0
#define button2 A5
#define button3 12
//led
#define led 13

//************************************SETUP***************************************
void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);

  pinMode(left_motor_speed, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);

  LoadCalibration();  //load calibration value from eeprom whenever the arduino is start or reset
}


//************************************LOOP***************************************
void loop() {
  if (digitalRead(button1) == 0) {  //when button 1 is pressed
    calibrateSensor();
  }
  if (digitalRead(button2) == 0) {  //when button 2 is pressed
    PID_Controller(base_speed, kp, kd);
  }
}
