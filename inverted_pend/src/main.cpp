#include<Arduino.h>
#include<ESP32Encoder.h>

#define PWM_PIN 13
#define DIR_PIN 12
#define ENC1_A 14
#define ENC1_B 27
#define ENC2_A 26
#define ENC2_B 25

double K[4] = {-38.7298, -22.0071, -63.2895, -13.3513};
const double pos_const = 2 * 3.1415 * 0.0065 / (600 * 4);
const double angle_const = 2 * 3.1415 / (2000 * 4);

double x = 0, theta = 0;
double x_dot = 0, theta_dot = 0;

double last_x = 0, last_theta = 0;
unsigned long current_time = 0, last_time = 0;

double f = 0, voltage = 0, last_voltage = 0;

ESP32Encoder enc1, enc2;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  digitalWrite(DIR_PIN, LOW);
  
  enc1.attachFullQuad(ENC1_A, ENC1_B);
  enc2.attachFullQuad(ENC2_A, ENC2_B);
  last_time = millis();
}

void loop() 
{
  x = -enc1.getCount() * pos_const;
  theta = -enc2.getCount() * angle_const;
  current_time = millis();
  
  if (current_time != last_time){
    x_dot = (x - last_x) / (0.001 * (current_time - last_time));
    theta_dot = (theta - last_theta) / (0.001 * (current_time - last_time));
  }
  
  last_x = x;
  last_theta = theta;
  last_time = current_time;

  f = (-K[0] * x) + (-K[1] * x_dot) + (-K[2] * theta) + (-K[3] * theta_dot);
  
  voltage = 3 * f; //+ (0.1 * f / fabs(f));
  
  //anti jittering
  // if (fabs(voltage - last_voltage) > 1.5){
  //   voltage = last_voltage + 1.5 * fabs(voltage - last_voltage) / (voltage - last_voltage);
  // }
  
  //voltage limiting
  if (fabs(voltage) > 12){
    voltage = 12.00 * voltage / fabs(voltage);
  }

  last_voltage = voltage;

  analogWrite(PWM_PIN, (int)((fabs(voltage) / 12.00) * 255.00));
  (voltage < 0) ? digitalWrite(DIR_PIN, LOW) : digitalWrite(DIR_PIN, HIGH);
  
  delay(10);
}