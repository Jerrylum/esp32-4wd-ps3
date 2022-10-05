// https://github.com/jvpernis/esp32-ps3
#include <Ps3Controller.h>
const int DATA_PIN = 12;  /* DS UNO-D4 esP32-IO12*/
const int CLOCK_PIN = 17; /* SHCP UNO-D8 esP32-IO17*/
const int latchPin = 19;  /* STCP UNO-D12 esP32-IO19 */

#define PWMA 23  // A電機轉向
#define PWMB 25  // B電機轉向
#define PWMC 16  // C電機轉向
#define PWMD 27  // D電機轉向

const int PWMFreq = 30000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel1 = 1;
const int PWMSpeedChannel2 = 2;
const int PWMSpeedChannel3 = 3;
const int PWMSpeedChannel4 = 4;
int dutyCycle = 200;

//遙桿位置參數 X由左-128至右127, Y由上-128至下127
int rX;
int rY;
int lX;
int lY;

int abs127(int t) {
  return abs(t) > 127 ? 127 : abs(t);
}

void motor(int leftFront, int leftRear, int rightFront, int rightRear) {
  int data = 0b00000000;

  data |= rightFront >= 0 ? 0b00000001 : 0;
  data |= rightFront <= 0 ? 0b00000010 : 0;

  data |= rightRear >= 0 ? 0b00000100 : 0;
  data |= rightRear <= 0 ? 0b10000000 : 0;

  data |= leftFront >= 0 ? 0b00100000 : 0;
  data |= leftFront <= 0 ? 0b00010000 : 0;

  data |= leftRear >= 0 ? 0b01000000 : 0;
  data |= leftRear <= 0 ? 0b00001000 : 0;

  digitalWrite(latchPin, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, data);
  digitalWrite(latchPin, HIGH);
  ledcWrite(PWMSpeedChannel1, 128 + abs127(leftFront));
  ledcWrite(PWMSpeedChannel2, 128 + abs127(leftRear));
  ledcWrite(PWMSpeedChannel3, 128 + abs127(rightFront));
  ledcWrite(PWMSpeedChannel4, 128 + abs127(rightRear));
}

void moveCar() {
  int left = -lY + lX;
  int right = -lY - lX;
  int x2 = rX;

  int leftFront = left + x2;
  int rightFront = right - x2;
  int leftRear = left - x2;
  int rightRear = right + x2;

  motor(leftFront, leftRear, rightFront, rightRear);
}

void setUpPinModes() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(PWMD, OUTPUT);
  ledcSetup(PWMSpeedChannel1, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel2, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel3, PWMFreq, PWMResolution);
  ledcSetup(PWMSpeedChannel4, PWMFreq, PWMResolution);
  ledcAttachPin(PWMA, PWMSpeedChannel1);
  ledcAttachPin(PWMB, PWMSpeedChannel2);
  ledcAttachPin(PWMC, PWMSpeedChannel3);
  ledcAttachPin(PWMD, PWMSpeedChannel4);
  ledcWrite(PWMSpeedChannel1, 0);
  ledcWrite(PWMSpeedChannel2, 0);
  ledcWrite(PWMSpeedChannel3, 0);
  ledcWrite(PWMSpeedChannel4, 0);
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();
  Ps3.begin("e4:5f:01:08:d4:54");  // PS3手制MAC地址
  Serial.println("Ready.");
}

void loop() {
  if (Ps3.isConnected()) {
    lX = (Ps3.data.analog.stick.lx);
    lY = (Ps3.data.analog.stick.ly);
    rX = (Ps3.data.analog.stick.rx);
    rY = (Ps3.data.analog.stick.ry);
    // Serial.print(lX);
    // Serial.print(" ");
    // Serial.print(lY);
    // Serial.print(" ");
    // Serial.print(rX);
    // Serial.print(" ");
    // Serial.println(rY);
    moveCar();
  }
}