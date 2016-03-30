#include "RealMouse.h"

RealMouse *RealMouse::instance = nullptr;

RealMouse *RealMouse::inst() {
  if (instance == NULL) {
    instance = new RealMouse();
  }

  return instance;
}

void RealMouse::setSpeed(int forwardVelocity, float ccwVelocity) {
  kc.setVelocity(forwardVelocity, ccwVelocity);
}

Pose RealMouse::getPose(){
  return kc.getPose();
}

float *RealMouse::getRawDistances() {
  rawDistances[0] = right_rangefinder.readRangeContinuous();
  rawDistances[1] = middle_rangefinder.readRangeContinuous();
  rawDistances[2] = left_rangefinder.readRangeContinuous();
  return rawDistances;
}

RealMouse::RealMouse() :
  display(OLED_RESET),
  middle_rangefinder(VL6180EN1,0x41),
  left_rangefinder(VL6180EN2,0x42),
  right_rangefinder(VL6180EN3,0x43),
  motL(ENCODER1A, ENCODER1B, MOTOR1B, MOTOR1A),
  motR(ENCODER2A, ENCODER2B, MOTOR2B, MOTOR2A),
  kc(&motL, &motR, 1, -1, 78.3f, 31.71f, 12 * (1537480.0f/20280)) { }

  SensorReading RealMouse::checkWalls() {
    bool walls[4] = {false, false, false, false};
    SensorReading sr(row, col, walls);
    return sr;
  }

void RealMouse::run(){
  kc.run();
}

void RealMouse::updateGlobalYaw(){
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
  imu.getCalibration(&system, &gyro, &accel, &mag);

  if (system != 0 ){
    imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);

    kc.updateGlobalYaw(euler.x());
  }
}

void RealMouse::suggestWalls(bool *walls) {
  hasSuggestion = true;
  for (int i=0;i<4;i++){
    suggestedWalls[i] = walls[i];
  }
}

void RealMouse::setup(){
  analogReadResolution(12);

  digitalWrite(LEDR,LOW);
  digitalWrite(LEDG,LOW);
  digitalWrite(LEDB,LOW);
  digitalWrite(LEDGO,LOW);
  digitalWrite(SDCS,LOW);
  digitalWrite(BUZZER,LOW);

  digitalWrite(IREMITTER1,LOW);
  digitalWrite(IREMITTER2,LOW);
  digitalWrite(IREMITTER3,LOW);
  digitalWrite(IREMITTER4,LOW);
  digitalWrite(IREMITTER5,LOW);
  digitalWrite(IREMITTER6,LOW);

  digitalWrite(VL6180EN1,LOW);
  digitalWrite(VL6180EN2,LOW);
  digitalWrite(VL6180EN3,LOW);
  digitalWrite(VL6180EN4,LOW);
  digitalWrite(VL6180EN5,LOW);
  digitalWrite(VL6180EN6,LOW);

  digitalWrite(MOTOR1A,LOW);
  digitalWrite(MOTOR1B,LOW);
  digitalWrite(MOTOR2A,LOW);
  digitalWrite(MOTOR2B,LOW);
  digitalWrite(MOTORDIR1,LOW);
  digitalWrite(MOTORDIR2,LOW);

  pinMode(IREMITTER1,OUTPUT);
  pinMode(IREMITTER2,OUTPUT);
  pinMode(IREMITTER3,OUTPUT);
  pinMode(IREMITTER4,OUTPUT);
  pinMode(IREMITTER5,OUTPUT);
  pinMode(IREMITTER6,OUTPUT);
  pinMode(IRRECEIVER1,INPUT);
  pinMode(IRRECEIVER2,INPUT);
  pinMode(IRRECEIVER3,INPUT);
  pinMode(IRRECEIVER4,INPUT);
  pinMode(IRRECEIVER5,INPUT);
  pinMode(IRRECEIVER6,INPUT);

  pinMode(BATTERYSENSE,INPUT);

  pinMode(VL6180EN1,OUTPUT);
  pinMode(VL6180EN2,OUTPUT);
  pinMode(VL6180EN3,OUTPUT);
  pinMode(VL6180EN4,OUTPUT);
  pinMode(VL6180EN5,OUTPUT);
  pinMode(VL6180EN6,OUTPUT);

  pinMode(ENCODER1A,INPUT);
  pinMode(ENCODER1B,INPUT);
  pinMode(ENCODER2A,INPUT);
  pinMode(ENCODER2B,INPUT);

  pinMode(MOTOR1A,OUTPUT);
  pinMode(MOTOR1B,OUTPUT);
  pinMode(MOTOR2A,OUTPUT);
  pinMode(MOTOR2B,OUTPUT);

  pinMode(MOTORDIR1,OUTPUT);
  pinMode(MOTORDIR2,OUTPUT);

  pinMode(LEDR,OUTPUT);
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);
  pinMode(LEDGO,OUTPUT);

  pinMode(BUTTONGO,INPUT_PULLUP);
  pinMode(BUTTON1,INPUT_PULLUP);
  pinMode(BUTTON2,INPUT_PULLUP);
  pinMode(SDCARDDETECT,INPUT_PULLUP);

  pinMode(SDCS,OUTPUT);
  pinMode(BUZZER,OUTPUT);


  digitalWrite(LEDR,LOW);
  digitalWrite(LEDG,LOW);
  digitalWrite(LEDB,LOW);
  digitalWrite(LEDGO,LOW);
  digitalWrite(SDCS,LOW);
  digitalWrite(BUZZER,LOW);

  digitalWrite(IREMITTER1,LOW);
  digitalWrite(IREMITTER2,LOW);
  digitalWrite(IREMITTER3,LOW);
  digitalWrite(IREMITTER4,LOW);
  digitalWrite(IREMITTER5,LOW);
  digitalWrite(IREMITTER6,LOW);

  digitalWrite(VL6180EN1,LOW);
  digitalWrite(VL6180EN2,LOW);
  digitalWrite(VL6180EN3,LOW);
  digitalWrite(VL6180EN4,LOW);
  digitalWrite(VL6180EN5,LOW);
  digitalWrite(VL6180EN6,LOW);

  digitalWrite(MOTOR1A,LOW);
  digitalWrite(MOTOR1B,LOW);
  digitalWrite(MOTOR2A,LOW);
  digitalWrite(MOTOR2B,LOW);
  digitalWrite(MOTORDIR1,LOW);
  digitalWrite(MOTORDIR2,LOW);

  //Start I2C library
  Wire.begin();
  Wire1.begin();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  // initialize with the I2C addr 0x3D (for the 128x64)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(200);

  /* Initialise the sensor */
  if(!imu.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    display.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    display.display();
    exit(0);
  }

  int left_rangefinder_init_status = left_rangefinder.initMouse();
  int middle_rangefinder_init_status = middle_rangefinder.initMouse();
  int right_rangefinder_init_status = right_rangefinder.initMouse();
  if(left_rangefinder_init_status != 0 || middle_rangefinder_init_status != 0 || right_rangefinder_init_status != 0 ){
    display.println("Ooops, rangefinder broken");
    display.print(left_rangefinder_init_status);
    display.print(middle_rangefinder_init_status);
    display.print(right_rangefinder_init_status);
    display.display();
    exit(0);
  }

  int range_interval = 50;

  left_rangefinder.startRangeContinuous(range_interval);
  middle_rangefinder.startRangeContinuous(range_interval);
  right_rangefinder.startRangeContinuous(range_interval);

  float voltage = analogRead(BATTERYSENSE) * (3.3 / 4095) * (49 / 10.0) / 3.0;

  if (voltage < 3.6){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Voltage is too low!");
    display.println(voltage);
    display.display();
    exit(0);
  }

  Serial.print("voltage :");
  Serial.println(voltage);

  imu.setExtCrystalUse(true);

  motL.setPID(0.09,0.01,0.01,0);
  motR.setPID(0.09,0.01,0.01,0);
  kc.setSampleTime(20);
  kc.setAcceleration(4000,M_PI,4000,M_PI);

  kc.setup();
}
