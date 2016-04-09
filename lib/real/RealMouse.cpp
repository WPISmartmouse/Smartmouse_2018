#include "RealMouse.h"
#include "KinematicController.h"

RealMouse *RealMouse::instance = nullptr;

RealMouse *RealMouse::inst() {
  if (instance == NULL) {
    instance = new RealMouse();
  }

  return instance;
}

RealMouse::RealMouse() :
  display(OLED_RESET),
  middle_rangefinder(VL6180EN2,0x41),
  left_rangefinder(VL6180EN1,0x42),
  right_rangefinder(VL6180EN3,0x43),
  motL(ENCODER1A, ENCODER1B, MOTOR1B, MOTOR1A),
  motR(ENCODER2A, ENCODER2B, MOTOR2B, MOTOR2A),
  hasSuggestion(false),
  kc(&motL, &motR, 1, -1, 78.3f, 31.71f, 12 * (1537480.0f/20280)),
  eastYaw(0),
  lastDisplayUpdate(0) { }

void RealMouse::setSpeed(int forwardVelocity, float ccwVelocity) {
  if (ccwVelocity > RealMouse::MAX_ROT_SPEED) ccwVelocity = RealMouse::MAX_ROT_SPEED;
  if (ccwVelocity < -RealMouse::MAX_ROT_SPEED) ccwVelocity = -RealMouse::MAX_ROT_SPEED;
  if (forwardVelocity > MAX_SPEED) forwardVelocity = MAX_SPEED;
  if (forwardVelocity < -MAX_SPEED) forwardVelocity = -MAX_SPEED;

  kc.setVelocity(forwardVelocity, ccwVelocity);
}

Pose RealMouse::getPose(){
  return kc.getPose();
}

float *RealMouse::getRawDistances() {
  rawDistances[0] = right_rangefinder.readRangeContinuous()/1000.0;
  rawDistances[1] = middle_rangefinder.readRangeContinuous()/1000.0;
  rawDistances[2] = left_rangefinder.readRangeContinuous()/1000.0;
  return rawDistances;
}

SensorReading RealMouse::checkWalls() {
  SensorReading sr(row, col);
  std::array<bool, 4> *walls = &sr.walls;

  //if nothing was suggested yet, then do a raw sensor read
  if (hasSuggestion){
    (*walls)[0] = suggestedWalls[0];
    (*walls)[1] = suggestedWalls[1];
    (*walls)[2] = suggestedWalls[2];
    (*walls)[3] = suggestedWalls[3];

  }
  else {
    (*walls)[0] = true;
    (*walls)[1] = false;
    (*walls)[2] = true;
    (*walls)[3] = true;
  }

  return sr;
}

void RealMouse::run(){

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastRunTime;

  if (deltaTime >= sampleTime){
    kc.updateGlobalYaw(getIMUYaw());
    kc.runNow(deltaTime, currentTime);
    lastRunTime = currentTime;
  }

  goButton.update();
  aButton.update();
  bButton.update();
}

void RealMouse::brake(){
  kc.brake();
}

void RealMouse::updateGlobalYaw(){
  kc.updateGlobalYaw(getIMUYaw());
}

float RealMouse::getVoltage(){
  return analogRead(BATTERYSENSE) * (3.3 / 4095) * (49 / 10.0) / 3.0;
}

uint8_t RealMouse::getIMUCalibration(){
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
  imu.getCalibration(&system, &gyro, &accel, &mag);
  return system;
}

float RealMouse::getRawIMUYaw(){
  imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  float degreeYaw = euler.x();
  float radYaw = - (degreeYaw * M_PI / 180.0);
  return KinematicController::constrainAngle(radYaw);
}

float RealMouse::getIMUYaw(){
  return KinematicController::constrainAngle(getRawIMUYaw() - eastYaw);
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

  goButton.attach(BUTTONGO);
  goButton.interval(10);

  aButton.attach(BUTTON1);
  aButton.interval(200);

  bButton.attach(BUTTON2);
  bButton.interval(200);

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
  kc.setAcceleration(6000,2*M_PI,6000,10*M_PI);

  kc.setup();
}

void RealMouse::setEastYaw(float yaw) {
  eastYaw = yaw;
}

void RealMouse::clearDisplay() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
}

void RealMouse::updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayUpdate > displayUpdateInterval){
    lastDisplayUpdate = now;
    display.display();
  }
}
