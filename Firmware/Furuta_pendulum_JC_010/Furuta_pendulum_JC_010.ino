//Furuta Pendulum Firmware 010 for Janus Controller 20.01

//SimpleFOC Version 2.0.1
#include <SimpleFOC.h>
#include <SPI.h>

//#######_USER VARIABLES_#######
byte pp = 7;                  //BLDC motor number of pole pairs
byte sourceVoltage = 12;      //Voltage of your power source [Volts]

//#######_SIMPLEFOC PARAMETERS_#######
float lpFilter = 0.00;
float voltageRamp = 25;       //Change in voltage allowed [Volts per sec]
float voltageLimit = 0.8;
float velocityLimit = 2000;   //Velocity limit [rpm]
 
//#######_DRV8305_########
//Datasheet: www.ti.com/lit/ds/symlink/drv8305.pdf
#define enGate 17       //Chip Enable
#define nFault 14       //Fault reading
#define cs 5            //DRV8305 Chip-select
bool faultTrig = false;

//######_MA730_######
//A and B encoder inputs
#define Int1 4             // interrupt 0
#define Int2 2             // interrupt 1

//######_TEMPERATURE SENSOR_######
#define vTemp 39
byte maxTemp = 80;      //Maximum temperature [°C]
float temp;

//#####_TIME MANAGEMENT_#####
float runTime, prevT = 0, timeDif, stateT, sampleTime;
int timeInterval = 1000, totalTempTime;
long swingDownTime = 0, swingUpTime = 0;

//#####_CONTROL VARIABLES_#####
float target_voltage;         // Voltage applied to the motor depending on the corresponding control algorithm
bool overTurn = false;        // if many turn have occured (encoder cable chocking with no slip ring)
float cosFourth = 0;          // Term used for the Energy Shaping Controller
int energy = 0;               // decided wether to use evergy shaping for swing-up or swing-down (currently swing-down is not used)
bool sSwitch = false;         // decide if stability has been achieved
float balanceAngle = 0.5;     // Threshold to switch between controllers

//####_GAINS OF THE CONTROLLERS_#####
// LQR
float k1 = 1.65;
float k2 = 0.14;
float k3 = 0.033;
float k4 = 0.015;
// P controller (return from overturning)
float k5 = 1;
// Energy Shaping
float k6 = 0.00365;

//####_SIMPLEFOC INSTANCES_####
BLDCMotor motor = BLDCMotor(pp);                        //BLDCMotor instance
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27);     //3PWM Driver instance

Encoder sensor = Encoder(Int1, Int2, 1024);       // Quadrature encoder instance for motor control
void doA(){sensor.handleA();}                     // Interrupt routine intialisation
void doB(){sensor.handleB();}                     // channel A and B callbacks

Encoder pend = Encoder(21, 22, 600);      // Quadrature encoder instance for the pendulum
void doA2(){pend.handleA();}              // Interrupt routine intialisation
void doB2(){pend.handleB();}              // channel A and B callbacks

Commander command = Commander(Serial);
void doM(char* cmd){ command.motor(&motor, cmd); }

LowPassFilter LPFpend{0.1};
LowPassFilter LPFmot{0.1};
LowPassFilter LPFswitch{0.02};
LowPassFilter LPFvoltage{0.02};

//-------------------------------------Set-up-------------------------------------------
void setup() {
  Serial.begin(115200);

  Serial.println("Init Pendulum.");
  _delay(1000);

   //Initialise magnetic sensor hardware
  pend.init();
  pend.enableInterrupts(doA2, doB2);

  _delay(500);

  Serial.println("Pendulum Sensor Ready.");

  //Pinmodes
  pinMode(nFault, INPUT);
  pinMode(enGate, OUTPUT);
  digitalWrite(enGate, LOW);

  //SPI start up
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);

  //Motor driver initialization
  _delay(250);
  Serial.println("DRV8305 INIT");
  drv_init();
  _delay(250);

  Serial.println("enGate Enabled");
  digitalWrite(enGate, HIGH);
  _delay(100);
  
  sensor.init();
  sensor.enableInterrupts(doA, doB); 
  motor.linkSensor(&sensor);
  
  // driver config, power supply voltage [V]
  driver.voltage_power_supply = sourceVoltage;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;

  // Sensor aligning voltage
  motor.voltage_sensor_align = voltageLimit/1.1;
  
  // velocity PI controller parameters, default K=0.5 Ti = 0.01

  motor.voltage_limit = voltageLimit;
  
  // maximal velocity of the poisition control, default 20
  motor.velocity_limit = velocityLimit;

  motor.useMonitoring(Serial);      // use monitoring functionality
  motor.init();                     // initialise motor
  motor.initFOC();                  // align sensor/ encoder and start FOC

  command.add('A', doM, "motor");
  Serial.println("BLDC Ready.");

  _delay(1000);
  Serial.println("Furuta Pendulum Ready");
  
}

//---------------------------------------Loop-------------------------------------------
void loop() {
  motor.loopFOC();
  serialEvent();
  command.run();
  
  //float e = pend.getAngle();
  //Serial.println(e,4);

  //Time managment
  runTime = micros();
  timeDif = runTime - prevT;
  prevT = runTime;
  stateT += timeDif;
  sampleTime += timeDif;

  // control loop each ~10ms (100 Hz)
  if (sampleTime >= 10000){
    sampleTime = 0;
    // calculate the pendulum angle and velocity, and motor angle and velocity with respective filtering
    float pendulum_angle = constrainAngle(pend.getAngle() + M_PI);
    float pendulum_velocity = LPFpend( pend.getVelocity() );
    float motor_velocity = LPFmot(-motor.shaftVelocity() );
    float motor_angle = sensor.getAngle();

    float pendulum_velocity_switch = LPFswitch( pend.getVelocity() );

    // if the encoder cable is NOT choked
    if (overTurn == false){
      // if angle small enough stabilize    
      if ( abs(pendulum_angle) < balanceAngle ) {
        //target_voltage = controllerLQR( pendulum_angle, pendulum_velocity, motor_velocity, motor_angle, k1, k2, k3, k4 );
        
        // Uses LQR with stronger gains for the transition between swing-up and up-right position
        if(sSwitch == false){
          //LQR controller with gains that saturate the controller to help with the transition from swing-up to up-right
          target_voltage = controllerLQR( pendulum_angle, pendulum_velocity_switch, motor_velocity, motor_angle, 4, 0.15, 0.033, 0 );
          // if pendulum is up-right after a defined time change to LQR with great balance gains
          if (abs(pendulum_angle) < 0.2) swingUpTime += timeDif;
          else swingUpTime = 0;
          if (swingUpTime >= 200) {
            sSwitch = true;
            swingUpTime = 0;
          }
        }
        else{
          // LQR controller for balance
          target_voltage = controllerLQR( pendulum_angle, pendulum_velocity, motor_velocity, motor_angle, k1, k2, k3, k4 );
          energy = 0;       // Change to 1 to do swing-down when the pendulum is disturbed hard enough
        }
      }
      else if (energy == 0){
        // do swing-up
        target_voltage = energyShaping( pendulum_angle, pendulum_velocity, k6 );
        sSwitch = false;
      }
      else if (energy == 1){
        // do swing-down
        target_voltage = -energyShaping( pendulum_angle, pendulum_velocity, 0.0045 );
        if (pendulum_velocity <= 0.25){
          swingDownTime += timeDif;
          // if the pendulum is down-right for a defined time change to swing-up
          if(swingDownTime >= 1000) energy = 0;
        }
        else{
          swingDownTime = 0;
        }
      }
      /*else {
        target_voltage = 0;
      }*/

      // Check if the motor is within a safe range for the encoder cable to not choke
      if (abs( motor_angle ) >= 23.562) {
        target_voltage = 0;
        overTurn = true;
      }
      
    }
    // if the motor position is too far from the start (and to avoid encoder cable choking)
    // use P controller to move the motor position back to the start
    else if (overTurn == true){
      target_voltage = k5 * motor_angle;
      // saturate the controller
      if ( abs(target_voltage) > voltageLimit*0.31 ) target_voltage = -_sign(target_voltage)*voltageLimit*0.31;
      if ( abs(motor_angle) < 0.15) {
        overTurn = false;
        energy = 0;
        swingDownTime = 0;
        sSwitch = false;
      }
    }

    //Serial.println(-target_voltage);
    //Serial.println(pendulum_angle);
    
    // set the target voltage to the motor
    motor.move(-target_voltage);

  }

  // 5 Hz function caller for Janus Controller security features. 
  if(stateT >= 200000){
    stateT = 0;
    tempStatus();         //Check Janus controller power-stage temperature
    faultStatus();        //Check power-stage driver for faults (Red LED will turn on if a fault is detected)
    //Serial.println(temp);
  }
  
}


//----------------------------------Control Functions-----------------------------------

// Function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR stabilization controller functions
// calculates the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel, float m_pos, float c1, float c2, float c3, float c4){
  // calculate the control law 
  // LQR controller u = k*x
  //  - k = [c1, c2, c3, c4]
  //  - x = [pendulum angle, pendulum velocity, motor velocity, motor position]' 
  float u =  c1*p_angle + c2*p_vel + c3*m_vel + c4*m_pos;
  
  // limit the voltage set to the motor (saturation)
  if(abs(u) > voltageLimit) u = _sign(u)*voltageLimit;
  
  return u;
}

// Energy shaping controller
// calculates the voltage that needs to be set to the motor in order to add/ remove energy to the pendulum
float energyShaping(float p_angle, float p_vel, float c6){
  // Ben Katz' energy shaping controller for swing-up and swing-down
  // https://build-its-inprogress.blogspot.com/2016/08/desktop-inverted-pendulum-part-2-control.html
  
  float cosFourth = cos( p_angle )*cos( p_angle )*cos( p_angle )*cos( p_angle );
  float u = c6*cosFourth*(p_vel)*(9.81*(1-cos(p_angle)-(0.0075*(p_vel)*(p_vel))));

  // limit the voltage set to the motor (saturation)
  if ( abs(target_voltage) > voltageLimit*1 ) target_voltage = _sign(target_voltage)*voltageLimit*1;

  return u;
}

// Tune the LQR controller, the threshold transition angle and energy shaping controller from the Serial monitor.
void serialEvent() {
  // a string to hold incoming data
  static String inputString;
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      if(inputString.charAt(0) == 'X'){
        k1 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == 'Y'){
        k2 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == 'Z'){
        k3 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == 'V'){
        voltageLimit = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == 'W'){
        k4 = inputString.substring(1).toFloat();
        spaceGains();
      }else if(inputString.charAt(0) == 'S'){
        k6 = inputString.substring(1).toFloat();
        Serial.print("Swing up gain: ");
        Serial.println(k6,4);
      }else if(inputString.charAt(0) == 'H'){
        balanceAngle = inputString.substring(1).toFloat();
        Serial.print("Balance angle: ");
        Serial.println(balanceAngle,4);
      }
      inputString = "";
    }
  }
}

// Print the LQR controller gains
void spaceGains(){
  Serial.print("K1 pAngle: ");
  Serial.print(k1);
  Serial.print(",\t K2 pVel: ");
  Serial.print(k2,4);
  Serial.print(",\t K3 mVel: ");
  Serial.print(k3,4);
  Serial.print(",\t K4 mPos: ");
  Serial.print(k4,4);
  Serial.print(",\t Voltage Limit: ");
  Serial.println(voltageLimit,4);
}



//-------------------------------------Janus Controller Functions--------------------------------------

//Temperature status and manager
void tempStatus(){
  static int tFlag;

  //Read voltage from temperature sensor and transform it to °C
  float vOut = analogRead(vTemp);
  temp = (((vOut*3.3)/4095)-0.4)/0.0195;
  //Serial.println(temp,2);
  
  if (temp >= 80 && tFlag == false){
    int tempTime = micros();
    totalTempTime += tempTime;

    //If temperature is high for 3 seconds disable DRV
    if(totalTempTime >= 3000000){
      tFlag = true;
      digitalWrite(enGate, LOW);
      Serial.print("enGate Disabled - Temperature protection: ");
      Serial.println(temp);
    }
    
  }
  else if (temp <= 80 && tFlag == false){
    totalTempTime = 0;
  }
  
}

//Configure DRV8305 to desired operation mode
void drv_init(){

  //Set to three PWM inputs mode
  digitalWrite(cs, LOW);
  byte resp1 = SPI.transfer(B00111010);
  byte resp2 = SPI.transfer(B10000110);
  digitalWrite(cs, HIGH);

  Serial.println(resp1, BIN);
  Serial.println(resp2, BIN);
  
}

//Fault status and manager for the DRV8305
//Datahseet pages 37 and 38
void faultStatus(){
  //Read nFault pin from DRV8305 - LOW == error / HIGH == normal operation
  int fault = digitalRead(nFault);
  
  if(fault == LOW && faultTrig == false){
    Serial.println("Fault detected");
    faultTrig = true;
    //Check warning and watchdog reset (Address = 0x1)
    digitalWrite(cs, LOW);
    byte ft1 = SPI.transfer(B10001000);
    byte ft2 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x1");
    Serial.println(ft1,BIN);
    Serial.println(ft2,BIN);

    //Check OV/VDS Faults (Address = 0x2)
    digitalWrite(cs, LOW);
    byte ft3 = SPI.transfer(B10010000);
    byte ft4 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x2");
    Serial.println(ft3,BIN);
    Serial.println(ft4,BIN);

    //Check IC Faults (Address = 0x3)
    digitalWrite(cs, LOW);
    byte ft5 = SPI.transfer(B10011000);
    byte ft6 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x3");
    Serial.println(ft5,BIN);
    Serial.println(ft6,BIN);

    //Check VGS Faults (Address = 0x4)
    digitalWrite(cs, LOW);
    byte ft7 = SPI.transfer(B10100000);
    byte ft8 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x4");
    Serial.println(ft7,BIN);
    Serial.println(ft8,BIN);
  }
}
