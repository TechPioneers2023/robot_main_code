#include <MPU6050.h>

#include <Wire.h>
#include<Servo.h>

//servo mappings
#define SERVO1 11                
#define SERVO2 2                  
#define SERVO3 4                  
#define SERVO4 3

//motor mappings
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 12
#define IN4 13  

//driver motor movements
#define MOTOR_GO_FORWARD {digitalWrite(IN1, LOW);digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);}
#define MOTOR_GO_BACKWARD {digitalWrite(IN1, HIGH);digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);}
#define MOTOR_GO_LEFT {digitalWrite(IN1, LOW);digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);}
#define MOTOR_GO_RIGHT {digitalWrite(IN1, HIGH);digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);}
#define MOTOR_STOP {digitalWrite(IN1, LOW);digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);}

//servo time-based control
#define SERVO_INTERVAL_SLOW 30
#define SERVO_INTERVAL_MEDIUM 15
#define SERVO_INTERVAL_FAST 10
#define SERVO_INTERVAL_ULTRA_FAST 0.01

//LED
#define LED A0

//servo motors
Servo servo1; //base of arm
Servo servo2;
Servo servo3;
Servo servo4; //head of arm

//servo angles
int set_servo1_pos = 0;
int set_servo2_pos = 0;
int set_servo3_pos = 0;
int set_servo4_pos = 0;

int curr_servo1_pos = 90;
int curr_servo2_pos = 90;
int curr_servo3_pos = 90;
int curr_servo4_pos = 92;

int servo1_time = 0;
int servo2_time = 0;
int servo3_time = 0;
int servo4_time = 0;
int combined_servo_time = 0;

//driver motor speed
int left_motor_speed = 255;
int right_motor_speed = 255;

//Head Light Status, default 1 (ON)
bool led = 1;

//serial commands
char command = '\n';
String  cmd;

//gyroscope
//MPU6050 gyroscope;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;
//bool gyroscope_connected = false;

void PrintDigits(int digits)
{
  if (digits >= 0 && digits <= 9)
  {
    Serial.print(digits);
    Serial.print(" ");
    Serial.print(" ");
    Serial.print(" ");
  }
    else if (digits <= -1 && digits >= -9 || digits >= 10 && digits <= 99)  // 2 Digits
  {
    Serial.print(digits);
    Serial.print(" ");
    Serial.print(" ");
  } else if (digits <= -10 && digits >= -99 || digits >= 100) // 3 Digits
  {
    Serial.print(digits);
    Serial.print(" ");
  } else if (digits <= -100)  // 4 Digits
  {
    Serial.print(digits);
  }
}

//servo1 rotation functions
void rotate_base(int set_servo1_pos){
  PrintDigits(curr_servo1_pos);
  Serial.print("|");
  bool angle_set = false;
  servo1_time = millis();
  while(!angle_set){
    if(millis()-servo1_time > SERVO_INTERVAL_MEDIUM){
       servo1_time += SERVO_INTERVAL_MEDIUM;
      if(set_servo1_pos > curr_servo1_pos){
          curr_servo1_pos++;
          servo1.write(curr_servo1_pos);
      } else if(set_servo1_pos < curr_servo1_pos){
          curr_servo1_pos--;
          servo1.write(curr_servo1_pos);
          Serial.print("=");
      }
      else {
        angle_set = true;
        Serial.print("|");
      }
    }
  }
}

void base_bend_90_front(){
  rotate_base(0);
}

void base_straighten(){
  rotate_base(90);
}

void base_bend_90_back(){
  rotate_base(180);
}

//servo2 rotation functions
void rotate_joint(int set_servo2_pos){
  PrintDigits(curr_servo2_pos);
  Serial.print("|");
  bool angle_set = false;
  servo2_time = millis();
  while(!angle_set){
    if(millis()-servo2_time > SERVO_INTERVAL_MEDIUM){
      servo2_time += SERVO_INTERVAL_MEDIUM;
      if(set_servo2_pos > curr_servo2_pos){
          curr_servo2_pos++;
          servo2.write(curr_servo2_pos);
      } else if(set_servo2_pos < curr_servo2_pos){
          curr_servo2_pos--;
          servo2.write(curr_servo2_pos);
          Serial.print("=");
      }
      else {
        angle_set = true;
        Serial.print("|");
      }
    }
  }
}

void collapase_arm(){
  rotate_joint(0);
}

void bend_arm_90_degrees(){
  rotate_joint(90);
}

void straighten_arm(){
  rotate_joint(180);
}

//servo3 rotation functions
void rotate_claw(int set_servo3_pos){
  PrintDigits(curr_servo3_pos);
  Serial.print("|");
  bool rotated = false;
  servo3_time = millis();
  while(!rotated){
    if(millis()-servo3_time > SERVO_INTERVAL_MEDIUM){
      servo3_time += SERVO_INTERVAL_MEDIUM;
      if(set_servo3_pos > curr_servo3_pos){
          curr_servo3_pos++;
          servo3.write(curr_servo3_pos);
      } else if(set_servo3_pos < curr_servo3_pos){
          curr_servo3_pos--;
          servo3.write(curr_servo3_pos);
          Serial.print("=");
      }
      else {
        rotated = true;
        Serial.print("|");
      }
    }
  }
}

void claw_vertical(){
  rotate_claw(0);
}

void claw_horizontal(){
  rotate_claw(90);
}

//claw control functions
void set_claw_angle(int set_servo4_pos){
  PrintDigits(curr_servo4_pos);
  Serial.print("|");
  bool angle_set = false;
  servo4_time = millis();
  while(!angle_set){
    if(millis()-servo4_time > SERVO_INTERVAL_FAST){
      servo4_time += SERVO_INTERVAL_FAST;
      if(set_servo4_pos > curr_servo4_pos){
          curr_servo4_pos++;
          servo4.write(curr_servo4_pos);
      } else if(set_servo4_pos < curr_servo4_pos){
          curr_servo4_pos--;
          servo4.write(curr_servo4_pos);
          Serial.print("=");
      }
      else {
        angle_set = true;
        Serial.print("|");
      }
    }
  }
}

void open_claw(){
  set_claw_angle(30);
}

void close_claw(){
  set_claw_angle(93);
}

//function for combined servo movements
void combined_movement(int set_servo1_pos, int set_servo2_pos, int set_servo3_pos, int set_servo4_pos, int movement_interval, int driving_mode, int left_motor_speed, int right_motor_speed){
  bool locked = false;
  bool servo1_angle_set = false;
  bool servo2_angle_set = false;
  bool servo3_angle_set = false;
  bool servo4_angle_set = false;
  combined_servo_time = millis();
  while(!locked){
    switch(driving_mode)
    {
      case 0:
        Serial.print(" STOPPED");
        MOTOR_STOP;
        Serial.print(" ... ... OK!");
        break;

      case 1:
        analogWrite(ENA, left_motor_speed);
        analogWrite(ENB, right_motor_speed);
        MOTOR_GO_FORWARD;
        break;
  
      case 2:
        analogWrite(ENA, left_motor_speed);
        analogWrite(ENB, right_motor_speed);
        MOTOR_GO_BACKWARD;
        break;
  
      case 3:
        analogWrite(ENA, left_motor_speed);
        analogWrite(ENB, right_motor_speed);
        MOTOR_GO_LEFT;
        break;
  
      case 4:
        analogWrite(ENA, left_motor_speed);
        analogWrite(ENB, right_motor_speed);
        MOTOR_GO_RIGHT;
        break;   
    }
    if(millis()-combined_servo_time > movement_interval){
      combined_servo_time += movement_interval;
      if(set_servo1_pos > curr_servo1_pos && !servo1_angle_set){
        curr_servo1_pos++;
        servo1.write(curr_servo1_pos);
      }
      else if(set_servo1_pos < curr_servo1_pos && !servo1_angle_set){
        curr_servo1_pos--;
        servo1.write(curr_servo1_pos);
      }
      else {
        servo1_angle_set = true;
      }
      
     if(set_servo2_pos > curr_servo2_pos && !servo2_angle_set){
        curr_servo2_pos++;
        servo2.write(curr_servo2_pos);
      } 
      else if(set_servo2_pos < curr_servo2_pos && !servo2_angle_set){
        curr_servo2_pos--;
        servo2.write(curr_servo2_pos);
      }
      else {
        servo2_angle_set = true;
      }
          
      if(set_servo3_pos > curr_servo3_pos && !servo3_angle_set){
        curr_servo3_pos++;
        servo3.write(curr_servo3_pos);
      }
      else if(set_servo3_pos < curr_servo3_pos && !servo3_angle_set){
        curr_servo3_pos--;
        servo3.write(curr_servo3_pos);
      }
      else {
        servo3_angle_set = true;
      }
      
      if(set_servo4_pos > curr_servo4_pos && !servo4_angle_set){
        curr_servo4_pos++;
        servo4.write(curr_servo4_pos);
      }
      else if(set_servo4_pos < curr_servo4_pos && !servo4_angle_set){
        curr_servo4_pos--;
        servo4.write(curr_servo4_pos);
      }
      else {
        servo4_angle_set = true;
      }
  
      if(servo1_angle_set && servo2_angle_set && servo3_angle_set && servo4_angle_set){
        locked = true;
      }
    } 
  }
}

void grab_90_degree(){
  combined_movement(2, 90, 90, 30, SERVO_INTERVAL_MEDIUM, 0, 0, 0 );
}

void put_down_90_degree(){
  combined_movement(2, 90, 90, 93, SERVO_INTERVAL_MEDIUM, 0, 0, 0 );
  delay(1000);
  open_claw();
}

void put_down_into_cup(){
  combined_movement(16, 120, 90, 93, SERVO_INTERVAL_MEDIUM, 0, 0, 0 );
  delay(1000);
  open_claw();
}

void grab_90_degree_and_return(){
  combined_movement(2, 90, 90, 30, SERVO_INTERVAL_MEDIUM, 0, 0, 0);
  delay(500);
  close_claw();
  delay(500);
  combined_movement(80, 90, 90, 93, SERVO_INTERVAL_MEDIUM, 0, 0, 0);
}

void return_to_original(){
//  combined_movement(50, 46, 90, 30, SERVO_INTERVAL_FAST, 0, 0, 0);
    combined_movement(90, 90, 90, 93, SERVO_INTERVAL_FAST, 0, 0, 0);
}

void enter_strike_position(){
  combined_movement(16, 25, 180, 93, SERVO_INTERVAL_MEDIUM, 0, 0, 0);
}

void strike(){
  combined_movement(21, 85, 180, 93, SERVO_INTERVAL_ULTRA_FAST, 1, 100, 100);
}

void prepare_seesaw(){
  combined_movement(85, 110, 180, 93, SERVO_INTERVAL_FAST, 0, 0, 0); //lift the arm
  delay(2000);
  combined_movement(2, 90, 180, 93, SERVO_INTERVAL_MEDIUM, 2, 50, 50); //press down the see-saw while backing slowly
}

//functions for micro servo movements

void servo_lift_x_degrees(Servo *servo, int *curr_servo_pos, int *servo_time, int degree){
  PrintDigits(*curr_servo_pos);
  Serial.print("|");
  if(*curr_servo_pos<=180){
    bool angle_set = false;
    *servo_time = millis();
    int new_servo_pos = *curr_servo_pos + degree;
    while(!angle_set){
      if(millis()-*servo_time > SERVO_INTERVAL_FAST){
        *servo_time += SERVO_INTERVAL_FAST;
        if(new_servo_pos > *curr_servo_pos){
            *curr_servo_pos = *curr_servo_pos + 1;
            servo->write(*curr_servo_pos);
            Serial.print("=");
        }
        else {
          angle_set = true;
          Serial.print("|");
        }
      }
    }
  }
}

void servo_lower_x_degrees(Servo *servo, int *curr_servo_pos, int *servo_time, int degree){
  PrintDigits(*curr_servo_pos);
  Serial.print("|");
  if(*curr_servo_pos>=0){
    bool angle_set = false;
    *servo_time = millis();
    int new_servo_pos = *curr_servo_pos - degree;
    while(!angle_set){
      if(millis()-*servo_time > SERVO_INTERVAL_FAST){
        *servo_time += SERVO_INTERVAL_FAST;
        if(new_servo_pos < *curr_servo_pos){
            *curr_servo_pos = *curr_servo_pos - 1;
            servo->write(*curr_servo_pos);
            Serial.print("=");
        }
        else {
          angle_set = true;
          Serial.print("|");
        }
      }
    }
  }
}

void servo1_lift_x_degrees(int x){
  servo_lift_x_degrees(&servo1, &curr_servo1_pos, &servo1_time, x);
}

void servo2_lift_x_degrees(int x){
  servo_lift_x_degrees(&servo2, &curr_servo2_pos, &servo2_time, x);
}

void servo3_turn_x_degrees_clockwise(int x){
  servo_lift_x_degrees(&servo3, &curr_servo3_pos, &servo3_time, x);
}

void servo1_lower_x_degrees(int x){
  servo_lower_x_degrees(&servo1, &curr_servo1_pos, &servo1_time, x);
}

void servo2_lower_x_degrees(int x){
  servo_lower_x_degrees(&servo2, &curr_servo2_pos, &servo2_time, x);
}

void servo3_turn_x_degrees_anticlockwise(int x){
  servo_lower_x_degrees(&servo3, &curr_servo3_pos, &servo3_time, x);
}

void ledStatus()
{
  if (led == 0)
  {
    led = 1;
    digitalWrite(LED, led);
    Serial.print(" ON ");
  }
    else
  {
    led = 0;
    digitalWrite(LED, led);
    Serial.print(" OFF");
  }
}

/*  Handle incomming commands and do respective actions
 *  
 */

void handleCommands()
{
  switch(cmd[0])
  {
    case ' ':
      Serial.print(" STOPPED");
      MOTOR_STOP;
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'W':
      Serial.print(" MVM FWD");
      MOTOR_GO_FORWARD;
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'S':
      Serial.print(" MVM BKD");
      MOTOR_GO_BACKWARD;
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'A':
      Serial.print(" MVM LFT");
      MOTOR_GO_LEFT;
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'D':
      Serial.print(" MVM RHT");
      MOTOR_GO_RIGHT;
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'Q':
      Serial.print(" CLW <=>");
      open_claw();
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'E':
      Serial.print(" CLW >=<");
      close_claw();
      Serial.print(" ... ... ... ............ OK!");
      break;
      
    case '1':

      break;
    
    case '2':

      break;

    case '3':

      break;

    case '4':
      Serial.print(" SV1 VVV 10  DEG ");
      servo1_lower_x_degrees(10);
      Serial.print(" OK!");
      break;

    case '5':
      Serial.print(" SV2 ΛΛΛ 10  DEG ");
      servo2_lower_x_degrees(10);
      Serial.print(" OK!");
      break;

    case '6':
      Serial.print(" SV3 <<< 10  DEG ");
      servo3_turn_x_degrees_anticlockwise(10);
      Serial.print(" OK!");
      break;

    case '7':
      Serial.print(" SV1 ΛΛΛ 10  DEG ");
      servo1_lift_x_degrees(10);
      Serial.print(" OK!");
      break;

    case '8':
      Serial.print(" SV2 ΛΛΛ 10  DEG ");
      servo2_lift_x_degrees(10);
      Serial.print(" OK!");
      break;

    case '9':
      Serial.print(" SV3 >>> 10  DEG ");
      servo3_turn_x_degrees_clockwise(10);
      Serial.print(" OK!");
      break;

    case 'U':
      Serial.print(" GRB & RETURN");
      grab_90_degree_and_return();
      Serial.print(" OK!");
      break;

    case 'X':
      Serial.print(" RESET");
      return_to_original();
      Serial.print(" . ... ... ... ...RESET!... OK!");
      break;

    case 'L':
      Serial.print(" LHT");
      ledStatus();
      Serial.print(" ... ... ... ............ OK!");
      break;

    case 'B':
      Serial.print(" PRP");
      enter_strike_position();
      Serial.print(" ... ... ... ... ............ OK!");
      break;

    case 'N':
      Serial.print(" SHT");
      strike();
      Serial.print(" ... ... ... ... ............ OK!");
      break;

    default:
      Serial.print(" ERR ERR ERR ...ERROR!... ERR");
      break;
  }
}

/*  interpret commands from Pi
 *  readSerialPort()    Read serial port from RPi
 *  sendData()          Re-iterate command to RPi
 *  PrintDigits()       Print exactly 4 digits
 */

void readSerialPort() {
  cmd = "";
  if (Serial.available()) {
      delay(10);
      while (Serial.available() > 0) {
          cmd += (char)Serial.read();
      }
      Serial.flush();
  }
}

void sendData() {
  //write data
  Serial.print("[ ");
  Serial.print(cmd[0]);
  Serial.print(" ] : ");
}

void setup() {
  // join I2C bus
  Wire.begin(); 
  
  //begin serial communication with Pi
  Serial.begin(9600);
  Serial.print("[INI] :  COM");
  
  //setup servo motors
  Serial.print(" SVO");
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);

  //setup driver motors
  Serial.print(" MTR");
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //setup LED
  pinMode(LED, OUTPUT);

  //activate driver motors
  Serial.print(" ACT");
  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed);
//
//  // initialize device Serial.println("Initializing I2C devices..."); 
//  gyroscope.initialize();
//  // verify connection Serial.println("Testing device connections..."); 
//  gyroscope_connected = gyroscope.testConnection();
//  Serial.println( gyroscope_connected ? "GYRO CONNECTED" : "GYRO CONNECT FAIL");
// 
//  Serial.println(" OK!");
}

void loop() { 
  readSerialPort();
  if (cmd != "") {
      sendData();
      handleCommands();
      Serial.println(" >>");
  }
//  if (gyroscope_connected) {
//    gyroscope.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    Serial.print("ax: "); Serial.print(ax);Serial.print(" ");
//    Serial.print("ay: "); Serial.print(ay);Serial.print(" ");
//    Serial.print("az: "); Serial.print(az);Serial.print(" ");
//    Serial.print("gx: "); Serial.print(gx);Serial.print(" ");
//    Serial.print("gy: "); Serial.print(gy);Serial.print(" ");
//    Serial.print("gz: ");Serial.print(gz);Serial.println(" ");
//    
//  }
//  delay(3000);
}
