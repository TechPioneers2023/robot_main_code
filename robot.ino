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
#define SERVO_INTERVAL 10

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

//Head Light Status, default 0 (OFF)
bool led = 0;

//serial commands
char command = '\n';
String  cmd;

//servo1 rotation functions
void rotate_base(int set_servo1_pos){
  bool angle_set = false;
  while(millis()-servo1_time > SERVO_INTERVAL && !angle_set){
    servo1_time += SERVO_INTERVAL;
    if(set_servo1_pos > curr_servo1_pos){
        curr_servo1_pos++;
    } else if(set_servo1_pos < curr_servo1_pos){
        curr_servo1_pos--;
    }
    else {
      angle_set = true;
    }
    servo1.write(curr_servo1_pos);
  }
}

void base_bend_90_front(){
  rotate_base(5);
}

void base_straighten(){
  rotate_base(90);
}

void base_bend_90_back(){
  rotate_base(180);
}

//servo2 rotation functions
void rotate_joint(int set_servo2_pos){
  bool angle_set = false;
  while(millis()-servo2_time > SERVO_INTERVAL && !angle_set){
    servo2_time += SERVO_INTERVAL;
    if(set_servo2_pos > curr_servo2_pos){
        curr_servo2_pos++;
    } else if(set_servo2_pos < curr_servo2_pos){
        curr_servo2_pos--;
    }
    else {
      angle_set = true;
    }
    servo2.write(curr_servo2_pos);
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
  bool rotated = false;
  while(millis()-servo3_time > SERVO_INTERVAL && !rotated){
    servo3_time += SERVO_INTERVAL;
    if(set_servo3_pos > curr_servo3_pos){
        curr_servo3_pos++;
    } else if(set_servo3_pos < curr_servo3_pos){
        curr_servo3_pos--;
    }
    else {
      rotated = true;
    }
    servo3.write(curr_servo3_pos);
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
  bool angle_set = false;
  while(millis()-servo4_time > SERVO_INTERVAL && !angle_set){
    servo4_time += SERVO_INTERVAL;
    if(set_servo4_pos > curr_servo4_pos){
        curr_servo4_pos++;
    } else if(set_servo4_pos < curr_servo4_pos){
        curr_servo4_pos--;
    }
    else {
      angle_set = true;
    }
    servo4.write(curr_servo4_pos);
  }
}

void open_claw(){
  set_claw_angle(30);
}

void close_claw(){
  set_claw_angle(93);
}

//function for combined servo movements
void combined_grab(int set_servo1_pos, int set_servo2_pos, int set_servo3_pos, int set_servo4_pos){
  bool locked = false;
  bool servo1_angle_set = false;
  bool servo2_angle_set = false;
  bool servo3_angle_set = false;
  bool servo4_angle_set = false;
  while(millis()-combined_servo_time > SERVO_INTERVAL && !locked){
    combined_servo_time += SERVO_INTERVAL;
    if(set_servo1_pos > curr_servo1_pos && !servo1_angle_set){
      curr_servo1_pos = curr_servo1_pos+0.1;
      servo1.write(curr_servo1_pos);
    }
    else if(set_servo1_pos < curr_servo1_pos && !servo1_angle_set){
      curr_servo1_pos = curr_servo1_pos-0.1;
      servo1.write(curr_servo1_pos);
    }
    else {
      servo1_angle_set = true;
    }
    
   if(set_servo2_pos > curr_servo2_pos && !servo2_angle_set){
      curr_servo2_pos = curr_servo2_pos+0.1;
      servo2.write(curr_servo2_pos);
    } 
    else if(set_servo2_pos < curr_servo2_pos && !servo2_angle_set){
      curr_servo2_pos = curr_servo2_pos-0.1;
      servo2.write(curr_servo2_pos);
    }
    else {
      servo2_angle_set = true;
    }
        
    if(set_servo3_pos > curr_servo3_pos && !servo3_angle_set){
      curr_servo3_pos = curr_servo3_pos+0.1;
      servo3.write(curr_servo3_pos);
    }
    else if(set_servo3_pos < curr_servo3_pos && !servo3_angle_set){
      curr_servo3_pos = curr_servo3_pos-0.1;
      servo3.write(curr_servo3_pos);
    }
    else {
      servo3_angle_set = true;
    }
    
    if(set_servo4_pos > curr_servo4_pos && !servo4_angle_set){
      curr_servo4_pos = curr_servo4_pos+0.1;
      servo4.write(curr_servo4_pos);
    }
    else if(set_servo4_pos < curr_servo4_pos && !servo4_angle_set){
      curr_servo4_pos = curr_servo4_pos-0.1;
      servo4.write(curr_servo4_pos);
    }
    else {
      servo4_angle_set = true;
    }

    if(servo1_angle_set && servo2_angle_set && servo3_angle_set && servo4_angle_set){
      locked = true;
      Serial.print(" APPROACHED ");
    }
  }
}

void grab_90_degree(){
  combined_grab(5, 90, 0, 30);
//  base_bend_90_front();
//  bend_arm_90_degrees();
//  open_claw();
}

void return_to_original(){
  combined_grab(90, 90, -180, 20);
}

void servo1_lift_10degrees(){
  PrintDigits(curr_servo1_pos);
  Serial.print("|");
  if(curr_servo1_pos<=120){
    bool angle_set = false;
    servo1_time = 0;
    int new_servo1_pos = curr_servo1_pos + 10;
    while(millis()-servo1_time > SERVO_INTERVAL && !angle_set){
      servo1_time += SERVO_INTERVAL;
      if(new_servo1_pos > curr_servo1_pos){
          curr_servo1_pos++;
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

void servo2_lift_10degrees(){
  PrintDigits(curr_servo2_pos);
  Serial.print("|");
  if(curr_servo2_pos<=180){
    bool angle_set = false;
    servo2_time = 0;
    int new_servo2_pos = curr_servo2_pos + 10;
    while(millis()-servo2_time > SERVO_INTERVAL && !angle_set){
      servo2_time += SERVO_INTERVAL;
      if(new_servo2_pos > curr_servo2_pos){
          curr_servo2_pos++;
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

void servo3_turn_10degrees_clockwise(){
  PrintDigits(curr_servo3_pos);
  Serial.print("|");
  if(curr_servo3_pos<=180){
    bool angle_set = false;
    servo1_time = 0;
    int new_servo3_pos = curr_servo3_pos + 10;
    while(millis()-servo3_time > SERVO_INTERVAL && !angle_set){
      servo3_time += SERVO_INTERVAL;
      if(new_servo3_pos > curr_servo3_pos){
          curr_servo3_pos++;
          servo3.write(curr_servo3_pos);
          Serial.print("=");
      }
      else {
        angle_set = true;
        Serial.print("|");
      }
    }
  }
}

void servo1_lower_10degrees(){
  PrintDigits(curr_servo1_pos);
  Serial.print("|");
  if(curr_servo1_pos>=0){
    bool angle_set = false;
    servo1_time = 0;
    int new_servo1_pos = curr_servo1_pos - 10;
    while(millis()-servo1_time > SERVO_INTERVAL && !angle_set){
      servo1_time += SERVO_INTERVAL;
      if(new_servo1_pos < curr_servo1_pos){
          curr_servo1_pos--;
          servo1.write(curr_servo1_pos);
          Serial.print("=");
          //Serial.println(curr_servo1_pos);
      }
      else {
        angle_set = true;
        Serial.print("|");
      }
    }
  }
}

void servo2_lower_10degrees(){
  PrintDigits(curr_servo2_pos);
  Serial.print("|");
  if(curr_servo2_pos>=0){
    bool angle_set = false;
    servo2_time = 0;
    int new_servo2_pos = curr_servo2_pos - 10;
    while(millis()-servo2_time > SERVO_INTERVAL && !angle_set){
      servo2_time += SERVO_INTERVAL;
      if(new_servo2_pos < curr_servo2_pos){
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

void servo3_turn_10degrees_anticlockwise(){
  PrintDigits(curr_servo3_pos);
  Serial.print("|");
  if(curr_servo3_pos>=0){
    bool angle_set = false;
    servo3_time = 0;
    int new_servo3_pos = curr_servo3_pos - 10;
    while(millis()-servo3_time > SERVO_INTERVAL && !angle_set){
      servo1_time += SERVO_INTERVAL;
      if(new_servo3_pos < curr_servo3_pos){
          curr_servo3_pos--;
          servo3.write(curr_servo3_pos);
          Serial.print("=");
      }
      else {
        angle_set = true;
        Serial.print("|");
      }
    }
  }
}

void ledStatus()
{
  if (led == 0)
  {
    led = 1;
    Serial.print(" ON ");
  }
    else
  {
    led = 0;
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
      Serial.print(" ... ... OK!");
      break;

    case 'W':
      Serial.print(" MVM FWD");
      MOTOR_GO_FORWARD;
      Serial.print(" ... ... OK!");
      break;

    case 'S':
      Serial.print(" MVM BKD");
      MOTOR_GO_BACKWARD;
      Serial.print(" ... ... OK!");
      break;

    case 'A':
      Serial.print(" MVM LFT");
      MOTOR_GO_LEFT;
      Serial.print(" ... ... OK!");
      break;

    case 'D':
      Serial.print(" MVM RHT");
      MOTOR_GO_RIGHT;
      Serial.print(" ... ... OK!");
      break;

    case 'Q':
      Serial.print(" CLW <=>");
      open_claw();
      Serial.print(" ... ... OK!");
      break;

    case 'E':
      Serial.print(" CLW >=<");
      close_claw();
      Serial.print(" ... ... OK!");
      break;

    case '4':
      Serial.println(" SV1 VVV 10  DEG");
      Serial.print("         ");
      servo1_lower_10degrees();
      Serial.print("OK!");
      break;

    case '5':
      Serial.println(" SV2 ΛΛΛ 10  DEG");
      Serial.print("         ");
      servo2_lower_10degrees();
      Serial.print("OK!");
      break;

    case '6':
      Serial.println(" SV3 <<< 10  DEG");
      Serial.print("         ");
      servo3_turn_10degrees_anticlockwise();
      Serial.print("OK!");
      break;

    case '7':
      Serial.println(" SV1 ΛΛΛ 10  DEG");
      Serial.print("         ");
      servo1_lift_10degrees();
      Serial.print("OK!");
      break;

    case '8':
      Serial.println(" SV2 ΛΛΛ 10  DEG");
      Serial.print("         ");
      servo2_lift_10degrees();
      Serial.print("OK!");
      break;

    case '9':
      Serial.println(" SV3 >>> 10  DEG");
      Serial.print("         ");
      servo3_turn_10degrees_clockwise();
      Serial.print("OK!");
      break;

    case 'U':
      Serial.print(" GRB");
      grab_90_degree();
      Serial.print(" OK!");
      break;

    case 'L':
      Serial.print(" LHT");
      ledStatus();
      Serial.print(" ... ... OK!");
      break;
    
    default:
    Serial.print(" ERR ERR ERR ERR ERR");
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

void setup() {
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

  //activate driver motors
  Serial.print(" ACT");
  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed);
 
  Serial.println(" OK!");
}

void loop() { 
  readSerialPort();
  if (cmd != "") {
      sendData();
      handleCommands();
      Serial.println(" >>");
  }
  delay(200);
}
