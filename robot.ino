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
#define SERVO_INTERVAL 1000

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

//serial commands
char command = '\n';

//servo1 rotation functions
void rotate_base(int set_servo1_pos){
  bool angle_set = false;
  servo1_time = millis();
  while(!angle_set){
    if(millis()-servo1_time > SERVO_INTERVAL){
       servo1_time += SERVO_INTERVAL;
      if(set_servo1_pos > curr_servo1_pos){
          curr_servo1_pos++;
          servo1.write(curr_servo1_pos);
      } else if(set_servo1_pos < curr_servo1_pos){
          curr_servo1_pos--;
          servo1.write(curr_servo1_pos);
      }
      else {
        angle_set = true;
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
  bool angle_set = false;
  servo2_time = millis();
  while(!angle_set){
    if(millis()-servo2_time > SERVO_INTERVAL){
      servo2_time += SERVO_INTERVAL;
      if(set_servo2_pos > curr_servo2_pos){
          curr_servo2_pos++;
          servo2.write(curr_servo2_pos);
      } else if(set_servo2_pos < curr_servo2_pos){
          curr_servo2_pos--;
          servo2.write(curr_servo2_pos);
      }
      else {
        angle_set = true;
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
  bool rotated = false;
  servo3_time = millis();
  while(!rotated){
    if(millis()-servo3_time > SERVO_INTERVAL){
      servo3_time += SERVO_INTERVAL;
      if(set_servo3_pos > curr_servo3_pos){
          curr_servo3_pos++;
          servo3.write(curr_servo3_pos);
      } else if(set_servo3_pos < curr_servo3_pos){
          curr_servo3_pos--;
          servo3.write(curr_servo3_pos);
      }
      else {
        rotated = true;
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
  bool angle_set = false;
  servo4_time = millis();
  while(!angle_set){
    if(millis()-servo4_time > SERVO_INTERVAL){
      servo4_time += SERVO_INTERVAL;
      if(set_servo4_pos > curr_servo4_pos){
          curr_servo4_pos++;
          servo4.write(curr_servo4_pos);
      } else if(set_servo4_pos < curr_servo4_pos){
          curr_servo4_pos--;
          servo4.write(curr_servo4_pos);
      }
      else {
        angle_set = true;
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
void combined_grab(int set_servo1_pos, int set_servo2_pos, int set_servo3_pos, int set_servo4_pos){
  bool locked = false;
  bool servo1_angle_set = false;
  bool servo2_angle_set = false;
  bool servo3_angle_set = false;
  bool servo4_angle_set = false;
  combined_servo_time = millis();
  while(!locked){
    if(millis()-combined_servo_time > SERVO_INTERVAL){
      combined_servo_time += SERVO_INTERVAL;
      if(set_servo1_pos > curr_servo1_pos && !servo1_angle_set){
        curr_servo1_pos++;
        servo1.write(curr_servo1_pos);
      }
      else if(set_servo1_pos < curr_servo1_pos && !servo1_angle_set){
        curr_servo1_pos = curr_servo1_pos--;
        servo1.write(curr_servo1_pos);
      }
      else {
        servo1_angle_set = true;
      }
      
     if(set_servo2_pos > curr_servo2_pos && !servo2_angle_set){
        curr_servo2_pos = curr_servo2_pos++;
        servo2.write(curr_servo2_pos);
      } 
      else if(set_servo2_pos < curr_servo2_pos && !servo2_angle_set){
        curr_servo2_pos = curr_servo2_pos--;
        servo2.write(curr_servo2_pos);
      }
      else {
        servo2_angle_set = true;
      }
          
      if(set_servo3_pos > curr_servo3_pos && !servo3_angle_set){
        curr_servo3_pos = curr_servo3_pos++;
        servo3.write(curr_servo3_pos);
      }
      else if(set_servo3_pos < curr_servo3_pos && !servo3_angle_set){
        curr_servo3_pos = curr_servo3_pos--;
        servo3.write(curr_servo3_pos);
      }
      else {
        servo3_angle_set = true;
      }
      
      if(set_servo4_pos > curr_servo4_pos && !servo4_angle_set){
        curr_servo4_pos = curr_servo4_pos++;
        servo4.write(curr_servo4_pos);
      }
      else if(set_servo4_pos < curr_servo4_pos && !servo4_angle_set){
        curr_servo4_pos = curr_servo4_pos--;
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
  combined_grab(5, 90, 0, 30);
//  base_bend_90_front();
//  bend_arm_90_degrees();
//  open_claw();
}

void return_to_original(){
  combined_grab(90, 90, 0, 20);
}

void servo1_lift_10degrees(){
  Serial.println("servo1 degree");
  Serial.println(curr_servo1_pos);
  if(curr_servo1_pos<=120){
    bool angle_set = false;
    servo1_time = millis();
    int new_servo1_pos = curr_servo1_pos + 10;
    while(millis()-servo1_time > SERVO_INTERVAL && !angle_set){
      servo1_time += SERVO_INTERVAL;
      if(new_servo1_pos > curr_servo1_pos){
          curr_servo1_pos++;
          servo1.write(curr_servo1_pos);
          Serial.println(curr_servo1_pos);
      }
      else {
        angle_set = true;
      }
    }
  }
}

void servo2_lift_10degrees(){
  if(curr_servo2_pos<=180){
    bool angle_set = false;
    servo2_time = millis();
    int new_servo2_pos = curr_servo2_pos + 10;
    while(millis()-servo2_time > SERVO_INTERVAL && !angle_set){
      servo2_time += SERVO_INTERVAL;
      if(new_servo2_pos > curr_servo2_pos){
          curr_servo2_pos++;
          servo2.write(curr_servo2_pos);
      }
      else {
        angle_set = true;
      }
    }
  }
}

void servo3_turn_10degrees_clockwise(){
  Serial.println("servo3 degree");
  Serial.println(curr_servo3_pos);
  if(curr_servo3_pos<=180){
    bool angle_set = false;
    servo3_time = millis();
    int new_servo3_pos = curr_servo3_pos + 10;
    while(millis()-servo3_time > SERVO_INTERVAL && !angle_set){
      servo3_time += SERVO_INTERVAL;
      if(new_servo3_pos > curr_servo3_pos){
          curr_servo3_pos++;
          servo3.write(curr_servo3_pos);
          Serial.println(curr_servo3_pos);
      }
      else {
        angle_set = true;
      }
    }
  }
}

void servo1_lower_10degrees(){
  Serial.println("servo1 degree");
  Serial.println(curr_servo1_pos);
  if(curr_servo1_pos>=0){
    bool angle_set = false;
    servo1_time = millis();
    int new_servo1_pos = curr_servo1_pos - 10;
    while(millis()-servo1_time > SERVO_INTERVAL && !angle_set){
      servo1_time += SERVO_INTERVAL;
      if(new_servo1_pos < curr_servo1_pos){
          curr_servo1_pos--;
          servo1.write(curr_servo1_pos);
          Serial.println(curr_servo1_pos);
      }
      else {
        angle_set = true;
      }
    }
  }
}

void servo2_lower_10degrees(){
  if(curr_servo2_pos>=0){
    bool angle_set = false;
    servo2_time = millis();
    int new_servo2_pos = curr_servo2_pos - 10;
    while(millis()-servo2_time > SERVO_INTERVAL && !angle_set){
      servo2_time += SERVO_INTERVAL;
      if(new_servo2_pos < curr_servo2_pos){
          curr_servo2_pos--;
          servo2.write(curr_servo2_pos);
      }
      else {
        angle_set = true;
      }
    }
  }
}

void servo3_turn_10degrees_anticlockwise(){
  Serial.println("servo3 degree");
  Serial.println(curr_servo3_pos);
  if(curr_servo3_pos>=0){
    bool angle_set = false;
    servo3_time = millis();
    int new_servo3_pos = curr_servo3_pos - 10;
    while(millis()-servo3_time > SERVO_INTERVAL && !angle_set){
      servo1_time += SERVO_INTERVAL;
      if(new_servo3_pos < curr_servo3_pos){
          curr_servo3_pos--;
          servo3.write(curr_servo3_pos);
          Serial.println(curr_servo3_pos);
      }
      else {
        angle_set = true;
      }
    }
  }
}

void handleCommands(char cmd)
{
  switch(cmd)
  {
    case 'Z':
      MOTOR_STOP;
      break;

    case 'W':
      MOTOR_GO_FORWARD;
      break;

    case 'S':
      MOTOR_GO_BACKWARD;
      break;

    case 'A':
      MOTOR_GO_LEFT;
      break;

    case 'D':
      MOTOR_GO_RIGHT;
      break;

    case 'Q':
      open_claw();
      break;

    case 'E':
      close_claw();
      break;

    case '4':
      servo1_lower_10degrees();
      break;

    case '5':
      servo2_lower_10degrees();
      break;

    case '6':
      servo3_turn_10degrees_anticlockwise();
      break;

    case '7':
      servo1_lift_10degrees();
      break;

    case '8':
      servo2_lift_10degrees();
      break;

    case '9':
      servo3_turn_10degrees_clockwise();
      break;

    case 'U':
      grab_90_degree();
      break;
    
    
   default:
    int c = Serial.parseInt();
    Serial.print(c);
    rotate_base(c);
    break;
  }
}

//interpret commands from Pi

void readSerial()
{
  while(Serial.available()){
    command = Serial. read();
    handleCommands(command);
  }
}

void setup() {
  //setup servo motors
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  
  //setup driver motors
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //activate driver motors
  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed);
  
  //begin serial communication with Pi
  Serial.begin(9600);
}

void loop() { 
  readSerial();
  
}
