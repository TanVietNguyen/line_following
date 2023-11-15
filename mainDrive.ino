#include <ECE3.h>

uint16_t sensorValues[8];

float fusionValue;

float kd = 0.015;
float kp = 0.015;
float newError = 612;
float oldError = 612;
float deltaError;

float SV;
int spinCheck = 0;
float sum;

//PIN ASSIGNMENT
//left
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
//right
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;


//motor speed
int speed;
void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd);
void setup()
{
  ECE3_Init();
 // Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  speed = 50; //This is our Base speed
  
  delay(5000); //~5 second delay from start
}

void loop()
{
    // read raw sensor values
    ECE3_read_IR(sensorValues);
      
    //normalize the average values 
    //min values 826 641 756 641 710 664 710 779
    //max values 1392.6 1160  1259  1019  807 1422  1428  1383
    float sens1 = ((sensorValues[0] - 695.8) * 1000 )/ 1176.6;
    float sens2 = ((sensorValues[1] - 587) * 1000 )/ 1081;
    float sens3 = ((sensorValues[2] - 672.4) * 1000 )/ 1211.6; 
    float sens4 = ((sensorValues[3] - 566) * 1000 )/ 978.4;
    float sens5 = ((sensorValues[4] - 608) * 1000 )/ 794.6;
    float sens6 = ((sensorValues[5] - 566) * 1000 )/ 1338.8;
    float sens7 = ((sensorValues[6] - 720.6) * 1000 )/ 1091.1;
    float sens8 = ((sensorValues[7] - 790.2) * 1000 )/ 901.2;
    //get fusion output, currently there are two weighting scheme, 8-4-2-1,15-14-12-8, can be adjusted. 
    fusionValue = (sens1*(-15) + sens2*(-14) + sens3*(-12) + sens4*(-8) + sens5*(8) + sens6*12 + sens7*14 + sens8*15 )/8;
    sum = sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7];
  
    oldError = newError;

    deltaError = newError - oldError; //differnce in error position
    SV = kp * fusionValue + kd * deltaError; // this is our steering value that will adjust motors

    //THIS IS OUR END OF TRACK 180 CONDITION
    //This checks for end of track and adds to a counter
    //There are still some issue, for some reason it only work 50% of the time, other times it will talk right off.
    if(sum > 17000){
      spinCheck ++;
    }
    else{
      spinCheck = 0;  
    }
    if(sum > 17000 && spinCheck > 3){
      ChangeBaseSpeed(speed, 0);
      
      delay(1000);
      digitalWrite(left_dir_pin,LOW); //sets left to forward
      digitalWrite(right_dir_pin,HIGH); //set right to backwards
      ChangeBaseSpeed(0, speed);
      analogWrite(left_pwm_pin, speed); //both are set to same speed
      analogWrite(right_pwm_pin, speed);

      delay(1450); // THIS IS THE TIME FOR IT TO SPIN, CALIBRATED FOR 50 SPEED MUST CHANGE AS SPEED CHANGES
      ChangeBaseSpeed(speed, 0);
      delay(1000);
      digitalWrite(right_dir_pin,LOW);
      ChangeBaseSpeed(0, speed);
      spinCheck = 0;
    }

    //if we arent at the end of the track then we adjust our speed to for correction
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(right_dir_pin,LOW);
    //if this seems like it backwards, its not. Due to the senor orientation this is our output.
    analogWrite(left_pwm_pin, speed - SV);
    analogWrite(right_pwm_pin, speed + SV);
    
    newError = fusionValue;
 }
 
 void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {

  //Changes speed from initial to final over ~30ms
  int numSteps = 5;
  int pwmVal = initialBaseSpd; // initialize left wheel speed 
  int deltaSpeed = (finalBaseSpd-initialBaseSpd)/numSteps; // in(de)crement
  for(int k = 0; k<numSteps ;k++) {
    pwmVal = pwmVal + deltaSpeed;
    analogWrite(left_pwm_pin,pwmVal);
    analogWrite(right_pwm_pin,pwmVal);
    delay(30);
  } 
} 