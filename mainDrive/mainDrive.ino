#include <ECE3.h>

uint16_t sensorValues[8];

float fusionValue;
int encoderCount;
float kd = 0.012;
//float kp = 0.01;
float kp = 0.015;
float newError = -112;
float oldError = -112;
float deltaError;
int spinNum = 0;
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
int speed = 35; //This is our Base speed
void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd);
void setup()
{
  ECE3_Init();
 Serial.begin(9600); // set the data rate in bits per second for serial data transmission

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
  delay(1000); //~5 second delay from start
}

void loop()
{
    // read raw sensor values
    ECE3_read_IR(sensorValues);
      
    //normalize the average values 
    //min values 826 641 756 641 710 664 710 779
    //max values 1392.6 1160  1259  1019  807 1422  1428  1383
//    float sens1 = ((sensorValues[0] - 695.8) * 1000 )/ 1176.6;
//    float sens2 = ((sensorValues[1] - 587) * 1000 )/ 1081;
//    float sens3 = ((sensorValues[2] - 672.4) * 1000 )/ 1211.6; 
//    float sens4 = ((sensorValues[3] - 566) * 1000 )/ 978.4;
//    float sens5 = ((sensorValues[4] - 608) * 1000 )/ 794.6;
//    float sens6 = ((sensorValues[5] - 566) * 1000 )/ 1338.8;
//    float sens7 = ((sensorValues[6] - 720.6) * 1000 )/ 1091.1;
//    float sens8 = ((sensorValues[7] - 790.2) * 1000 )/ 901.2;
    //get fusion output, currently there are two weighting scheme, 8-4-2-1,15-14-12-8, can be adjusted. 
    //fusionValue = (sens1*(6) + sens2*(-14) + sens3*(-12) + sens4*(-8) + sens5*(8) + sens6*12 + sens7*14 + sens8*6 )/8;
    //fusionValue = (sens1*(-15) + sens2*(-14) + sens3*(-12) + sens4*(-8) + sens5*(8) + sens6*12 + sens7*14 + sens8*15 )/8;

    
    //This is TA's recommendation, his reasoning is that normalization is to make sure all sensors read the same value, but it is
    //not necessary in this case. I don't really fully understand, but it kind of makes sense to me and it actually works!
    float sens1 = sensorValues[0];
    float sens2 = sensorValues[1];
    float sens3 = sensorValues[2]; 
    float sens4 = sensorValues[3];
    float sens5 = sensorValues[4];
    float sens6 = sensorValues[5];
    float sens7 = sensorValues[6];
    float sens8 = sensorValues[7];

    encoderCount = average();
   //When the encoderCount is in these value ranges, the car is going through black area
    if ((encoderCount > 250 && encoderCount < 1200)||(encoderCount > 3800 && encoderCount < 4800)||(encoderCount > 5400 && encoderCount < 6200)){
       kp = 0.02; //This Kp and weighting scheme work perfectly during those turns
       fusionValue = (sens1*(0) + sens2*(-14) + sens3*(-12) + sens4*(-8) + sens5*(8) + sens6*12 + sens7*14 + sens8*0 )/8;
       SV = kp * fusionValue; // this is our steering value that will adjust motors
    }
    //Other parts of the track, getting back to our original PID and weighting scheme!
    else{
      newError = fusionValue; 
      deltaError = newError - oldError; 
      fusionValue = (sens1*(-15) + sens2*(-14) + sens3*(-12) + sens4*(-8) + sens5*(8) + sens6*12 + sens7*14 + sens8*15)/8;
      SV = kp * fusionValue + kd * deltaError; // this is our steering value that will adjust motors
      oldError = newError; 
    }
    
    sum = sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7];

    //THIS IS OUR END OF TRACK 180 CONDITION
    //This checks for end of track and adds to a counter
    //There are still some issue, for some reason it only work 50% of the time, other times it will talk right off.;
    
    if(sum > 17000){
      spinCheck ++;
    }
    else{
      spinCheck = 0;  
    }
    if(spinCheck > 3){
      //the car does a spin when reaching the end, but it has to stop for the next time when it's coming back.
      // it has not done any spin for the first time it reaches the end, so spinNum = 0.
      //After that, we increment the spinNum by 1, so the nex time it will stop.
      if(spinNum == 0){
      ChangeBaseSpeed(speed, 0);
      digitalWrite(right_dir_pin,HIGH); //set right to backwards
      ChangeBaseSpeed(0, 110);
      delay(500); // THIS IS THE TIME FOR IT TO SPIN, CALIBRATED FOR 50 SPEED MUST CHANGE AS SPEED CHANGES
      ChangeBaseSpeed(110, 0);
      digitalWrite(right_dir_pin,LOW);
      ChangeBaseSpeed(0, speed);
      spinCheck = 0;
      spinNum++;
      }
      else{
        speed = 0;
      }
      
    }

    //if we arent at the end of the track then we adjust our speed to for correction
    //if this seems like it backwards, its not. Due to the senor orientation this is our output.
    analogWrite(left_pwm_pin, speed - SV); 
    analogWrite(right_pwm_pin, speed + SV);
      //Serial.print(average());
      //Serial.print("\n");

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
int average()
{
  int getL = getEncoderCount_left();
  int getR = getEncoderCount_right();
  return (getL+getR)/2;
}
