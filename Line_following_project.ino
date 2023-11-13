#include <ECE3.h>

uint16_t sensorValues[8];
int AvgSenValue[8];
int fusionValue;
// Bump Switches
uint16_t bumpSw2_pin = 6;
uint16_t bumpSw2Reading;
uint16_t bumpSw3_pin = 27;
uint16_t bumpSw3Reading;

// User Switches (sides of LaunchPad)
uint16_t userSw1_pin = 73;
int userSw1Reading;
uint16_t userSw2_pin = 74;
int userSw2Reading;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  pinMode(bumpSw2_pin, INPUT_PULLUP);
  pinMode(bumpSw3_pin, INPUT_PULLUP);
  pinMode(userSw1_pin,INPUT_PULLUP);
  pinMode(userSw2_pin,INPUT_PULLUP);

  delay(2000);
}

void loop()
{
  AvgSenValue = [ 0, 0, 0, 0, 0, 0, 0, 0 ];
  // read raw sensor values
  if (digitalRead(bumpSw3_pin) == 0 || digitalRead(userSw2_pin) == 0)// why not reading sensor values in this case?
  {
    Serial.print("-999");
    Serial.println();
    delay(500);
  }
  bumpSw2Reading = digitalRead(bumpSw2_pin);
  userSw1Reading = digitalRead(userSw1_pin);
  if (bumpSw2Reading == 0 || userSw1Reading == 0) {
    for (int k = 0; k < 5; k++) {
      ECE3_read_IR(sensorValues);
      // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
      // 2500 means minimum reflectance
      for (unsigned char i = 0; i < 8; i++)
      {
        avgSenValue[i] += sensorValues[i];
        Serial.print(sensorValues[i]);
        Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
      }
      //      Serial.print(readCount);
      //      Serial.print('\t');
      //      Serial.print(bumpSw2Reading);
      Serial.println();
      delay(1000);//Conider lowering the delay time to increase the car's reacting time
    } 
    //take the average value of each sensor
    //Do we have to take the average of 5 values? or less?
    for (unsigned char i = 0; i < 8; i++){
      avgSenValue[i] = avgSenValue[i] / 8;
    }
    //normalize the average values 
    //min values 826 641 756 641 710 664 710 779
    //max values 1392.6 1160  1259  1019  807 1422  1428  1383
    avgSenValue[0] = (avgSenValue[0] - 826) * 1000 / 1392.6;
    avgSenValue[1] = (avgSenValue[1] - 641) * 1000 / 1160;
    avgSenValue[2] = (avgSenValue[2] - 756) * 1000 / 1259; 
    avgSenValue[3] = (avgSenValue[3] - 641) * 1000 / 1019;
    avgSenValue[4] = (avgSenValue[4] - 710) * 1000 / 807;
    avgSenValue[5] = (avgSenValue[5] - 664) * 1000 / 1422;
    avgSenValue[6] = (avgSenValue[6] - 710) * 1000 / 1428;
    avgSenValue[7] = (avgSenValue[7] - 779) * 1000 / 1383;
    //get fusion output, currently there are two weighting scheme, 8-4-2-1,15-14-12-8, can be adjusted. 
    fusionValue = (avgSenValue[0] * (-8) + avgSenValue[1]*(-4) + avgSenValue[2]*(-2) - avgSenValue[3] + avgSenValue[4] + avgSenValue[5]*2 + avgSenValue[6]*4 + avgSenValue[7]*8 )/4;
  }
  //After getting fusion outpus, feed this data to PID control to keep the car on the line
  //Remember to use changeBaseSpeed() function to avoid damaging the gears
  //how to determine how much the car is off the line given a fusion output
  //
  
}
