#include <ECE3.h>

uint16_t sensorValues[8];
int fusionValue;


void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  delay(2000);
}

void loop()
{
  // read raw sensor values
      ECE3_read_IR(sensorValues);
      // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
      // 2500 means minimum reflectance
      
      //      Serial.print(readCount);
      //      Serial.print('\t');
      //      Serial.print(bumpSw2Reading);
    //normalize the average values 
    //min values 826 641 756 641 710 664 710 779
    //max values 1392.6 1160  1259  1019  807 1422  1428  1383
    sensorValues[0] = (sensorValues[0] - 826) * 1000 / 1392.6;
    sensorValues[1] = (sensorValues[1] - 641) * 1000 / 1160;
    sensorValues[2] = (sensorValues[2] - 756) * 1000 / 1259; 
    sensorValues[3] = (sensorValues[3] - 641) * 1000 / 1019;
    sensorValues[4] = (sensorValues[4] - 710) * 1000 / 807;
    sensorValues[5] = (sensorValues[5] - 664) * 1000 / 1422;
    sensorValues[6] = (sensorValues[6] - 710) * 1000 / 1428;
    sensorValues[7] = (sensorValues[7] - 779) * 1000 / 1383;
    //get fusion output, currently there are two weighting scheme, 8-4-2-1,15-14-12-8, can be adjusted. 
    fusionValue = (avgSenValue[0] * (-8) + avgSenValue[1]*(-4) + avgSenValue[2]*(-2) - avgSenValue[3] + avgSenValue[4] + avgSenValue[5]*2 + avgSenValue[6]*4 + avgSenValue[7]*8 )/4;
  }
  //After getting fusion outpus, feed this data to PID control to keep the car on the line
  //Remember to use changeBaseSpeed() function to avoid damaging the gears
  //how to determine how much the car is off the line given a fusion output
  //
  
}
