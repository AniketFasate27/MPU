/*

CORE0: MPU9250 data acquisition and serial write done sequentially.
CORE1: ESP Restart over HTTP.

Python File for sending requests: relay_post.py
Python Command: python relay_post.py

Device IP: 192.168.0.110 (Static IP)
Backup IP: 192.168.0.120 (Static IP)

HTTP Link: http://192.168.x.110:80/post

////////HTTP COMMANDS////////
RELAY_OPEN - Switch Open
RELAY_CLOSE - Switch Close
ESP_RESTART - Restart ESP

*/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_wdt.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
#include <MPU9250_WE.h>
#include <Preferences.h>
#include <SPI.h>
#include "WiFi.h"
#include "global_variable.h"
#include "time.h"
#include <ESP32Ping.h>
#include "OTA.h"


/////////////MPU Setup/////////////
//VSPI
#define MPU_CS_PIN        5

const int8_t SYNC_BYTE = 0xAA;
bool useSPI = true;    // SPI use flag
unsigned long lastMicros = 0, lastMicros2 = 0;
const int16_t MAX_SAMPLING_FREQ = 4000;
const int16_t NUM_OF_CALIBRATION_SAMPLES = 5000;
unsigned long MINIMUM_SAMPLING_DELAY_uSec = (unsigned long)(1 * 1000000 / MAX_SAMPLING_FREQ);
const int16_t gyr_factor = 100;
const int16_t acc_factor = 10000;
const int16_t mag_factor = 10000;
///////////////WiFi-Setup//////////////


String control_val = "";

///////////////CONSTRUCTORS//////////////
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, useSPI);    //VSPI

void calibrate_MPU(float acc_bias[], float gyro_bias[], float mag_bias[]);

void setup(){


  

}

void loop(){
    myMPU9250.init();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  delay(1000);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);
  calibrate_MPU(Acc_bias,Gyro_bias,mag_bias);

  for (int i=0; i < N; i++) {
    esp_task_wdt_init(10, false);
    //  
    if (micros() >= (lastMicros + MINIMUM_SAMPLING_DELAY_uSec)) {
	  lastMicros = micros();
	  xyzFloat gValue = myMPU9250.getGValues();
	  xyzFloat gyr = myMPU9250.getGyrValues();
    xyzFloat magValue = myMPU9250.getMagValues();

        // Ax = (gValue.x)*acc_factor;
        // Ay = (gValue.y)*acc_factor;
        // Az = (gValue.z)*acc_factor;
        // Gx = (gyr.x)*gyr_factor;
        // Gy = (gyr.y)*gyr_factor;
        // Gz = (gyr.z)*gyr_factor;
    

        Ax[i] = (gValue.x - Acc_bias[0])*acc_factor;
        Ay[i] = (gValue.y  - Acc_bias[1])*acc_factor;
        Az[i] = (gValue.z - Acc_bias[2])*acc_factor;
        Gx[i] = (gyr.x - Gyro_bias[0])*gyr_factor;
        Gy[i] = (gyr.y - Gyro_bias[1])*gyr_factor;
        Gz[i] = (gyr.z - Gyro_bias[2])*gyr_factor;
        Mx[i] = (magValue.x - mag_bias[0])*mag_factor;
        My[i] = (magValue.y - mag_bias[1])*mag_factor;
        Mz[i] = (magValue.z - mag_bias[2])*mag_factor;

	      // Serial.write(SYNC_BYTE); // Send the start/sync byte
        // Serial.write((uint8_t*)&(Ax), sizeof(Ax));
        // Serial.write((uint8_t*)&(Ay), sizeof(Ay));
        // Serial.write((uint8_t*)&(Az), sizeof(Az));
        // Serial.write((uint8_t*)&(Gx), sizeof(Gx));
        // Serial.write((uint8_t*)&(Gy), sizeof(Gy));
        // Serial.write((uint8_t*)&(Gz), sizeof(Gz));
        // Serial.write((uint8_t*)&(Gz), sizeof(Gz));
        // Serial.write((uint8_t*)&(Gz), sizeof(Gz));


        Serial.print(" Ax ="); Serial.print(Ax[i]);
        Serial.print(" Ay ="); Serial.print(Ay[i]);
        Serial.print(" Az ="); Serial.println(Az[i]);
        Serial.print(" Gx ="); Serial.print(Gx[i]);
        Serial.print(" Gy ="); Serial.print(Gy[i]);
        Serial.print(" Gz ="); Serial.println(Gz[i]);
        Serial.print(" Mx ="); Serial.print(Mx[i]);
        Serial.print(" My ="); Serial.print(My[i]);
        Serial.print(" Mz ="); Serial.println(Mz[i]);
        // Serial.printf("%f %d \n",Acc_bias[2], Az);

    }
}