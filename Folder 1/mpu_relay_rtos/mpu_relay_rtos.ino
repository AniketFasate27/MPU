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


const char* remote_host = "www.google.com"; //Ping website

// #include "ESPAsyncWebServer.h"

/////////////MPU Setup/////////////
//VSPI
#define MPU_CS_PIN        5

Preferences preferences;

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

// AsyncWebServer server(80);

///////////////CONSTRUCTORS//////////////
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, useSPI);    //VSPI

///////////////FUNCTION-PROTO////////////
void calibrate_MPU(float acc_bias[], float gyro_bias[], float mag_bias[]);
// void connect_to_wifi();

/////////////FREE-RTOS Setup/////////////
// TaskHandle_t MPU_GET_h = NULL;

void MPU_GET( void * parameter ) {
  esp_task_wdt_delete(NULL);

  // vTaskSuspend(NULL);

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


        // Serial.print(" Ax ="); Serial.print(Ax[i]);
        // Serial.print(" Ay ="); Serial.print(Ay[i]);
        // Serial.print(" Az ="); Serial.println(Az[i]);
        // Serial.print(" Gx ="); Serial.print(Gx[i]);
        // Serial.print(" Gy ="); Serial.print(Gy[i]);
        // Serial.print(" Gz ="); Serial.println(Gz[i]);
        // Serial.print(" Mx ="); Serial.print(Mx[i]);
        // Serial.print(" My ="); Serial.print(My[i]);
        // Serial.print(" Mz ="); Serial.println(Mz[i]);
        // Serial.printf("%f %d \n",Acc_bias[2], Az);

    }
  }
  Serial.println("Data Gathering completed");
  // if (send_Flag) {
  //   send_Flag = false;
  codeForDataSend();
  delay(50000);
 
}



void setup() {
    // Serial.begin(1500000);
    Serial.begin(115200);

    preferences.begin("ACC_pref", false); // Create a “Storage space” in the flash memory called dandi_pref
    
    cps = preferences.getInt("cps", 1);
    Serial.print("cps-");
    Serial.println(cps);
    ota_flag   = preferences.getBool("ota_flag", 0);
    reset_flag = preferences.getBool("reset_flag", 0);
    ota_execute = preferences.getBool("ota_execute", 0);
    Serial.print("ota-flag");
    Serial.println(ota_flag);
    xTaskCreatePinnedToCore(MPU_GET, "MPU_GET",    10000,    NULL,    1,    &Task1,   1);

    connect_to_wifi();//This functions tries to connect to either wifi ssids, if not connected previously
     //Deducing MpAcID of the ESP32 and stores in variable name device_ide
   chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
   sprintf(id_, "%04X", (uint16_t)(chipid >> 32));
   sprintf(id_1, "%08X\n", (uint32_t)chipid);
   strcat(kid, id_);
   strcat(kid, id_1);
   sprintf(k_id, "%c%c%c%c%c%c%c%c%c%c%c%c", kid[10], kid[11], kid[8], kid[9], kid[6], kid[7], kid[4], kid[5], kid[2], kid[3], kid[0], kid[1]);//k_id is the ssid name of AP
   Serial.println(k_id);
   device_id = atoi(k_id);
    
    vTaskDelete(NULL);

}

void loop() {
  // delay(1000);
  // vTaskDelete(NULL);
  // connect_to_wifi();
}

/*codeForDataSend sends N datapoints in N_set_send chunks each of N_gather_size datapoints.
*/
void codeForDataSend() {
  // vTaskSuspend(NULL);
  Serial.println("Sending Data");
  // while (1) {
  //   for (int j = 0; j < N_set_gather; j++) {
  //     int start_index;
  //     start_index = j * N; //0 -511, 512- 1023 , 1024-1535
  //     SendData_rawdata(&Ax[start_index], &Ay[start_index], &Az[start_index], &Gx[start_index], &Gy[start_index], &Gz[start_index], start_index ); //??

  //     // send_Flag = true;
  //     // vTaskSuspend(NULL);
  //   }
  String Body = "";
  Body = "CHIPID=" + String(k_id) + "&Header=" + "AcX_AcY_AcZ_GyX_GyY_GyZ" + "&Row=" + String(N) + "&Column=" + "6" + "&serialno="
         + String(Srn) + "&Input0=" + array_to_string(Ax) + "&Input1=" + array_to_string(Ay) + "&Input2=" + array_to_string(Az)
         + "&Input3=" + array_to_string(Gx) + "&Input4=" + array_to_string(Gy) + "&Input5=" + array_to_string(Gz)
         + "&Freq=" + String(cps) + "&SSID=" + String(ssid_current) + "&FW_VER=" + String(FW_VERSION);    //correct for the current cps ??
  Srn = Srn + N;
  Serial.println(Body);
  int BodyLen = Body.length();
  Serial.println(BodyLen);

  connect_to_wifi();
  int  conn1;
  Serial.println(); Serial.print("For sending raw data, Connecting to "); Serial.print(server);
  conn1 = client.connect(server, port); //Connects to asimtewari.com server
  if (conn1 == 1)                       // After succesful connection sending data to client
  {
    Serial.println(); Serial.print("Sending Parameters...");

    //Request
    client.println("POST /mmmf/test_vib/digital_twin_test_post_new.php HTTP/1.1");

    //Headers

    client.print("Host: "); client.println(server);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: "); client.println(BodyLen);
    client.println("Connection: Close");
    client.println();
    client.println(Body);
    client.println();
    Body = "";
    Serial.print("Free Heap size : ");//Serial.println(j);
    Serial.println(ESP.getFreeHeap());

    while (client.available() == 0) { //Wait for client to respond
      wait_timer++;
      delay(100);
      if (wait_timer > 2000)
      {
        ESP.restart();
      }

    };
    while (client.available()) {   //Check if server is sending any response after the request
      wait_timer = 0;
      char c = client.read();                                     //Read the response
      if (c == '\n') {
        Serial.println(key_word);
        key_word = "";
      }
      else if (c == '^') {
        final_resp = key_word;
      }
      else {
        key_word = key_word + char(c);

      }
    }
  }
  else {
    client.stop();                                               //Stop the client id couldnt connect to server
  }
  /*
     there are 3 responses sent from dandi_test_new.php stored in the variable string "key_word"
     if "Success,OTA" then client will undergo OTA
     If "Sucess,Param" then client will be updated with new frequency
     if "Sucess,reset" then client will undergo a reboot
  */
  if (final_resp == "Success") {
    Serial.println("success");
  }

  else if (final_resp == "Success,OTA") {
    vTaskSuspend(Task1);
    Serial.println("Inside success of ota ");
    checkVersion();                           //updates Latest_FW
    if (Latest_FW > FW_VERSION) {             //FW_VERSION is current version of ESP, Latest_FW is version being pushed by server
      ota_flag = 1;
      ota_execute = 1;
      preferences.putBool("ota_flag", ota_flag);
      preferences.putBool("ota_execute", ota_execute);
      preferences.end();
      execOTA();
      Serial.println("Resetting index");
      //delay(1000);
      //ota_index_reset();
      ESP.restart();

    }
    else {
      Serial.println("Same version available");
      ota_execute = 0;
      ota_index_reset();
      Serial.println("We are inside ota reset ::5");
      delay(5000);
      ESP.restart();
    }
  }

  else if (final_resp == "Success,Param") {
    parameter_change();
  }

  else if (final_resp == "Success,reset") {
    device_reset();                       //soft reset
  }

  else if (final_resp == "Success,Self_Check") {
    Serial.println("Self check");
    vTaskSuspend(Task1);
    i2cscanner();
  }
  else if (final_resp == "Success,Restart") {
    Serial.println("restart");
    device_reset();
  }

  
}

// void SendData_rawdata(int16_t *pAcX, int16_t *pAcY, int16_t *pAcZ, int16_t *pGyX, int16_t *pGyY, int16_t *pGyZ, int  start_index ) {
//   String Body = "";
//   Body = "CHIPID=" + String(k_id) + "&Header=" + "AcX_AcY_AcZ_GyX_GyY_GyZ" + "&Row=" + String(N) + "&Column=" + "6" + "&serialno="
//          + String(Srn) + "&Input0=" + array_to_string(pAcX) + "&Input1=" + array_to_string(pAcY) + "&Input2=" + array_to_string(pAcZ)
//          + "&Input3=" + array_to_string(pGyX) + "&Input4=" + array_to_string(pGyY) + "&Input5=" + array_to_string(pGyZ)
//          + "&Freq=" + String(cps) + "&SSID=" + String(ssid_current) + "&FW_VER=" + String(FW_VERSION);    //correct for the current cps ??
//   Srn = Srn + N;
//   Serial.println(Body);
//   int BodyLen = Body.length();
//   Serial.println(BodyLen);

//   connect_to_wifi();
//   int  conn1;
//   Serial.println(); Serial.print("For sending raw data, Connecting to "); Serial.print(server);
//   conn1 = client.connect(server, port); //Connects to asimtewari.com server
//   if (conn1 == 1)                       // After succesful connection sending data to client
//   {
//     Serial.println(); Serial.print("Sending Parameters...");

//     //Request
//     client.println("POST /mmmf/test_vib/digital_twin_test_post_new.php HTTP/1.1");

//     //Headers

//     client.print("Host: "); client.println(server);
//     client.println("Content-Type: application/x-www-form-urlencoded");
//     client.print("Content-Length: "); client.println(BodyLen);
//     client.println("Connection: Close");
//     client.println();
//     client.println(Body);
//     client.println();
//     Body = "";
//     Serial.print("Free Heap size : ");//Serial.println(j);
//     Serial.println(ESP.getFreeHeap());

//     while (client.available() == 0) { //Wait for client to respond
//       wait_timer++;
//       delay(100);
//       if (wait_timer > 2000)
//       {
//         ESP.restart();
//       }

//     };
//     while (client.available()) {   //Check if server is sending any response after the request
//       wait_timer = 0;
//       char c = client.read();                                     //Read the response
//       if (c == '\n') {
//         Serial.println(key_word);
//         key_word = "";
//       }
//       else if (c == '^') {
//         final_resp = key_word;
//       }
//       else {
//         key_word = key_word + char(c);

//       }
//     }
//   }
//   else {
//     client.stop();                                               //Stop the client id couldnt connect to server
//   }
//   /*
//      there are 3 responses sent from dandi_test_new.php stored in the variable string "key_word"
//      if "Success,OTA" then client will undergo OTA
//      If "Sucess,Param" then client will be updated with new frequency
//      if "Sucess,reset" then client will undergo a reboot
//   */
//   if (final_resp == "Success") {
//     Serial.println("success");
//   }

//   else if (final_resp == "Success,OTA") {
//     vTaskSuspend(Task1);
//     Serial.println("Inside success of ota ");
//     checkVersion();                           //updates Latest_FW
//     if (Latest_FW > FW_VERSION) {             //FW_VERSION is current version of ESP, Latest_FW is version being pushed by server
//       ota_flag = 1;
//       ota_execute = 1;
//       preferences.putBool("ota_flag", ota_flag);
//       preferences.putBool("ota_execute", ota_execute);
//       preferences.end();
//       execOTA();
//       Serial.println("Resetting index");
//       //delay(1000);
//       //ota_index_reset();
//       ESP.restart();

//     }
//     else {
//       Serial.println("Same version available");
//       ota_execute = 0;
//       ota_index_reset();
//       Serial.println("We are inside ota reset ::5");
//       delay(5000);
//       ESP.restart();
//     }
//   }

//   else if (final_resp == "Success,Param") {
//     parameter_change();
//   }

//   else if (final_resp == "Success,reset") {
//     device_reset();                       //soft reset
//   }

//   else if (final_resp == "Success,Self_Check") {
//     Serial.println("Self check");
//     vTaskSuspend(Task1);
//     i2cscanner();
//   }
//   else if (final_resp == "Success,Restart") {
//     Serial.println("restart");
//     device_reset();
//   }

// }
/*

    Array to string function accepts the strting address of array
   Then sends it in chunks of N_set_send data points
*/
String array_to_string(int16_t *Ac) {
  String stringData;
  int  i_start = 0;
  stringData = String(Ac[i_start]); // this is to make sure that the stringData does not start with a "_" character
  for (int i = 1; i < N; i++) {
    stringData = stringData + "_" + String(Ac[i]);
   
  }
  // Serial.println("String data"); Serial.println(stringData);

  return stringData;
}

/*
   The below function is evoked in void setup() to
   check if ota was conducted and if yes, then it
   will reset the value of ota in drvice_x_data.txt from ota_index_reset.php
   x in device_x_data.txt can be 1 or 2.
*/
void ota_index_reset() {
  checkVersion();
  //  if (Latest_FW <= FW_VERSION) {
  Serial.println("We are inside ota reset");
  vTaskSuspend(Task1);
  //    vTaskSuspend(Task2);

  int conn3;
  Fn = "OTA_INDEX_RESET";
  conn3 = client4.connect(server, port);
  Serial.print("conn3-");
  Serial.println(conn3);
  if (conn3 == 1) {
    client4.print("GET /dandi/dandi_device_3/dandi_test_post_new.php?Fn="); client4.print(Fn);

    client4.print("&SSID="); client4.print(ssid_current);
    client4.print("&CHIPID="); client4.print(k_id);
    client4.print("&FW_VER="); client4.print(FW_VERSION);
    client4.print("&OTA_EXECUTE="); client4.print(ota_execute);
    client4.println(" HTTP/1.1");
    client4.print("HOST: "); client4.println(server);
    client4.println("Cache-Control: no-cache");

    client4.println();

  }
  else {
    client4.stop();
    //    }
  }
  //while (client4.available() == 0);
}

void i2cscanner() {
  Serial.println("check connection");
}


//????
void self_check(int result) {
  int conn6;
  Fn = "SELF_CHECK";
  Serial.println("inside self check ");
  conn6 = client6.connect(server, port);
  if (conn6 == 1) {
    client6.print("GET /dandi/dandi_device_3/dandi_test_post_new.php?Fn=");
    client6.print(Fn);

    client6.print("&SSID="); client6.print(ssid_current);
    client6.print("&CHIPID="); client6.print(k_id);
    client6.print("&RESULT="); client6.print(result);
    client6.println(" HTTP/1.1");
    client6.print("HOST: "); client6.println(server);
    client6.println("Cache-Control: no-cache");
    client6.println();
  }
  else {
    client6.stop();
  }
  while (client6.available() == 0);
  while (client6.available()) {
    char ch = client6.read();
    if (ch == '\n') {
      serialno = "";
    }
    else if (ch == '^') {
      final_srn = serialno;
    }
    else {
      serialno = serialno + char(ch);
      //Serial.println(serialno);
    }
  }

  Serial.println(final_srn);

  ESP.restart();
}

void calibrate_MPU(float acc_bias[], float gyro_bias[], float mag_bias[]) {  //pass array by ref
  
  const int MPU9250_MINIMUM_SAMPLING_DELAY_uSEC = 250;  //250 because max sampling rate of accelerometer is 4 khz
  
  for(int i=0; i<3; i++) {
    gyro_bias[i] = 0.0;
    acc_bias[i] = 0.0;
  }
  
  for(int i=0; i<NUM_OF_CALIBRATION_SAMPLES; i++){
    xyzFloat gValue = myMPU9250.getGValues();
    xyzFloat gyr = myMPU9250.getGyrValues();
    xyzFloat magValue = myMPU9250.getMagValues();
    
    acc_bias[0] = acc_bias[0] + (gValue.x);
    acc_bias[1] = acc_bias[1] + (gValue.y);
    acc_bias[2] = acc_bias[2] + (gValue.z);
    gyro_bias[0] = gyro_bias[0] + (gyr.x);
    gyro_bias[1] = gyro_bias[1] + (gyr.y);
    gyro_bias[2] = gyro_bias[2] + (gyr.z);
    mag_bias[0] = mag_bias[0] + (magValue.x);
    mag_bias[1] = mag_bias[1] + (magValue.y);
    mag_bias[2] = mag_bias[2] + (magValue.z);

	  delayMicroseconds(MPU9250_MINIMUM_SAMPLING_DELAY_uSEC); //delay because max sampling rate of accelerometer is 4 khz

  }
  
  for(int i=0; i<3; i++){
    gyro_bias[i] = gyro_bias[i]/NUM_OF_CALIBRATION_SAMPLES;
    acc_bias[i] = acc_bias[i]/NUM_OF_CALIBRATION_SAMPLES;
    mag_bias[i] = mag_bias[i]/NUM_OF_CALIBRATION_SAMPLES;
  }
  acc_bias[2] = acc_bias[2] - 1; //z axis -
  // Serial.printf("%f, %f, %f, %f, %f, %f \n", acc_bias[0], acc_bias[1], acc_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]); 
}

/*
   The below function is evoked when server sends
   a response to update a parameter, like freuqency
   of data acquisition
*/
void parameter_change() {
  int conn3;
  Fn = "PARAM_CHECK";
  conn3 = client3.connect(server, port);
  if (conn3 == 1) {

    client3.print("GET /dandi/dandi_device_3/dandi_test_post_new.php?Fn="); client3.print(Fn);

    client3.print("&SSID="); client3.print(ssid_current);
    client3.print("&CHIPID="); client3.print(k_id);
    client3.print("&Freq="); client3.print(cps);
    client3.println(" HTTP/1.1");
    client3.print("HOST: "); client3.println(server);
    client3.println("Cache-Control: no-cache");
    client3.println();
    while (client3.available() == 0);
    while (client3.available()) {
      char ch = client3.read();
      if (ch == '\n') {
        param_word = "";
      }
      if (ch == '^') {
        final_resp = param_word;
      }

      else {
        param_word = param_word + char(ch);
      }
    }
    cps = final_resp.toInt() ;
    Serial.println(cps);
    preferences.putInt("cps", cps);
    preferences.end();
  }
  else {
    client3.stop();
  }
  ESP.restart();
}


int internet_check()
{

  Serial.print("Pinging host ");
  Serial.println(remote_host);


  int ret = Ping.ping(remote_host);
  delay(2000);
  Serial.print("return bit-");
  Serial.println(ret);
  //return ret;

  if (ret == 1)
  {
    //int returnbit=1;
    Serial.println("connected with internet");
    //return returnbit;
  }
  else if (ret == 0) {
    //int returnbit=0;
    Serial.println("connected without internet");
    //return returnbit;
  }

  return ret;



}

/*The below function is evoked when server sends
   a response to reset the client ESP32
*/
void device_reset()
{
  reset_flag = 1;
  preferences.putBool("reset_flag", reset_flag);
  preferences.end();
  ESP.restart();
}


void reset_index_reset() {
  int conn5;
  Serial.println("inside php reset");
  Fn = "ALL_INDEX_RESET";
  conn5 = client5.connect(server, port);
  if (conn5 == 1) {

    client5.print("GET /dandi/dandi_device_3/dandi_test_post_new.php?Fn="); client5.print(Fn);

    client5.print("&CHIPID="); client5.print(k_id);
    client5.print("&Freq="); client5.print(cps);
    client5.println(" HTTP/1.1");
    client5.print("HOST: "); client5.println(server);
    client5.println("Cache-Control: no-cache");
    client5.println();
  }
  else {
    client5.stop();
  }
}

int connect_to_wifi() {
  int timeout = millis();
  Serial.print("inside wifi connect & wifi status = ");
  Serial.println(WiFi.status());
  int returnbit = internet_check();
  while ((WiFi.status() != 3) )
  {
    Serial.println(WiFi.status());
    WiFi.begin(ssid1, password1);
    delay(5000);
    Serial.println("after connection");
    Serial.println(WiFi.status());
    // int returnbit = internet_check();
    Serial.println(returnbit);
    if (WiFi.status() == 3 )
    {
      Serial.println("Connected to ssid1");
      strncpy(ssid_current, ssid1, 15);       //If connected to ssid1, copy ssid1 name into ssid_current
      vTaskResume(Task1);
      return 1;
    }

    else {
      WiFi.begin(ssid2, password2);
      delay(5000);
      int returnbit = internet_check();
      if (WiFi.status() == 3 &&  returnbit == 1) {
        Serial.println("Connected to ssid2");
        strncpy(ssid_current, ssid2, 15);
        vTaskResume(Task1);
        return 2;
      }
    }
    if (WiFi.status() != 3) {
      if (wifi_counter == 10)
      {
        Serial.print("ESP Getting Restart");
        delay(100);
        ESP.restart();
      }

      Serial.println("Couldnt Connect to any ssid");
      wifi_counter += 1;
      Serial.print("WiFi Counter"); Serial.println(wifi_counter);

    }
  }

  
  // int returnbit = internet_check();
  if (returnbit == 1)
  {
    Serial.println("internet available");
//    vTaskResume(Task1);
  }
  else
  {
    Serial.println("internet not available");
    //    ESP.restart();
    //    connect_to_wifi();
    if (wifi_counter == 10)
    {
      Serial.print("ESP Getting Restart");
      delay(100);
      ESP.restart();
    }
    Serial.println("Couldnt Connect to any ssid");
    wifi_counter += 1;
    Serial.print("WiFi Counter second "); Serial.println(wifi_counter);

  }
}