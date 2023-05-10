#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ESP32Time.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <ESP32Ping.h>
#include "global_variables.h"
#include "time.h"
#include "OTA.h"

Preferences preferences;

const char* remote_host = "www.google.com"; //Ping website 

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable BROWNOUT detector
  Serial.begin(115200);
  xTaskCreatePinnedToCore(codeForDataGather, "data_gather",    10000,    NULL,    1,    &Task1,   0);
  xTaskCreatePinnedToCore(codeForDataSend, "data_send",    10000,    NULL,    1,    &Task2,   1);
  connect_to_wifi();//This functions tries to connect to either wifi ssids, if not connected previously
   
  preferences.begin("dandi_pref", false); // Create a “Storage space” in the flash memory called dandi_pref
  cps = preferences.getInt("cps", 20);
  Serial.print("cps-");
  Serial.println(cps);
  ota_flag   = preferences.getBool("ota_flag", 0);
  reset_flag = preferences.getBool("reset_flag", 0);
  ota_execute = preferences.getBool("ota_execute", 0);
  Serial.print("ota-flag");
  Serial.println(ota_flag);

  //Deducing MpAcID of the ESP32 and stores in variable name device_ide
  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  sprintf(id_, "%04X", (uint16_t)(chipid >> 32));
  sprintf(id_1, "%08X\n", (uint32_t)chipid);
  strcat(kid, id_);
  strcat(kid, id_1);
  sprintf(k_id, "%c%c%c%c%c%c%c%c%c%c%c%c", kid[10], kid[11], kid[8], kid[9], kid[6], kid[7], kid[4], kid[5], kid[2], kid[3], kid[0], kid[1]);//k_id is the ssid name of AP
  Serial.println(k_id);
  device_id = atoi(k_id);
  
  /*otaq_flag initially 0, it is set to 1 when (Latest_FW > FW_VERSION) is true. then execOTA() is executed.
    below code runs the php file to reset OTA position index to 0.
  */
  if (ota_flag) {
    ota_flag = 0;
    preferences.putBool("ota_flag", ota_flag);
    preferences.end();
    ota_index_reset();
  }

  /*reset_flag initially 0, it is set to 1 when device_reset() is executed.
    below code runs the php file to reset the reset position index to 0.
  */
  if (reset_flag) {
    reset_flag = 0;
    preferences.putBool("reset_flag", reset_flag);
    preferences.end();
    reset_index_reset();
  }
  Serialnumber();
}

void loop() {}

/*
   Code for data_gather acquires N data points from MPU6050,
   it acquires N data points in N_set_gather chunks of N_gather_size data points each.
*/
void codeForDataGather( void * parameter ) {
  vTaskSuspend(NULL);
  Serial.println("Data Gathering Func");
  Wire.begin(21, 22, 400000); // sda, scl, freq
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //The data gather task will now continunoulsy run in this infinite for loop.
  while(1) {
    for (int j = 0; j < N_set_gather; j++) { //512 data points is 1 set and there are 4 sets
      Serial.println("Data Gathering Started");
      for (int i = j * N_gather_size; i < (j + 1) * N_gather_size; i++) { 
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);       // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(true);
        Wire.requestFrom(MPU, 14, true); // request a total of  7 registers
        AcX[i] = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        AcY[i] = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ[i] = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Tmp    = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) // Tmpt = Tmp/340.00+36.53  (i.e. Tmp/340 +35+521/340)
        GyX[i] = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        GyY[i] = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        GyZ[i] = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        delay(1000 / cps);
      }
      Serial.println("Data Gathering completed");

      if (send_Flag) {
        send_Flag = false;
        vTaskResume(Task2);
      }
    }
  }
}

/*codeForDataSend sends N datapoints in N_set_send chunks each of N_gather_size datapoints.
*/
void codeForDataSend( void * parameter ) {
  vTaskSuspend(NULL);
  Serial.println("Sending Data");
  while(1) {
    for (int j = 0; j < N_set_gather; j++) {
      int start_index;
      start_index = j * N_send_size; //0 -511, 512- 1023 , 1024-1535
      SendData_rawdata(&AcX[start_index], &AcY[start_index], &AcZ[start_index], &GyX[start_index], &GyY[start_index], &GyZ[start_index], start_index ); //??

      send_Flag = true;
      vTaskSuspend(NULL);
    }
  }
}

/*
    Below Function accepts the starting address of array
   Tries to connect to wifi, if not already connected before sending these array datapoints
   Connects to server and port, and sends the data points
   Datapoints are sent in N_set_send cycles, each cycle sending N_gather_size datapoints
*/

void SendData_rawdata(int16_t *pAcX, int16_t *pAcY, int16_t *pAcZ, int16_t *pGyX, int16_t *pGyY, int16_t *pGyZ, int  start_index ) {
  String Body = "";
  Body = "CHIPID=" + String(k_id) + "&Header=" + "AcX_AcY_AcZ_GyX_GyY_GyZ" + "&Row=" + String(N_send_size) + "&Column=" + "6" + "&serialno="
         + String(Srn) + "&Input0=" + array_to_string(pAcX) + "&Input1=" + array_to_string(pAcY) + "&Input2=" + array_to_string(pAcZ)
         + "&Input3=" + array_to_string(pGyX) + "&Input4=" + array_to_string(pGyY) + "&Input5=" + array_to_string(pGyZ)
         + "&Freq=" + String(cps) + "&SSID=" + String(ssid_current) + "&FW_VER=" + String(FW_VERSION);    //correct for the current cps ??
  Srn = Srn + N_gather_size;
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
    client.println("POST /dandi/dandi_device_1/dandi_test_post_new.php HTTP/1.1");

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

    while (client.available() == 0){//Wait for client to respond
      wait_timer++;
      delay(100);
      if(wait_timer>2000)
      {
         ESP.restart();
      }
                                 
    };
    while (client.available()) {   //Check if server is sending any response after the request
      wait_timer=0;
      char c = client.read();                                     //Read the response
      if (c == '\n') {
          Serial.println(key_word);
        key_word = "";
      }
      else if(c == '^'){
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

/*
   Array to string function accepts the strting address of array
   Then sends it in chunks of N_set_send data points
*/
String array_to_string(int16_t *Ac) {
  String stringData;
  int  i_start = 0;
  stringData = String(Ac[i_start]); // this is to make sure that the stringData does not start with a "_" character
  for (int i = 1; i < N_send_size  ; i++) {
    stringData = stringData + "_" + String(Ac[i]);
  }

  return stringData;
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

    client3.print("GET /dandi/dandi_device_1/dandi_test_post_new.php?Fn="); client3.print(Fn);

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

/*
   The below function tries to connect to 2 SSIDs, if not connected previously
   It is once evoked in void setup(), and then called every time before sending
   datapoints to server. It will keep on searching for either SSIDs for 15 seconds of timeout.
   if no SSIDs avaible within 15 seconds then the device will restart.

*/
int connect_to_wifi() {
  int timeout = millis();
  Serial.print("inside wifi connect & wifi status = ");
  Serial.println(WiFi.status());
  while ((WiFi.status() != 3))
  {
    Serial.println(WiFi.status());
    WiFi.begin(ssid1, password1);
    delay(5000);
    Serial.println("after connection");
    Serial.println(WiFi.status());
    int returnbit = internet_check();
    Serial.println(returnbit);
    if (WiFi.status() == 3)
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
      if (WiFi.status() == 3 && returnbit == 1) {
        Serial.println("Connected to ssid2");
        strncpy(ssid_current, ssid2, 15);
        vTaskResume(Task1);
        return 2;
      }
    }
    if (WiFi.status() != 3) {
      if (wifi_counter == 10)
      {
        ESP.restart();
      }

      Serial.println("Couldnt Connect to any ssid");
      wifi_counter += 1;

    }
  }
  int returnbit = internet_check();
  if (returnbit == 1)
  {
    Serial.println("internet available");
    vTaskResume(Task1);
  }
  else
  {
    Serial.println("internet not available");
    //     ESP.restart();
    connect_to_wifi();

  }

}

/*
   The below function is evoked in void setup() to
   check if client reset was conducted and if yes, then it
   will reset the value of reset in device_x_data.txt from reset_index_reset.php
   x in device_x_data.txt can be 1 or 2.
*/
void reset_index_reset() {
  int conn5;
  Serial.println("inside php reset");
  Fn = "ALL_INDEX_RESET";
  conn5 = client5.connect(server, port);
  if (conn5 == 1) {

    client5.print("GET /dandi/dandi_device_1/dandi_test_post_new.php?Fn="); client5.print(Fn);

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
void Serialnumber() {
  int conn7;
  Fn = "SERIAL";
  Srn = 0;
  Serial.print("chip id -");
  Serial.println(k_id);
  conn7 = client7.connect(server, port);
  if (conn7 == 1) {

    client7.print("GET /dandi/dandi_device_1/dandi_test_post_new.php?Fn="); client7.print(Fn);

    client7.print("&CHIPIDsr="); client7.print(k_id);
    client7.println(" HTTP/1.1");
    client7.print("HOST: "); client7.println(server);
    client7.println("Cache-Control: no-cache");
    client7.println();
    while (client7.available() == 0);
    while (client7.available()) {
      char ch = client7.read();
      if (ch == '\n') {
        Serial.println(serialno);
        serialno = "";
      }
      else if(ch == '^'){
        final_srn = serialno;
      }
      else {
        serialno = serialno + char(ch);
//        Serial.println(serialno);
      }

    }
    Serial.print("serial val in string is :");
//    Serial.println(serialno);
    Serial.println(final_srn);
    Srn = final_srn.toInt() ;
//    Srn = Srn + 1; 
//      Srn = strtol(serialno.c_str(), NULL, 10);
//  Serial.println(theLong);
  
    Serial.print("serial number from the last line is -");
    Serial.println(Srn);
    vTaskResume(Task1);
    //Srn =0;


  }
  else {
    client7.stop();
  }
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
    client4.print("GET /dandi/dandi_device_1/dandi_test_post_new.php?Fn="); client4.print(Fn);

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
  for (addressi2c = 1; addressi2c < 127; addressi2c++ )      //new
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(addressi2c);
    errori2c = Wire.endTransmission();

    if (errori2c == 0)
    {
      Serial.print("I2C device found at address 0x");

      if (addressi2c < 16)
        Serial.print("0");
      Serial.print((addressi2c-1), HEX);
      Serial.println("  !");
      i2c = 1;


    }

  }

  if (i2c == true)
  {
    Serial.print("MPU connected properly at sda,scl");
    //sensor_flag = false;
    //digitalWrite(RED_LED, HIGH);
    i2c = 0;
    //      self_check(i2c);
  }

  else
  {
    Serial.print("sda/scl not connected properly");
    i2c = 1;
    

  }
  Serial.println("inside self check");
  self_check(i2c);

}

//????
void self_check(int result) {
  int conn6;
  Fn = "SELF_CHECK";
  Serial.println("inside self check ");
  conn6 = client6.connect(server, port);
  if (conn6 == 1) {
    client6.print("GET /dandi/dandi_device_1/dandi_test_post_new.php?Fn=");
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
      else if(ch == '^'){
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
