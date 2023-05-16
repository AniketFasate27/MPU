/***************************************************************************
  Example sketch for the MPU9250_WE library

  This sketch shows how to get acceleration, gyroscocope, magnetometer and
  temperature data from the MPU9250 using SPI.

  For further information visit my blog:

  https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
  https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)

***************************************************************************/

#include <MPU9250_WE.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include "global_variable.h"

const char* remote_host = "www.google.com"; //Ping website

const int csPin = 5;  // Chip Select Pin
bool useSPI = true;    // SPI use flag

unsigned long lastMicros = 0;
const int16_t MAX_SAMPLING_FREQ = 100;
unsigned long MINIMUM_SAMPLING_DELAY_uSec = (unsigned long)(1 * 1000000 / MAX_SAMPLING_FREQ);
const int16_t NUM_OF_CALIBRATION_SAMPLES = 500;
// float Mag_bias[3];
const int16_t gyr_factor = 100;
const int16_t acc_factor = 10000;
const int16_t mag_factor = 10000;

float Bx, By, Bz = 0;
const int8_t SYNC_BYTE = 0xAA;

/* There is only one construictor for SPI: */
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, csPin, useSPI);

void calibrate_MPU(float acc_bias[], float gyro_bias[], float mag_bias[]);

void setup() {
  Serial.begin(115200);
  //Deducing MpAcID of the ESP32 and stores in variable name device_ide
  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  sprintf(id_, "%04X", (uint16_t)(chipid >> 32));
  sprintf(id_1, "%08X\n", (uint32_t)chipid);
  strcat(kid, id_);
  strcat(kid, id_1);
  sprintf(k_id, "%c%c%c%c%c%c%c%c%c%c%c%c", kid[10], kid[11], kid[8], kid[9], kid[6], kid[7], kid[4], kid[5], kid[2], kid[3], kid[0], kid[1]);//k_id is the ssid name of AP
  Serial.println(k_id);
  device_id = atoi(k_id);
  Serial.print("Chip Id"); Serial.println(k_id);


  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  }
  else {
    Serial.println("MPU9250 is connected");
  }
  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
    delay(100);
    Serial.println("ESP getting restart");
    ESP.restart();
  }
  else {
    Serial.println("Magnetometer is connected");
  }

  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  calibrate_MPU(Acc_bias, Gyro_bias, Mag_bias);
  delay(200);
}

void loop() {
  Serial.println("Data Gathering Started");
  for (int i = 0; i < N; i++)
  {
    if (micros() >= (lastMicros + MINIMUM_SAMPLING_DELAY_uSec)) {
      lastMicros = micros();

      xyzFloat gValue = myMPU9250.getGValues();
      xyzFloat gyr = myMPU9250.getGyrValues();
      xyzFloat magValue = myMPU9250.getMagValues();

      //      Bx = magValue.x - Mag_bias[0];
      //      By = magValue.y - Mag_bias[1];
      //      Bz = magValue.z - Mag_bias[2];
      //      Serial.print(Bx); Serial.print(By); Serial.println(Bz);

      Ax[i] = (gValue.x - Acc_bias[0]) * acc_factor;
      Ay[i] = (gValue.y  - Acc_bias[1]) * acc_factor;
      Az[i] = (gValue.z - Acc_bias[2]) * acc_factor;
      Gx[i] = (gyr.x - Gyro_bias[0]) * gyr_factor;
      Gy[i] = (gyr.y - Gyro_bias[1]) * gyr_factor;
      Gz[i] = (gyr.z - Gyro_bias[2]) * gyr_factor;
      Mx[i] = (magValue.x - Mag_bias[0]);
      My[i] = (magValue.y - Mag_bias[1]);
      Mz[i] = (magValue.z - Mag_bias[2]);
      //    Serial.print(" Ax ="); Serial.print(Ax[i]);
      //    Serial.print(" Ay ="); Serial.print(Ay[i]);
      //    Serial.print(" Az ="); Serial.println(Az[i]);
      //    Serial.print(" Gx ="); Serial.print(Gx[i]);
      //    Serial.print(" Gy ="); Serial.print(Gy[i]);
      //    Serial.print(" Gz ="); Serial.println(Gz[i]);
      //    Serial.print(" Mx ="); Serial.print(Mx[i]);
      //    Serial.print(" My ="); Serial.print(My[i]);
      //    Serial.print(" Mz ="); Serial.println(Mz[i]);
    }
    else {
      i--;
    }
  }

  //  delay(500);
  Serial.println("Data Gather completed");
  connect_to_wifi();
  //  SendData_rawdata();
  delay(2000);
}

void calibrate_MPU(float acc_bias[], float gyro_bias[], float mag_bias[]) { //pass array by ref

  const int MPU9250_MINIMUM_SAMPLING_DELAY_uSEC = 10000;  //250 because max sampling rate of accelerometer is 4 khz

  for (int i = 0; i < 3; i++) {
    gyro_bias[i] = 0.0;
    acc_bias[i] = 0.0;
    mag_bias[i] = 0.0;
  }

  for (int i = 0; i < NUM_OF_CALIBRATION_SAMPLES; i++) {

    xyzFloat magValue = myMPU9250.getMagValues();
    xyzFloat gValue = myMPU9250.getGValues();
    xyzFloat gyr = myMPU9250.getGyrValues();

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

  for (int i = 0; i < 3; i++) {
    gyro_bias[i] = gyro_bias[i] / NUM_OF_CALIBRATION_SAMPLES;
    acc_bias[i] = acc_bias[i] / NUM_OF_CALIBRATION_SAMPLES;
    mag_bias[i] = mag_bias[i] / NUM_OF_CALIBRATION_SAMPLES;
  }
  acc_bias[2] = acc_bias[2] - 1; //z axis -
  // Serial.printf("%f, %f, %f, %f, %f, %f \n", acc_bias[0], acc_bias[1], acc_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}


int connect_to_wifi() {
  int timeout = millis();
  Serial.print("Inside wifi connect & wifi status = ");
  Serial.println(WiFi.status());
  while (WiFi.status() != 3)
  {
    Serial.println(WiFi.status());
    WiFi.begin(ssid1, password1);
    delay(5000);
    Serial.println("after connection");
    Serial.println(WiFi.status());
    int returnbit = internet_check();
    Serial.print("Return bitdata = "); Serial.println(returnbit);
    if (WiFi.status() == 3 && returnbit == 1)
    {
      Serial.println("Connected to SSID 1 ");
      strncpy(ssid_current, ssid1, 15);       //If connected to ssid1, copy ssid1 name into ssid_current
      //      vTaskResume(Task1);
      Serial.println("Ready to send data");
      SendData_rawdata();
      return 1;
    }

    else {
      WiFi.begin(ssid2, password2);
      delay(5000);
      int returnbit = internet_check();
      Serial.print("Return bitdata = "); Serial.println(returnbit);
      if (WiFi.status() == 3 && returnbit == 1) {
        Serial.println("Connected to ssid2");
        strncpy(ssid_current, ssid2, 15);
        //        vTaskResume(Task1);
        Serial.println("Ready to send data");
        SendData_rawdata();
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
  int returnbit = internet_check();
  Serial.print("Return bitdata = "); Serial.println(returnbit);
  if (returnbit == 1)
  {
    Serial.println("internet available");
    //    vTaskResume(Task1);
    Serial.println("Ready to send data");
    SendData_rawdata();
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

int internet_check()
{
  Serial.print("Pinging host ");
  Serial.println(remote_host);

  int ret = Ping.ping(remote_host);
  delay(2000);
  Serial.print("return bit = ");
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
void SendData_rawdata() {
  String Body = "";
  Body = "CHIPID=" + String(k_id) + "&Header=" + "AcX_AcY_AcZ_GyX_GyY_GyZ_Mx_Mz" + "&Row=" + String(N) + "&Column=" + "9" + "&serialno="
         + String(Srn) + "&Input0=" + array_to_string(Ax) + "&Input1=" + array_to_string(Ay) + "&Input2=" + array_to_string(Az)
         + "&Input3=" + array_to_string(Gx) + "&Input4=" + array_to_string(Gy) + "&Input5=" + array_to_string(Gz)
         + "&Input6=" + array_to_string(Mx) + "&Input7=" + array_to_string(My) + "&Input8=" + array_to_string(Mz)
         + "&Freq=" + String(cps) + "&SSID=" + String(ssid_current) + "&FW_VER=" + String(FW_VERSION);    //correct for the current cps ??

  Srn = Srn + N;

  //  String Body = "&ChipId=" + String(k_id) + "&SentBy=" + "Aniket";
  Serial.print("printing body");
  Serial.println(Body);
  int BodyLen = Body.length();
  Serial.println(BodyLen);

  //  connect_to_wifi();
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
    Body = "";
    while (client.available() == 0);

    //Print Server Response
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
  } else {
    client.stop();
    Serial.println("Connection Failed");
  }
  delay(5000);
}

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
  //  Serial.println(stringData);

  return stringData;
}
