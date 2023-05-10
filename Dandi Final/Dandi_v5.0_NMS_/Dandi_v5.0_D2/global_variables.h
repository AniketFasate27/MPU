
/*Preferences related variables
  to store parameters in E2PROM of ESP32
*/
bool send_Flag= true;
bool ota_flag = 0;
bool reset_flag = 0;
bool ota_execute = 0;

//checking sda/scl connections
byte errori2c, addressi2c;
int i2c = false;


//OTA related variables
const int FW_VERSION = 1;
char Response[500];
String FW_Version;
int FW_Version_Start = 0;
int FW_Version_End = 0;
char FW_Version_Num[4];
int Latest_FW = 0;
String Fn,final_srn;


uint32_t timer1,timer2,timer3,timer4;
//Data Points related variables
const int N = 2048;
const int N_set_gather = 4;
const int N_set_send = 1;
const int N_gather_size = N / N_set_gather; // 2048/4 = 512
const int N_send_size = N_gather_size / N_set_send; // 512/1 = 512
int gathering_status[N_set_gather] ;  //Value of 1 means data gather complete, 0 means data is being currently gathered
int sending_status[N_set_gather];  //Value of 0 means N_gather_size data send complete, 1 means N_gather_size data is being currently send
uint32_t Srn=0;

//int index_arr[6]; ??
//int index_start_arr[N_gather_size]; ??
//int index_stop_arr[data_gather_size];  ??

//WiFi Credentials related variables
const char* ssid2 = "NCAIR IOT";
const char* password2 = "Asim@123Tewari";
const char* ssid1 = "MIP_JIO2";
const char* password1 = "dandi@12345";
char ssid_current[15];
int wifi_counter=0;

//Server Address
char server[] = "www.nationalsaltmemorial.in";
int port = 80;
WiFiClient client;
WiFiClient client3;
WiFiClient client4;
WiFiClient client5;
WiFiClient client6;
WiFiClient client7;
WiFiClient client8;
WiFiClient client9;

//Mac id related variables
uint64_t chipid;
char id_[6];
char id_1[10];
char kid[12];
char k_id[12];
int device_id;

//MPU6050 related variables
const int MPU = 0x68; //I2C address of MPU6050
int16_t temperature;
int16_t AcX[N], AcY[N], AcZ[N], Tmp, GyX[N], GyY[N], GyZ[N];

int wait_timer;

int cps; //Freq of data gathering in counts per second
TaskHandle_t Task1, Task2;

//String variables to receive response
String key_word = "";
String final_resp = "";
String param_word = "";
String ota_word = "";
String serialno = "";

//NTP Time related variables
char my_local_time[11];
char  mytime_1[11];
char  mytime_2[11];
char  mytime_3[11];
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 0;
