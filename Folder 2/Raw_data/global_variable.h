
int cps = 20; //Freq of data gathering in counts per second
TaskHandle_t Task1, Task2;

const int N = 400;
const int N_set_gather = 1;
uint32_t Srn = 0;
int wait_timer;

int16_t Ax [N];
int16_t Ay [N];
int16_t Az [N];
int16_t Gx [N];
int16_t Gy [N];
int16_t Gz [N];
int16_t Mx [N];
int16_t My [N];
int16_t Mz [N];

float Acc_bias[3];
float Gyro_bias[3];
float Mag_bias[3];

// bool send_Flag= true;
// bool ota_flag = 0;
// bool reset_flag = 0;
// bool ota_execute = 0;

//WiFi Credentials related variables
const char* ssid2 = "iron_dome1";
const char* password2 = "123456789";
const char* ssid1 = "iron_dome";
const char* password1 = "123456789";
char ssid_current[15];
int wifi_counter = 0;


//Server Address
char server[] = "www.lab40.in";
int port = 80;
WiFiClient client;
WiFiClient client3;
WiFiClient client4;
WiFiClient client5;
WiFiClient client6;
WiFiClient client7;
WiFiClient client8;
// WiFiClient client9;

//String variables to receive response
String key_word = "";
String final_resp = "";
String param_word = "";
String ota_word = "";
String serialno = "";

//OTA related variables
const int FW_VERSION = 1;
char Response[500];
String FW_Version;
int FW_Version_Start = 0;
int FW_Version_End = 0;
char FW_Version_Num[4];
int Latest_FW = 0;
String Fn, final_srn;



//Mac id related variables
uint64_t chipid;
char id_[6];
char id_1[10];
char kid[12];
char k_id[12];
int device_id;
