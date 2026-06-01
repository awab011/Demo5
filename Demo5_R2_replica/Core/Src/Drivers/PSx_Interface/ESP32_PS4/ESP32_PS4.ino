
#include <esp_now.h>
#include <WiFi.h>
#include "PS4.h"
#include <Wire.h>
#include "config.h"

#define I2C_SDA 22
#define I2C_SCL 23
#define LED_PIN1 21

#define SLAVE //define ESP32 as SLAVE or MASTER here

//counter
uint8_t rec_count;
unsigned long lastTriggerTime = 0; // Initialize the last trigger time
boolean toggle = false;  // Initialize to false (0)


// PMK and LMK keys
static const char *PMK_KEY_STR = "pmk1234567890123";
static const char *LMK_KEY_STR = "lmk1234567890123";

ps4_msg_t *buf;
#define I2C_CLOCK 400000L // 25kHz

uint8_t receiverAddress[] = {0xA0, 0xB7, 0x65, 0x4B, 0x0D, 0xF4};

/******************************************************************************************************************
   shitty way to handle butons but it is what was used in arduino ps4 code so i wil just reuse to keep the format:
 ******************************************************************************************************************/
typedef union
{
  unsigned int picdata;
  struct
  {
    unsigned char buff1;
    unsigned char buff2;
  };
} bits;
bits bits1, bits2;
unsigned int rbcdata;
unsigned char rbcdata1;

#define UTMRBC_SHARE 0
#define UTMRBC_L3 1
#define UTMRBC_R3 2
#define UTMRBC_UP 4
#define UTMRBC_RIGHT 5
#define UTMRBC_DOWN 6
#define UTMRBC_LEFT 7
#define UTMRBC_L2 8
#define UTMRBC_R2 9
#define UTMRBC_L1 10
#define UTMRBC_R1 11
#define UTMRBC_TRIANGLE 12
#define UTMRBC_CIRCLE 13
#define UTMRBC_CROSS 14
#define UTMRBC_SQUARE 15
#define UTMRBC_OPTION 0
#define UTMRBC_TOUCH 1

/*****************************************************************************************************************/
uint8_t data_to_send[11] = {0x02, 0, 0, 125, 125, 125, 125, 0, 0, 0, 'd'}; // The data to send


esp_now_peer_info_t peerInfo;
bool checkDpad(ButtonEnum b)
{
  switch (b)
  {
    case UP:
      return buf->buttons_1.dpad == DPAD_LEFT_UP || buf->buttons_1.dpad == DPAD_UP || buf->buttons_1.dpad == DPAD_UP_RIGHT;
    case RIGHT:
      return buf->buttons_1.dpad == DPAD_UP_RIGHT || buf->buttons_1.dpad == DPAD_RIGHT || buf->buttons_1.dpad == DPAD_RIGHT_DOWN;
    case DOWN:
      return buf->buttons_1.dpad == DPAD_RIGHT_DOWN || buf->buttons_1.dpad == DPAD_DOWN || buf->buttons_1.dpad == DPAD_DOWN_LEFT;
    case LEFT:
      return buf->buttons_1.dpad == DPAD_DOWN_LEFT || buf->buttons_1.dpad == DPAD_LEFT || buf->buttons_1.dpad == DPAD_LEFT_UP;
    default:
      return false;
  }
}




void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  pinMode (LED_PIN1, OUTPUT);



  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }

  // Set PMK key
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // Register the receiver board as peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  // Set the receiver device LMK key
  for (uint8_t i = 0; i < 16; i++)
  {
    peerInfo.lmk[i] = LMK_KEY_STR[i];
  }

  peerInfo.encrypt = true;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);



#ifdef MASTER
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
#endif

#ifdef SLAVE
  Wire.onRequest(requestEvent);
  Wire.begin(0x44, I2C_SDA, I2C_SCL, I2C_CLOCK);
  Wire.slaveWrite(data_to_send, 11);
#endif
}

uint8_t switchoff = 0;

void loop() {
  
  unsigned long currentTime = millis(); // Get the current time in milliseconds

  
  if (currentTime - lastTriggerTime >= 200) {
    
  data_to_send[0] = 0x02;
  data_to_send[1] = 0;
  data_to_send[2] = 0;
  data_to_send[3] = 125;
  data_to_send[4] = 125;
  data_to_send[5] = 125;
  data_to_send[6] = 125;
  data_to_send[7] = 0;
  data_to_send[8] = 0;
  data_to_send[9] = 0;
  data_to_send[10] = 'd';
    digitalWrite(LED_PIN1, 0);

  }


#ifdef MASTER


  Wire.beginTransmission(0x08);
  Wire.write(data_to_send, 11);
//   log_print_buf(data_to_send, 11);
  Wire.endTransmission();
  
#endif

}

#ifdef SLAVE
void requestEvent()
{
  Wire.write(data_to_send, 11);
  
  //  log_print_buf(data_to_send, 11);
}
#endif



void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  
  toggle = !toggle;  // This will toggle between true and false
  digitalWrite(LED_PIN1, toggle);
  buf = (ps4_msg_t *)incomingData;

  rbcdata = 0;
  rbcdata |= ((buf->buttons_2.share << UTMRBC_SHARE) |
              (buf->buttons_2.L3 << UTMRBC_L3) |
              (buf->buttons_2.R3  << UTMRBC_R3) |
              (checkDpad(UP) << UTMRBC_UP) |
              (checkDpad(RIGHT) << UTMRBC_RIGHT) |
              (checkDpad(DOWN) << UTMRBC_DOWN) |
              (checkDpad(LEFT) << UTMRBC_LEFT) |
              (0 << UTMRBC_L2) |
              (0 << UTMRBC_R2) |
              (buf->buttons_2.L1 << UTMRBC_L1) |
              (buf->buttons_2.R1 << UTMRBC_R1) |
              (buf->buttons_1.triangle << UTMRBC_TRIANGLE) |
              (buf->buttons_1.circle << UTMRBC_CIRCLE) |
              (buf->buttons_1.cross << UTMRBC_CROSS) |
              (buf->buttons_1.square << UTMRBC_SQUARE));

  rbcdata1 = 0;
  rbcdata1 |= ((buf->buttons_2.option << UTMRBC_OPTION) |
               (buf->buttons_3.Tpad << UTMRBC_TOUCH));

  bits1.picdata = rbcdata;
  bits2.picdata = rbcdata1;

  data_to_send[0] = 0x02;
  data_to_send[1] = bits1.buff1;
  data_to_send[2] = bits1.buff2;
  data_to_send[3] = buf->left_joy_x;
  data_to_send[4] = buf->left_joy_y;
  data_to_send[5] = buf->right_joy_x;
  data_to_send[6] = buf->right_joy_y;
  data_to_send[7] = buf->left_trigger;
  data_to_send[8] = buf->right_trigger;
  data_to_send[9] = rbcdata1;
  data_to_send[10] = 'c';


  lastTriggerTime = millis(); // Update the last trigger time
}
