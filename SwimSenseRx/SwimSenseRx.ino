/* Heltec Automation Receive communication test example
 *
 * Function:
 * 1. Receive the same frequency band lora signal program
 *  
 * Description:
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "float16.h"
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); 


#define RF_FREQUENCY                                915000000 // Hz

#define LORA_BANDWIDTH                              1         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       8         // [SF7..SF12]
#define LORA_CODINGRATE                             2         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
// #define BUFFER_SIZE                                 50 // Define the payload size here

// char txpacket[BUFFER_SIZE];
// char rxpacket[BUFFER_SIZE];

struct Telemetry {
  float16 vx, vy, vz;    // 12 bytes
  float16 qw, qx, qy, qz;// 16 bytes
  uint16_t signal;      // 2 byte
} __attribute__((packed));

Telemetry rxpacket;

static RadioEvents_t RadioEvents;

int16_t txNumber;

int16_t rssi,rxSize;

bool lora_idle = true;

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void setup() {
    Serial.begin(115200);

    VextON();

    display.init();
    display.screenRotate(ANGLE_90_DEGREE);
    display.setFont(ArialMT_Plain_24);
    display.drawString(0,0,"RX");

    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    
    txNumber=0;
    rssi=0;
  
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    
}

unsigned long lastRx = millis();
bool disconnected = false;
void loop()
{
  if(lora_idle)
  {
    lora_idle = false;
    Radio.Rx(0);
  }else if(!disconnected && millis() > lastRx + 1000){
    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(0,0,"RX");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0,26,"No signal...");
    display.display();
    disconnected = true;
  }
  Radio.IrqProcess( );
}
int STEP = 0;
const int s = 10;
const int x_bounds[] = {s, 50 - s};
const int y_bounds[] = {40 + s, 127 - s};
const int w = x_bounds[1] - x_bounds[0];
const int h = y_bounds[1] - y_bounds[0];
const int speed = 2;
void drawBall(int i){
  int x;
  int y;
  int d = (i * speed);
  if ((d / w) % 2 == 0){
    x = d % w;
  } else {
    x = w - (d % w);
  }

  if ((d/h) % 2 == 0){
    y = d % h;
  } else{
    y = h - (d % h);
  }

  display.drawCircle(x + x_bounds[0], y + y_bounds[0], s);
}
void drawRSSI(uint8_t signal){
  // display.screenRotate(ANGLE_180_DEGREE);
  // display.drawProgressBar(0, 0, 128, 16, progress);
  display.drawRect(52, 0, 8, 128);
  display.fillRect(54, 128 - signal, 4, signal - 2);
}
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(&rxpacket, payload, size);
    // rxpacket[size]='\0';
    Radio.Sleep( );
    disconnected = false;
    char rssiStr[10];
    sprintf(rssiStr, "RSSI: %d", rssi);
    uint8_t rssiStrength = (1.0 - min(max(-1 * rssi - 14.0, 0.0) / 106.0, 1.0)) * 124;
    display.clear();
    drawRSSI(rssiStrength);
    display.setFont(ArialMT_Plain_24);
    display.drawString(0,0,"RX");
    display.setFont(ArialMT_Plain_10);
    // drawBall(STEP);
    // drawBall(STEP + 500);
    // drawBall(STEP + 1000);
    display.drawString(0, 26, rssiStr);
    display.display();
    unsigned long curr = millis();
    unsigned long dt = curr - lastRx;
    lastRx = curr;
    Serial.printf(
      "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d\n",
      rxpacket.vx.toFloat(), rxpacket.vy.toFloat(), rxpacket.vz.toFloat(),
      rxpacket.qw.toFloat(), rxpacket.qx.toFloat(), rxpacket.qy.toFloat(), rxpacket.qz.toFloat(),
      rxpacket.signal, rssi, dt, STEP
    );
    // Serial.printf("[%d]{%s},rssi:%d\n",rxSize,rxpacket,rssi);
    lora_idle = true;
    STEP++;
    STEP %= 32767;
}
