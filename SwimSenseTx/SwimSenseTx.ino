/* Heltec Automation send communication test example
 *
 * Function:
 * 1. Send data from a esp32 device over hardware 
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

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "MPU9250.h"
#include "HT_SSD1306Wire.h"

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); 

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
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
#define BUFFER_SIZE                                 40 // Define the payload size here

#define TX_DELAY                                    50
MPU9250 mpu;


// heartrate monitoring
int PulseSensorPurplePin = 2;
int LED_B = LED_BUILTIN;   //  The on-board Arduion LED_B
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 2100;       // Determine which Signal to "count as a beat", and which to ingore.
// ============

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
// static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void setup() {
  Serial.begin(115200);
  Wire1.begin(41, 42, 100000);
  VextON();

  display.init();

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  delay(100);

  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 0, "Scanning for IMU...");
  display.display();

  if (!mpu.setup(0x68, MPU9250Setting(), Wire1)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

 
  mpu.setAccBias(16.02, 25.09, 38.67);
  mpu.setGyroBias(0.84, -0.88, -0.19);
  mpu.setMagBias(-471.10, 486.84, 424.44);
  mpu.setMagScale(0.83, 1.04, 1.20);

  display.drawString(104, 0, "Done");
  display.drawString(0, 12, "Initializing Radio...");
  display.display();

  txNumber=0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
  display.drawString(104, 12, "Done");
  display.display();
  delay(200);
  display.clear();
}
unsigned long prev_ms = millis();
int STEP = 0;
const int s = 16;
const int x_bounds[] = {s, 63 - s};
const int y_bounds[] = {38 + s, 127 - s};
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
void loop()
{
	if(mpu.update() && lora_idle == true)
	{
    if (millis() > prev_ms + TX_DELAY) {
      prev_ms = millis();
      float vx = mpu.getLinearAccX();
      float vy = mpu.getLinearAccY();
      float vz = mpu.getLinearAccZ();
      float qw = mpu.getQuaternionW();
      float qx = mpu.getQuaternionX();
      float qy = mpu.getQuaternionY();
      float qz = mpu.getQuaternionZ();

      float temp = mpu.getTemperature();

      Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
      uint8_t downcodedSignal = (uint8_t)(Signal / (2 * Threshold / 127)); // convert to a scale of 0-127 (assuming max hr signal is 5000)

      // float temp = mpu.getTemperature();
      sprintf(txpacket,"%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%d",vx,vy,vz,qw,qx,qy,qz,downcodedSignal);  //start a package
    
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
      

      // display.drawString(display.getWidth()/2, display.getHeight()/2-10/2, String(pitch,2));
      // display.drawString(display.getWidth()/2, display.getHeight()-30, String(roll,2));
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
      lora_idle = false;
      display.clear();
      display.screenRotate(ANGLE_90_DEGREE);
      display.setFont(ArialMT_Plain_24);
      display.drawString(0,0,"TX");
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 26, "Temp: " + String(temp,2));
      // drawBall(STEP);
      // drawBall(STEP + 500);
      display.display();
      // STEP+=1;
      // STEP %= 5000;
    }
	}
  Radio.IrqProcess( );
}

void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}