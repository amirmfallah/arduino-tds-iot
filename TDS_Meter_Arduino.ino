#define TINY_GSM_MODEM_SIM800

#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

#define TFT_CS        10
#define TFT_RST        8
#define TFT_DC         9

#define UP_BUTTON 7
#define DOWN_BUTTON 6
#define RELAY 5

#define sensorPin A5

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);



SoftwareSerial SerialAT(3, 2); // RX, TX

//Network details
const char apn[]  = "mtnirancell";
const char user[]  = "";
const char pass[]  = "";


// MQTT details
unsigned long lastReconnectAttempt = 0, lastSent = 0;

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);



#define TdsSensorPin A1
#define VREF 4.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 15;

int tdsOffset = 200;
int buttonState = 0;

void setup()
{

  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(RELAY, OUTPUT);

  
  //Serial.begin(9600);
  SerialAT.begin(9600);

  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("Connecting", ST77XX_RED, 0, 0, 2);
  //testdrawtext("Wait for network", ST77XX_YELLOW, 0, 70, 1);

//  modem.restart();
//  if(!modem.waitForNetwork())
//  {
//    while(true);
//  }

  if (!modem.gprsConnect(apn, user, pass))
  {
    while(true);
  }
  
  mqtt.setServer("185.220.224.46", 1883);
  while(mqttConnect()==false) continue;
  //tft.fillScreen(ST77XX_BLACK);
}

void loop()
{


  if (!modem.isGprsConnected()) {
      if (!modem.gprsConnect(apn, user, pass)) {
        delay(10000);
        return;
      }
  }

  if (!mqtt.connected()) {
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }
  
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT) 
       analogBufferIndex = 0;

    char buf[7];
    dtostrf(tdsValue, 7, 2, buf);
    testdrawtext("TDS Value:", ST77XX_YELLOW,0,0, 2);
    testdrawtext(buf, ST77XX_WHITE,0,20, 3);
    testdrawtext("ppm", ST77XX_BLUE,90,40, 2);

    testdrawtext("Temp", ST77XX_BLUE,0,70, 2);
    dtostrf(temperature, 6, 2, buf);
    testdrawtext(buf, ST77XX_RED,50,70, 2); 

    
    testdrawtext("TDS offset", ST77XX_BLUE,0,100, 2);
    dtostrf(tdsOffset, 8, 2, buf);
    testdrawtext(buf, ST77XX_RED,0,120, 2); 
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      temperature = (analogRead(sensorPin) * (4300 / 1024.0)) / 10;
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
   }

  buttonState = digitalRead(UP_BUTTON);
  if (buttonState == HIGH) {
    tdsOffset++;
  }

  buttonState = digitalRead(DOWN_BUTTON);
  if (buttonState == HIGH) {
    tdsOffset--;
  }

  if(tdsValue >= float(tdsOffset)){
    digitalWrite(RELAY, HIGH);
  }
  else {
    digitalWrite(RELAY, LOW);
  }


  uint32_t t2 = millis();
  if (t2 - lastSent > 10000L) {
    lastSent = t2;
    String message = "{\"uuid\":\"d465fc786a24\", \"tds\":" + String(tdsValue) + ", \"temp\":" + String(temperature) + "}";
    mqtt.publish("/tds", message.c_str());
    mqtt.loop();
  }
  
}

boolean mqttConnect()
{
  mqtt.connect("");
  return mqtt.connected();
}

void testdrawtext(char *text, uint16_t color, int x, int y, int size) {
  tft.setCursor(x, y);
  tft.setTextSize(size);
  tft.setTextColor(color, ST7735_BLACK);
  tft.setTextWrap(true);
  tft.print(text);
}


int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
