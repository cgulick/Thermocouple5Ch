
  // Import required libraries
#ifdef ESP32
  #include <WiFi.h>
  #include <ESPAsyncWebServer.h>
  #include <SPIFFS.h>
#else
  #include <Arduino.h>
  #include <ESP8266WiFi.h>
  #include <Hash.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <FS.h>
#endif

#include "PlayingWithFusion_MAX31856.h"
#include "PlayingWithFusion_MAX31856_STRUCT.h"
#include "SPI.h"

#define NumOfThemocouples 5
#define TRUE 1
#define FALSE 0
//seperate spi pins define
#define SpiClkPin  18
#define SpiMisoPin 19
#define SpiMosiPin 23

// Replace with your network credentials
const char* ssid = "TCDAS-2.4";
const char* password = "123456789";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

uint8_t TC0_CS  =  0;
uint8_t TC1_CS  =  1;
uint8_t TC2_CS  =  2;
uint8_t TC3_CS  =  3;
uint8_t TC4_CS  =  4;

PWF_MAX31856  thermocouple0(TC0_CS);
PWF_MAX31856  thermocouple1(TC1_CS);
PWF_MAX31856  thermocouple2(TC2_CS);
PWF_MAX31856  thermocouple3(TC3_CS);
PWF_MAX31856  thermocouple4(TC4_CS);

PWF_MAX31856 *PtrThermocouple[5]{&thermocouple0,&thermocouple1,&thermocouple2,&thermocouple3,&thermocouple4};

struct var_max31856 TC_CH0, TC_CH1, TC_CH2, TC_CH3, TC_CH4;
struct var_max31856 *PtrvarMax31856[5]={&TC_CH0, &TC_CH1, &TC_CH2, &TC_CH3, &TC_CH4};

// proto for display results function
void print31856Results(struct var_max31856 *tc_ptr);

const char* PARAM_INPUT_1 = "RGBLedHigh";
const char* PARAM_INPUT_2 = "RGBLedLow";

const char* PARAM_FLAG_1 = "flag1";
const char* PARAM_FLAG_2 = "flag2";
const char* PARAM_FLAG_3 = "flag3";
const char* PARAM_FLAG_4 = "flag4";
const char* PARAM_FLAG_5 = "flag5";

//gpio pins
const int ChrgSuspPin    =2;
const int ChrgHighPwrPin =4;
const int SpiCsEnpin     =5;
const int BlueLedPin     =12;
const int ChrgModePin    =13;
const int StatSel0Pin    =14;
const int StatSel1Pin    =15;
const int ChrgStatePin   =16;
const int LedSoutPin     =17;
const int SClkPin        =18;
const int StatSel2Pin    =21;
const int LedLatPin      =22;
const int SDatPin        =23;
const int SpiMuxAdd2pin  =25;
const int SpiMuxAdd1pin  =26;
const int SpiMuxAdd0pin  =27;
const int ChrgRegEn1Pin  =32;
const int GreenLedPin    =33;
const int DrdyStatPin    =34;
const int FaultStatPin   =35;
const int BatVoltagePin  =39;
const int BatChgIPin     =36;

//adc channels used
const int BatVoltageAdcCh    =3;
const int BatChrgIAdcCh      =0;

int ADC_VALUE = 0;
float voltage_value = 0; 
float BatteryVoltage = 0;
float ChargeCurrentValue = 0;
uint8_t BatteryIsCharging = false;
char ChargeOverRide = 0;
char DoneCharging = 0;


// setting PWM properties
const int freq = 1000;
const int ledChannel0 = 0;
const int ledChannel1 = 0;
const int resolution = 12;
uint16_t GrayScaleData[16];
uint8_t  DOTCorrectionData[17];
uint16_t FunctionControlData[8];
uint16_t StatusInformationDate[16];
uint8_t  LEDMode[8];
uint8_t  LEDDotMode[8];
uint16_t FillBits;

uint16_t SXmitData;

uint16_t LedTestData = 0;

//tell compiler not to optimize these volatile variables
volatile uint8_t SecondHandCount = 0;
volatile uint8_t MinuteHandCount = 0;
volatile unsigned long HourHandCount = 0;

volatile int TimerInterruptFlag = 0;
volatile long TimerIRQCounter = 0;
volatile char ErrorToggle = 0;


float tcurtemp = 0;
float tnewtemp = 75;
float tmax = 100*100;
float tmin = 57*100;

unsigned int tblue,tgreen,tred;

uint8_t humicdat;

uint8_t PrintSensorData = FALSE;
uint8_t PrintBatteryVData = FALSE;
uint8_t PrintBatteryIData = FALSE;
uint8_t PrintHumidityData = FALSE;
uint8_t PrintElapsedTimeData = FALSE;

float HIH7131Temp,HIH7131Humidity;
uint16_t HIH7131HReadValue;
uint16_t HIH7131TReadValue;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  //let the system stabilize for 1 second
  delay(1000);
  
  Serial.begin(115200);
  
  SetupGPIOPins();
  GpioToSpiMode1();
  
  TimerIrqSetup();
  InitMax31856();
  
  PrintSensorData = FALSE;
  PrintBatteryVData = FALSE;
  PrintBatteryIData = FALSE;
  PrintHumidityData = FALSE;
  PrintElapsedTimeData = FALSE;
  TimerIRQCounter = 0;
  MinuteHandCount = 0;
  SetupWebServer();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
  if(TimerInterruptFlag == true)
  {
    //get thermocouple data
    SampleThermocouples();
    
    //output to serial if enabled
    UpdateTCDisplay();
    
    //update thermocouple rgb leds
    UpdateTempLEDs();
    
    //get humidity data
    ProcessHumidity();
    
    //process battery
    ProcessBattery();

    if(WiFi.status()  == WL_CONNECTED)
    {
      digitalWrite(BlueLedPin, HIGH);
    }

    else
    {
      digitalWrite(BlueLedPin, LOW);
    }
    
    TimerInterruptFlag = 0;
  }

    //process second count
    ProcessTime();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessTime()
{
    static uint8_t curtimedif = 0;

    if(SecondHandCount > 59)
    {
      SecondHandCount = 0;
      MinuteHandCount++;
      
      if(MinuteHandCount > 59)
      {
        MinuteHandCount = 0;
        HourHandCount++;
      }
    }

    if(curtimedif != MinuteHandCount)
    {
      curtimedif = MinuteHandCount;
       if(PrintElapsedTimeData == true)
        { 
          Serial.println("");
          Serial.print("Elapsed Time = ");
          Serial.print(HourHandCount);
          Serial.print(":");
          Serial.print(MinuteHandCount);
          Serial.print(":");
          Serial.print(SecondHandCount);
          Serial.println("");
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessBattery()
{
     ChargeOverRide = 0;
    //check battery every 5 seconds
    if(TimerIRQCounter >= 5)
    {
      TimerIRQCounter = 0;
      TimerInterruptFlag = false;  
      
      //ChargerOn();
      
      MeasureBatteryCurrent();

      if(BatteryIsCharging == 1)
      {
        if(ChargeCurrentValue < .09)
        {
          DoneCharging = 1;
          ChargerOff();
        }
      }
      
      MeasureBattery();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CheckChargingStatus()
{
   char chargestate = 0;
   
  //read charger status pin
  chargestate = digitalRead(ChrgStatePin);

  //if charged state is high charging complete or not charging
  //no usb or wallwart connected
  if(chargestate == 1)
  {
    ChargerOff();
    ChargeCurrentValue = 0;
    ChargeOverRide = 0;
    MinuteHandCount = 0;
    BatteryIsCharging = 0;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MeasureBattery()
{
    ADC_VALUE = analogRead(BatVoltagePin);

    //update battery level led
    GrayScaleData[0] = ADC_VALUE;// << 4;
    
    BatteryVoltage = (ADC_VALUE * 3.115 ) / (4095);
    BatteryVoltage *= 1.454;

    if(BatteryVoltage <= 3.6)
    {
      ChargerOn();
      BatteryIsCharging = 1;
    }
    
    if(PrintBatteryVData == 1)
      {
      Serial.println("");
      Serial.print("ADC VALUE = ");
      Serial.print(ADC_VALUE);
      Serial.print(" Bat Voltage = ");
      Serial.print(BatteryVoltage);
      Serial.println(" Volts");
      }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MeasureBatteryCurrent()
{
    //measure charge current
    ADC_VALUE = analogRead(BatChgIPin);
    ADC_VALUE+=170;
    
    ChargeCurrentValue = (ADC_VALUE * 3.25 ) / (4095);
    ChargeCurrentValue/=2200;
    ChargeCurrentValue *=800;

    if(PrintBatteryIData == 1)
    {
    Serial.println("");
    Serial.print("ADC VALUE = ");
    Serial.print(ADC_VALUE);
    Serial.print(" Charge I = ");
    Serial.print(ChargeCurrentValue);
    Serial.print("Amps");
    //Serial.println("");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ChargerOn()
{
  pinMode(ChrgSuspPin, OUTPUT);
  digitalWrite(ChrgSuspPin, LOW);

  //set charge I to 100% charge current
  pinMode(ChrgHighPwrPin, OUTPUT);
  digitalWrite(ChrgHighPwrPin, HIGH);

  if(PrintBatteryVData == 1)
      {
        Serial.println("Charging On");
      }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ChargerOff()
{
  //turn off charger
   pinMode(ChrgSuspPin, OUTPUT);
   digitalWrite(ChrgSuspPin, HIGH);

   //set charge I to 10% charge current for safety
  pinMode(ChrgHighPwrPin, OUTPUT);
  digitalWrite(ChrgHighPwrPin, LOW);
   
   if(PrintBatteryVData == 1)
      {
        Serial.println("Charging Off");
      }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String  SendTempChartData()
{
  String tempstr = String(TC_CH0.FTemp);
  tempstr += ',';
  tempstr += TC_CH1.FTemp;
  tempstr += ',';
  tempstr += TC_CH2.FTemp;
  tempstr += ',';
  tempstr += TC_CH3.FTemp;
  tempstr += ',';
  tempstr += TC_CH4.FTemp;
  
  tempstr += ',';
  tempstr += BatteryVoltage;
  tempstr += ',';
  tempstr += ChargeCurrentValue*1000;
  tempstr += ',';
  tempstr += HIH7131Temp;
  tempstr += ',';
  tempstr += HIH7131Humidity;
  

  //Serial.println(tempstr);
  return(tempstr);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetupWebServer()
{
    // Initialize SPIFFS
  if(!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
////////////////////////////////////////////////////////
//non AP mode
////////////////////////////////////////////////////////
/*
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  */
////////////////////////////////////////////////////////
//AP mode
////////////////////////////////////////////////////////
// Connect to Wi-Fi network with SSID and password
  Serial.println("");
  Serial.print("Setting AP (Access Point)…");
  
  // Remove the password parameter, 
  //if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.println("");
  
  Serial.print("AP IP address: ");
  Serial.println(IP);
/////////////////////////////////////////////////////////
  
  digitalWrite(BlueLedPin, HIGH);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html");
    Serial.println("");
    Serial.println("sending index.html");
  });
  
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send_P(200, "text/plain", SendTempChartData().c_str());
  });

  // Route to load Chart.min.js file
  server.on("/Chart.min.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/Chart.min.js", "application/javascript");
    Serial.println("sending Chart.min.js ");
  });

  // Route to load gauge.min.js file
  server.on("/gauge.min.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/gauge.min.js", "application/javascript");
    Serial.println("sending gauge.min.js ");
  });

    // Route to load utils.js file
  server.on("/utils.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/utils.js", "application/javascript");
    Serial.println("sending utils.js ");
  });

  // 
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) 
  {
  String inputMessage1;
  String inputMessage2;
 
  if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) 
  {
    inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
    inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
    //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    tmax = (float)inputMessage1.toInt();
    tmin = (float)inputMessage2.toInt();

    //need to sclale up by 100
    tmax *= 100;
    tmin *= 100;
  }
  
  else 
  {
    inputMessage1 = "No message sent";
    inputMessage2 = "No message sent";
  }
  
  Serial.print("TempIndicatorHigh =");
  Serial.print(inputMessage1);
  Serial.print(" - TempIndicatorLow = ");
  Serial.println(inputMessage2);
  request->send(200, "text/plain", "OK");
  });

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
  server.on("/setflags", HTTP_GET, [] (AsyncWebServerRequest *request) 
  {
  String inputflags;
  uint8_t serialflags = 0;
  
  
  if (request->hasParam(PARAM_FLAG_1)) 
      
  {
    inputflags = request->getParam(PARAM_FLAG_1)->value();

    serialflags = (uint8_t)inputflags.toInt();
    
    PrintSensorData = serialflags & 1;
    PrintBatteryVData = (serialflags>>1) & 1;
    PrintBatteryIData = (serialflags>>2) & 1;
    PrintHumidityData = (serialflags>>3) & 1;
    PrintElapsedTimeData = (serialflags>>4) & 1;
 
 }

  /*
 else 
  {
    Serial.println("No message sent");
  }
  
  Serial.print("PrintSensorData =");
  Serial.println(PrintSensorData);

  Serial.print("PrintBatteryVData =");
  Serial.println(PrintBatteryVData);

  Serial.print("PrintBatteryIData =");
  Serial.println(PrintBatteryIData);

  Serial.print("PrintHumidityData =");
  Serial.println(PrintHumidityData);

  Serial.print("PrintElapsedTimeData =");
  Serial.println(PrintElapsedTimeData);
  */
  
  request->send(200, "text/plain", "OK");
  });

  // Start server
  server.begin();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessHumidity()
{
  uint8_t a,b;
  unsigned int dc;

  float tdat;

  GpioToSpiMode2();
  SetSpiCsValue(5);

  digitalWrite(SpiCsEnpin, LOW);
  humicdat= 0;

  SPI.transfer(humicdat);

  digitalWrite(SpiCsEnpin, HIGH);

  SetSpiCsValue(5);  

  HIH7131TReadValue = 0;
  HIH7131HReadValue = 0;

  digitalWrite(SpiCsEnpin, LOW);
  HIH7131HReadValue = SPI.transfer(humicdat) << 8;
  HIH7131HReadValue |= SPI.transfer(humicdat);

  a = SPI.transfer(humicdat) ;
  b = SPI.transfer(humicdat);
  HIH7131TReadValue = a;
  HIH7131TReadValue <<=6;
  b>>=2;
  b&=0x3f;
  HIH7131TReadValue|=b;

  digitalWrite(SpiCsEnpin, HIGH);

  //Serial.println(a);
  //Serial.println(b);

  HIH7131TReadValue&=0x3fff;
  HIH7131HReadValue&=0x3fff;

  HIH7131Humidity = (float)HIH7131HReadValue / (float)16382U;
  HIH7131Humidity *= 100;

  dc = (unsigned int)HIH7131Humidity;

  dc *= 40.95;

  dc = 4095 - dc;

  //ledcWrite(ledChannel1,(unsigned int)dc);

  HIH7131Temp = (((float)HIH7131TReadValue / 16382.0 * 165.0) - 40);
  //HIH7131Temp *= 125;
  
  //vTaskDelay(15);
  //printf("\r\nHumidty = %3.2f Temp = %3.2fC",HIH7131Humidity,HIH7131Temp);
 // printf(" C to F = %3.2fF",((HIH7131Temp*1.8)+32));
 
   if(PrintHumidityData == TRUE)
   {
   Serial.print("Hum = ");
   Serial.print(HIH7131Humidity);
   Serial.println();
   Serial.print("Temp = ");
   Serial.print(((HIH7131Temp*1.8)+32));
   Serial.println();
   }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetRGBValue()
{
  float ttempoffset;
  float tspan;
  float tspand3;
  float scale;
  unsigned long tcurclr;

  tcurtemp *=100;

  if(tcurtemp >= tmax)
    {
      tcurtemp = tmax;
    }

  if(tcurtemp < tmin)
  {
    tcurtemp = tmin;
  }

  //determine temp offset from relative 0
  ttempoffset = tcurtemp-tmin;
  
  //determine span from min to max
  tspan = tmax-tmin;
  
  //determine bgr span
  tspand3 = tspan/4;
  
  //determine which color byte is 0xff
  tcurclr = (tcurtemp-tmin) /((tmax-tmin)/4);
  
  //determine scale factor. rgb led driver is 16bit pwm
  scale=65535/tspand3;

  tblue  = 0;
  tgreen = 0;
  tred   = 0;


  if(tcurclr == 0)
  {
    tblue = 0xffff;
    tgreen += (ttempoffset*scale);
  }

  else if(tcurclr == 1)
  {
    tgreen = 0xffff;
    ttempoffset = (tcurtemp - tspand3- tmin) * scale;
    tblue = 0xffff - (ttempoffset);
  }

  else if(tcurclr == 2)
  {
    tgreen = 0xffff;
    ttempoffset =  (tcurtemp - (tspand3 *2) - tmin) * scale;
    tred += (ttempoffset);
  }

  else
  {
    tred = 0xffff;
    ttempoffset =  (tcurtemp - (tspand3 *3) - tmin) * scale;
    tgreen = 0xffff - (ttempoffset);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateTempLEDs()
{
//////////////////////////////////////////////////////
//TC0
//////////////////////////////////////////////////////
  tcurtemp = (TC_CH0.FTemp);
  SetRGBValue();
  
  //red
  GrayScaleData[13] = tred;
  //blue
  GrayScaleData[14] = tblue;
  //green
  GrayScaleData[15] =tgreen;

  TC_CH0.Error = FALSE;
  
  if(TC_CH0.status > 0)
   {
    TC_CH0.FTemp = 0;//tmax;
    TC_CH0.Error = TRUE;
    
    if(ErrorToggle == 0)
      {
        //Serial.println("error toggle");
        //red
        GrayScaleData[13] = 0; 
        //blue
        GrayScaleData[14] = 0; 
        //green
        GrayScaleData[15] = 0; 
      }
   }
   

////////////////////////////////////////////////////
//TC1
//////////////////////////////////////////////////////
  tcurtemp = (TC_CH1.FTemp);
  SetRGBValue();
  
   //red
  GrayScaleData[10] = tred; 
  //blue
  GrayScaleData[11] = tblue; 
  //green
  GrayScaleData[12] =tgreen; 

  TC_CH1.Error = FALSE;
  
  if(TC_CH1.status > 0)
   {
    TC_CH1.FTemp = 0;//tmax;
    TC_CH1.Error = TRUE;
    
    if(ErrorToggle == 0)
      {
        //Serial.println("error toggle");
        //red
        GrayScaleData[10] = 0; 
        //blue
        GrayScaleData[11] = 0; 
        //green
        GrayScaleData[12] = 0; 
      }
   }
  
////////////////////////////////////////////////////
//TC2
//////////////////////////////////////////////////////
  tcurtemp = (TC_CH2.FTemp);
  SetRGBValue();
  
   //red
  GrayScaleData[7] = tred; // << 8;
  //blue
  GrayScaleData[8] = tblue; // << 8;
  //green
  GrayScaleData[9] =tgreen; // << 8;

  TC_CH2.Error = FALSE;
  
  if(TC_CH2.status > 0)
   {
    TC_CH2.FTemp = 0;//tmax;
    TC_CH2.Error = TRUE;
    
    if(ErrorToggle == 0)
      {
        //Serial.println("error toggle");
        //red
        GrayScaleData[7] = 0; 
        //blue
        GrayScaleData[8] = 0; 
        //green
        GrayScaleData[9] = 0; 
      }
   }
////////////////////////////////////////////////////
//TC3
///////////////////////////////////////////////////
  tcurtemp = (TC_CH3.FTemp);
  SetRGBValue();
  
   //red
  GrayScaleData[1] = tred; 
  //blue
  GrayScaleData[2] = tblue;
  //green
  GrayScaleData[3] =tgreen;

  TC_CH3.Error = FALSE;
  
  if(TC_CH3.status > 0)
   {
    TC_CH3.FTemp = 0;//tmax;
    TC_CH3.Error = TRUE;
    
    if(ErrorToggle == 0)
      {
        //Serial.println("error toggle");
        //red
        GrayScaleData[1] = 0; 
        //blue
        GrayScaleData[2] = 0; 
        //green
        GrayScaleData[3] = 0; 
      }
   }
////////////////////////////////////////////////////
//TC4
////////////////////////////////////////////////////  
  tcurtemp = (TC_CH4.FTemp);
  SetRGBValue();
  
   //red
  GrayScaleData[4] = tred; 
  //blue
  GrayScaleData[5] = tblue; 
  //green
  GrayScaleData[6] =tgreen; 

  TC_CH4.Error = FALSE;
  
  if(TC_CH4.status > 0)
   {
    TC_CH4.FTemp = 0;//tmax;
    TC_CH4.Error = TRUE;
    
    if(ErrorToggle == 0)
      {
        //Serial.println("error toggle");
        //red
        GrayScaleData[4] = 0; 
        //blue
        GrayScaleData[5] = 0; 
        //green
        GrayScaleData[6] = 0; 
      }
   }

  SpiToGpio();
  SendLedData();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SampleThermocouples()
{
  double tmp;
  int a = 0;
  
  GpioToSpiMode1();

  for(a=0; a < NumOfThemocouples; a++)
  {
    // Read CH 0
    PtrThermocouple[a]->MAX31856_update(PtrvarMax31856[a]);        // Update MAX31856 channel 0
    tmp = (double)PtrvarMax31856[a]->lin_tc_temp * 0.0078125;      // convert fixed pt # to double
    
    PtrvarMax31856[a]->FTemp = ((tmp*1.8)+32);
    if(PtrvarMax31856[a]->status)
    {
      PtrvarMax31856[a]->FTemp = 68;
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitMax31856()
{
  // call config command... options can be seen in the PlayingWithFusion_MAX31856.h file
  int a = 0;
  char retrycount = 0;

  for(a=0; a<NumOfThemocouples; a++)
  {
    retrycount = 0;
    PtrThermocouple[a]->MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
    
    while(PtrvarMax31856[a]->status !=0 && retrycount < 10)
      {
        Serial.print("re-attempt config on TC");
        Serial.println(a);
        retrycount++;
        PtrThermocouple[a]->MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
      }
  }
  
 // thermocouple0.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  //thermocouple1.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  //thermocouple2.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
 // thermocouple3.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
 // thermocouple4.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);

  UpdateTCDisplay();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GpioToSpiMode1()
{
   // setup for the the SPI library:
  SPI.begin(SpiClkPin,SpiMisoPin,SpiMosiPin);                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV64);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE1);             // MAX31856 is a MODE3 device
 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GpioToSpiMode2()
{
   // setup for the the SPI library:
  SPI.begin(SpiClkPin,SpiMisoPin,SpiMosiPin);                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV64);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE2); 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpiToGpio()
{
  SPI.end();
  
  pinMode(SClkPin, OUTPUT);
  digitalWrite(SClkPin, LOW);
  
  pinMode(SDatPin, OUTPUT);
  digitalWrite(SDatPin, LOW);
  
  pinMode(LedLatPin, OUTPUT);
  digitalWrite(LedLatPin, LOW);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateTCDisplay()
{
  int a = 0;
  if(PrintSensorData != TRUE)
  {
    return;
  }
  
   for(a=0; a < NumOfThemocouples; a++)
  {
    Serial.print("TC_");
    Serial.print(a);
    Serial.print(": ");
    print31856Results(PtrvarMax31856[a]);
    Serial.println(" ");

    if(PtrvarMax31856[a]->status == 0xFF)
      {
        PtrThermocouple[a]->MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
        Serial.print("re-attempt config on TC");
        Serial.println(a);
      }
  }

  /*
  // ##### Print information to serial port ####
  Serial.print("TC_0: ");            // Print TC0 header
  print31856Results(&TC_CH0);
  Serial.println(" ");

  Serial.print("TC_1: ");            // Print TC1 header
  print31856Results(&TC_CH1);
  Serial.println(" ");

  Serial.print("TC_2: ");            // Print TC2 header
  print31856Results(&TC_CH2);
  Serial.println(" ");

  Serial.print("TC_3: ");            // Print TC3 header
  print31856Results(&TC_CH3);
  Serial.println(" ");

  Serial.print("TC_4: ");            // Print TC3 header
  print31856Results(&TC_CH4);
  Serial.println(" ");

  if(TC_CH0.status == 0xFF)
  {
    thermocouple0.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
   // Serial.println("re-attempt config on TC0");
  }
  if(TC_CH1.status == 0xFF)
  {
    thermocouple1.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
   // Serial.println("re-attempt config on TC1");
  }
  if(TC_CH2.status == 0xFF)
  {
    thermocouple2.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  // Serial.println("re-attempt config on TC2");
  }
  if(TC_CH3.status == 0xFF)
  {
    thermocouple3.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
   // Serial.println("re-attempt config on TC3");
  }

  if(TC_CH4.status == 0xFF)
  {
    thermocouple4.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_1SAMP, CMODE_AUTO);
   // Serial.println("re-attempt config on TC4");
  }
  */
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print31856Results(struct var_max31856 *tc_ptr)
{
  double tmp;
  
  if(tc_ptr->status)
  {
    tc_ptr->FTemp = 68;
    tc_ptr->Error = TRUE;
    
    // lots of faults possible at once, technically... handle all 8 of them
    // Faults detected can be masked, please refer to library file to enable faults you want represented
    Serial.println("fault(s) detected");
    Serial.print("Fault List: ");
    if(0x01 & tc_ptr->status){Serial.print("OPEN  ");}
    if(0x02 & tc_ptr->status){Serial.print("Overvolt/Undervolt  ");}
    if(0x04 & tc_ptr->status){Serial.print("TC Low  ");}
    if(0x08 & tc_ptr->status){Serial.print("TC High  ");}
    if(0x10 & tc_ptr->status){Serial.print("CJ Low  ");}
    if(0x20 & tc_ptr->status){Serial.print("CJ High  ");}
    if(0x40 & tc_ptr->status){Serial.print("TC Range  ");}
    if(0x80 & tc_ptr->status){Serial.print("CJ Range  ");}
    Serial.println(" ");
    
    // print internal temp anyway
    tmp = (double)tc_ptr->ref_jcn_temp * 0.015625;  // convert fixed pt # to double
    Serial.print("Tint = ");                      // print internal temp heading
    if((-100 > tmp) || (150 < tmp))
    {
      Serial.println("unknown fault");
    }
    else
    {
      Serial.println(tmp);
    }
  }
   
  else  // no fault, print temperature data
 {
 
    Serial.print("no faults detected");
    // MAX31856 Internal Temp
    tmp = (double)tc_ptr->ref_jcn_temp * 0.015625;  // convert fixed pt # to double
    Serial.print(" Tint = ");                      // print internal temp heading
    if((-100 > tmp) || (150 < tmp)){Serial.print("unknown fault");}
    else{Serial.print(((tmp*1.8)+32));}
    
    // MAX31856 External (thermocouple) Temp
    tmp = (double)tc_ptr->lin_tc_temp * 0.0078125;           // convert fixed pt # to double
    Serial.print(" TC Temp = ");                   // print TC temp heading
    Serial.print(((tmp*1.8)+32));
    tc_ptr->FTemp = ((tmp*1.8)+32);
    tc_ptr->Error = FALSE;
    //printf("%3.2f",tmp);
 }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetupGPIOPins()
{
  
  //turn charger off on reset or power up
  pinMode(ChrgSuspPin, OUTPUT);
  digitalWrite(ChrgSuspPin, HIGH);
  
  //set charge I to high
  pinMode(ChrgHighPwrPin, OUTPUT);
  digitalWrite(ChrgHighPwrPin, HIGH);
  
  //set spi cs high
  pinMode(SpiCsEnpin, OUTPUT);
  digitalWrite(SpiCsEnpin, HIGH);
  
  //turn blue led off
  pinMode(BlueLedPin, OUTPUT);
  digitalWrite(BlueLedPin, HIGH);
  
  pinMode(SClkPin, OUTPUT);
  digitalWrite(SClkPin, LOW);
  
  pinMode(SDatPin, OUTPUT);
  digitalWrite(SDatPin, LOW);
  
  pinMode(LedLatPin, OUTPUT);
  digitalWrite(LedLatPin, LOW);
  
  //turn regulator 1 on. 
  pinMode(ChrgRegEn1Pin, OUTPUT);
  digitalWrite(ChrgRegEn1Pin, HIGH);////off = low
  
  pinMode(SpiMuxAdd0pin, OUTPUT);
  digitalWrite(SpiMuxAdd0pin, LOW);
  
  pinMode(SpiMuxAdd1pin, OUTPUT);
  digitalWrite(SpiMuxAdd1pin, LOW);
   
  pinMode(SpiMuxAdd2pin, OUTPUT);
  digitalWrite(SpiMuxAdd2pin, LOW);
  
  pinMode(SpiCsEnpin, OUTPUT);
  digitalWrite(SpiCsEnpin, HIGH);
  //pinMode(ChrgModePin, OUTPUT);
  //digitalWrite(ChrgModePin, HIGH);
  
  pinMode(StatSel0Pin, OUTPUT);
  digitalWrite(StatSel0Pin, LOW);
  
  //pinMode(StatSel1Pin, OUTPUT);
  //digitalWrite(StatSel1Pin, LOW);
  
  pinMode(ChrgStatePin, INPUT_PULLUP);
  pinMode(LedSoutPin, INPUT);
  
  //pinMode(StatSel2Pin, OUTPUT);
  //digitalWrite(StatSel2Pin, LOW);
  
  pinMode(DrdyStatPin, INPUT);
  pinMode(FaultStatPin, INPUT);
  
  //pinMode(GreenLedPin, OUTPUT);
  //digitalWrite(GreenLedPin, HIGH);
  
  //pinMode(BatVoltagePin, INPUT);
  //pinMode(BatChgIPin, INPUT);

  // configure LED PWM functionalitites
  //ledcSetup(ledChannel0, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  //ledcAttachPin(GreenLedPin, ledChannel0);

  // configure LED PWM functionalitites
  //ledcSetup(ledChannel1, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  //ledcAttachPin(BlueLedPin, ledChannel1);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SendLedData()
{
  int a,b;
  SpiToGpio();
 ///////////////////////////////////////////////////////////// 
  //on first bit out, bit 257 address gray scale registers
  //make sure latch is low. might be redundant
  digitalWrite(SDatPin, LOW);
  digitalWrite(LedLatPin, LOW);
  digitalWrite(SClkPin, LOW);
  digitalWrite(SClkPin, HIGH);
  digitalWrite(SClkPin, LOW);

  //xmit grayscale data to driver
  for(a=15;a>=0;a--)
  {
    SXmitData = GrayScaleData[a];

    for( b=15; b>=0; b--)
    {
      digitalWrite(SDatPin, (SXmitData>>b)&1);
      //digitalWrite(SClkPin, LOW);
      digitalWrite(SClkPin, HIGH);
      digitalWrite(SClkPin, LOW);
    }
  }

  digitalWrite(LedLatPin, HIGH);
  //digitalWrite(SClkPin, LOW);
  //digitalWrite(SClkPin, HIGH);
  //digitalWrite(SClkPin, LOW);
  digitalWrite(LedLatPin, LOW);

  digitalWrite(SDatPin, LOW);
  //return;
  
/////////////////////////////////////////////////////////////
//now dot
  //0x0200 set msb bit 257 hi for control register data
  //fillbits = 0x0200;
  digitalWrite(SClkPin, LOW);
  digitalWrite(LedLatPin, LOW);

  for(a=0;a<1;a++)
  {
    FillBits = 0x200;

    for( b=9; b>=0; b--)
    {
      digitalWrite(SDatPin, (FillBits>>b)&1);
      //digitalWrite(SClkPin, LOW);
      digitalWrite(SClkPin, HIGH);
      digitalWrite(SClkPin, LOW);
    }
  }

  //turn auto repeat bit on
  FunctionControlData[6] = 0x0;
  FunctionControlData[7] = 0x2;
  
  for(a=7;a>=0;a--)
  {
    SXmitData = 2;//FunctionControlData[a];

    for( b=15; b>=0; b--)
    {
      digitalWrite(SDatPin, (SXmitData>>b)&1);
      //digitalWrite(SClkPin, LOW);
      digitalWrite(SClkPin, HIGH);
      digitalWrite(SClkPin, LOW);
    }
  }
 
  //xmit grayscale data to driver
 for( b=0; b<16; b++)
      {
        DOTCorrectionData[b]=0x3f;
      }
      
  for( b=2; b<16; b+=3)
    {
      DOTCorrectionData[b]=0x3f;
    }

  for(a=16;a>=0;a--)
  {
    
    SXmitData = DOTCorrectionData[a];

    for( b=6; b>=0; b--)
    {
      digitalWrite(SDatPin, (SXmitData>>b)&1);
      digitalWrite(SClkPin, HIGH);
      digitalWrite(SClkPin, LOW);
    }
  }

  //latch data in to register
  digitalWrite(LedLatPin, HIGH);
  digitalWrite(SClkPin, LOW);
  digitalWrite(LedLatPin, LOW);
}
