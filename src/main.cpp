/*
########################################################################################################
      This has been tested 30th Dec 2019 on M5Stack Core (black casing) 
      It is a standard M5 Stack unit
      Board Type = "M5Stack-Core-ESP32"
      TOF on I2C connectors
       on pin 16
########################################################################################################
    v0.1 m5stack basic cat door 
    v0.2 stable - tested
    v0.3 Added Alert Image when open
    v0.4 Add MQTT Support
    v0.5 Add NeoPixel Support
    v0.6 Add MQTT status messages
    v0.7 Add Cat Image when closed, Add status alive repeat (in mins)
    v1.0 Release    
    v1.1 Add WiFi reconnect on disconnect, 10 tries on startup, loops forever for MQTT msgs
    v1.2 Add PrivateConfig.h file to seperate secret info (WiFi network info etc), Add no wifi icon and no MQTT icon statuses
    v1.3 Changed screen notices sequence to cat face change before network messages
    v2.0 Released
    v2.1 Extra diag info on MQTT Diag Channel
    v2.2 Extra info on Door msg
    v2.3 Change alert distance and added door reset alert
    v3.0 Released - stable
    v3.1 Add auto close using servo on pin 16
    v3.2 Bug fix to allow startup with out comms or MQTT and blue screen when door reset after cat entry
    v3.3 Improve WiFi and MQTT retry process
    v4.0R - Released version 
            
*/

#include <WiFiMulti.h>                //Support for defining multiple WiFi points (used at startup)
#include <M5Stack.h>                  //M5Stack support
boolean M5stack = true;               //Enable M5 output text
#include <VL53L0X.h>                  //Time of flight distance sensor
#include <Wire.h>                     //i2C comms needed for VL sensor
#include <TimeLib.h>                  //Time functions
#include <ArduinoOTA.h>               //Over The Air updates
#include <Math.h>                     //needed for OTA % calculation
#include "XBMImages.h"                // needed for AlertImage xbm image
#include "PrivateConfig.h"            // needed for all network and security keys

const char* BuildTime = __TIME__;
const char* BuildDate = __DATE__;
const char* Version = "V4.0R";
char Build[30];
//String DoorStatus = "XXXXXX";
char* DoorStatus = "XXXXXX";
String PrevDoorStatus = "YYYYYY";
boolean DoorTriggerOpen;
int StdScreenBrightness = 100;
boolean TickTock = false;
int BackGroundColor = BLUE;
char OutputText[100];                 //Used to build output messages

#define HOSTNAME "M5CATDOOR"
#define SKETCH "Cat Door Monitor"
#define STARTUPMSGDELAY 1500           // ms to wait to display startup messages
#define SHORTVERSION "YES"             // output just the version no on screen at startup
#define STARTUPSCREENBRIGHTNESS 200    // startup brightness
#define MAINLOOPDELAY 750              // number if milliseconds to delay main loop cycle
#define OTAPort 3232

int AlertDist = 44;           // Set trigger open distance
boolean DebugDisplay = false;  //Enable M5 debug messages on LCD


String WiFiIP;
String WiFiSID;
WiFiMulti wifiMulti;


/** START - NeoPixel **/
#define NUMPIXELS 12
#define PIXELPIN 15
int NeoFadeSpeed = 10;             // speed in milliseconds that neos will fade at
int NeoBrightness = 250;
#include <NeoPixelBus.h>  //Neopixels
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixels(NUMPIXELS, PIXELPIN);
/** END - NeoPixel **/

/** START - Time Of Flight VL53L0X parameters **/
VL53L0X VL53L0Xsensor;
#define HIGH_ACCURACY
boolean TOFSensor = true;
int SensorDist = 70;
int VL53L0XAddress = 0x29;
/** END - Time Of Flight VL53L0X parameters **/

/** START - loop counter to measure i2c speed **/
int LoopCounter = 1;
int PreviousCounter = 0;
time_t LoopSecond;
time_t LoopDuration = 20;
/** END - loop counter **/

/** START - MQTT parameters **/
#include <PubSubClient.h> //MQTT support
WiFiClient espClient;
PubSubClient MQTTclient(espClient);
long lastMsg = 0;
char msg[50];
int MQTTRetryCount = 0;
int MQTTMAXRetryCount = 2;  // Sets no of times to try to connect for each message
boolean MQTTAvailable = true;  // used to track if MQTT connection ok
/** END - MQTT parameters **/


/** START - Status parameters **/
int CurrentMin;
#define MinToWaitFor 1
/** END - Status parameters **/

/** START - Servo parameters **/
#include <Servo.h>
static const int ServoPin = 16;
int ServoOpen = 20;
int ServoClosed = 180;
int ServoMid = 90;
Servo servo1;
time_t DoorTriggerTime = now() + 28000000;   //Time that door is triggered
int DoorTriggerWaitTime = 5;             // Seconds to wait until trigger servo to close door
/** END - Servo parameters **/

/*
 * ###################################################
 * #      START ROUTINES                             ############################################
 * ###################################################
*/

void DebugSensorDisplay()
{
  if (M5stack == true && DebugDisplay == true)
  {
    M5.Lcd.setCursor(120,100);
    M5.Lcd.setTextSize(6);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.print(SensorDist);
  }
}


void M5OutputBuildInfo()
{
  M5.Lcd.setBrightness(StdScreenBrightness);
  M5.Lcd.fillScreen(BackGroundColor);
  if (SHORTVERSION == "Yes" )
  {
    M5.Lcd.setCursor(80,90);
    M5.Lcd.setTextSize(10);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println(Version);
    delay(STARTUPMSGDELAY);
  }
  else
  {
    sprintf(Build, "%s %s", BuildTime, BuildDate );
    M5.Lcd.setCursor(0,20);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println(Version);
    M5.Lcd.println(SKETCH);
    M5.Lcd.println(BuildTime);
    M5.Lcd.println(BuildDate);
    M5.Lcd.println(HOSTNAME);
    delay(STARTUPMSGDELAY);
  }
}

/** START - NeoPixel Code **/
void SetupNeoPixels()
{
  delay(50);
  pixels.Begin();
  delay(50);
  pixels.Show(); // Initialize all pixels to 'off'

}


void NeoPixFadeUp()
{
  uint16_t Pixel, PixValue;

  for (PixValue = 1; PixValue < 100; PixValue++)
  {
    for (Pixel = 0; Pixel < NUMPIXELS; Pixel++)
    {
      pixels.SetPixelColor(Pixel, RgbColor(PixValue, 0, 0));
    }
    pixels.Show();
    delay(NeoFadeSpeed);
  }
}
// pulse Neostrip
void NeoAllPulse( int NeoRED, int NeoGREEN, int NeoBLUE, int NeoDELAY)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.SetPixelColor(i, RgbColor(NeoRED, NeoGREEN, NeoBLUE));
  }
  pixels.Show();
  delay(NeoDELAY);
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.SetPixelColor(i, RgbColor(0, 0, 0));
  }
  pixels.Show();
}

void NeoPixRunning()
{
  uint16_t Pixel, PixValue;

  for (PixValue = 1; PixValue < 100; PixValue++)
  {
    for (Pixel = 0; Pixel < NUMPIXELS; Pixel++)
    {
      pixels.SetPixelColor(Pixel, RgbColor(PixValue, 0, 0));
    }
    pixels.Show();
    delay(NeoFadeSpeed);
  }
}

void NeoPixFadeDown()
{
  uint16_t Pixel, PixValue;

  for (PixValue = 100; PixValue > 1; PixValue--)
  {
    for (Pixel = 0; Pixel < NUMPIXELS; Pixel++)
    {
      pixels.SetPixelColor(Pixel, RgbColor(PixValue, 0, 0));
    }
    pixels.Show();
    delay(NeoFadeSpeed);
  }
}
/** END - NeoPixel Code **/


void MQTTcallback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Responder to Status request
  if ((char)payload[0] == 'STATUS')
  {
    Serial.println("Output M5 Status");
    MQTTclient.publish(MQTTOutputTopic, "M5 Stack ALIVE");
    MQTTclient.loop();
  }
}

void MQTTconnect()
{
  
  while (!MQTTclient.connected() and MQTTAvailable)
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "M5StackClient-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (MQTTclient.connect(clientId.c_str()))
      {
        Serial.println("MQTT Connected");
  
        M5.Lcd.setCursor(300,220);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(" ");
  
        // Once connected, publish an announcement...
        MQTTclient.publish(MQTTOutputDiag, "M5 Stack Reconnected ");
        MQTTclient.loop();
        // ... and resubscribe
        MQTTclient.subscribe(MQTTInputTopic);
        MQTTclient.loop();

        MQTTclient.setCallback(MQTTcallback);
        Serial.println("MQTT Callback set");
        M5.Lcd.drawXBitmap(280,195, MQTTErr, MQTTErrWidth, MQTTErrHeight, TFT_BLACK);

      }
      else if (MQTTRetryCount < MQTTMAXRetryCount)
      {
        M5.Lcd.drawXBitmap(280,195, MQTTErr, MQTTErrWidth, MQTTErrHeight, TFT_RED);
        Serial.print("failed, rc=");
        Serial.print(MQTTclient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
        MQTTRetryCount = MQTTRetryCount + 1;
        Serial.print("Count=");
        Serial.println(MQTTRetryCount);
      }
      if ( MQTTRetryCount == MQTTMAXRetryCount )
      {
        MQTTAvailable = false;
      }
    }
    MQTTRetryCount = 0;
}

void ConnectWiFi(int Count)   // if count is zero then it will loop forever
{
  while (wifiMulti.run() != WL_CONNECTED)
  {
    M5.Lcd.drawXBitmap(280,0, WiFiErr, WiFiErrWidth, WiFiErrHeight, TFT_RED);
    delay(500);
    Serial.print(Count);
    Serial.print(" - ");
    if (M5stack == true )
    {
      M5.Lcd.setCursor(160,100);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(WHITE, BackGroundColor);
      M5.Lcd.println(Count);

    } 
    Count--;
    if ( Count == 0)  // its taken to long to connect so cycle the unit
    {
      Serial.println("Connection Failed - Rebooting");
      if (M5stack == true )
      {
        M5.Lcd.clear(BackGroundColor);
        M5.Lcd.setCursor(0,20);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setTextColor(WHITE, BackGroundColor);
        M5.Lcd.println("WIFI Connection Failed - Rebooting");
        delay(1000);
      }
      ESP.restart();
    }
  }
  M5.Lcd.drawXBitmap(280,195, WiFiErr, WiFiErrWidth, WiFiErrHeight, TFT_BLACK);
}

void MQTTSend(char* MQTTMsg,char* MQTTtopic )
{

char MQTTOutputText[300];
sprintf(MQTTOutputText, "%s:%s:", HOSTNAME, Version);  // Construct HOSTNAME and version message to a Char array Output Text
strcat(MQTTOutputText, MQTTMsg);                    // added OutputText and MQTT message together

  if ( MQTTAvailable )  // only send if MQTT is working
  {
    if ( WiFi.status() == WL_CONNECTED )
    {
      Serial.println("WiFi Connected");
    }
    else
    {
      Serial.println("ERROR - WiFi NOT Connected, reconnecting");
      ConnectWiFi(5);     //try 5 times then reboot
    }
  
    if (!MQTTclient.connected())
    {
      MQTTconnect();
    }
    
  
    if (MQTTclient.publish(MQTTtopic, MQTTOutputText, false))
    {
      Serial.print("Sent MQTT message: Topic=");
      Serial.print(MQTTtopic);
      Serial.print("   Msg=");
      Serial.println(MQTTOutputText);
    }
    else
    {
      Serial.print("*** ERROR *** Sending MQTT message: Topic=");
      Serial.print(MQTTtopic);
      Serial.print("   Msg=");
      Serial.println(MQTTOutputText);
      
    }
    MQTTclient.loop();
  }
  else
  {
    Serial.println("*** ERROR *** MQTT Not available");
    Serial.print(MQTTtopic);
    Serial.print("   Msg=");
    Serial.println(MQTTOutputText);
    M5.Lcd.drawXBitmap(280,195, MQTTErr, MQTTErrWidth, MQTTErrHeight, TFT_RED);

  }
}
/** END - MQTT Code **/

void SetupServo()
{
  servo1.attach(ServoPin);
  servo1.write(ServoOpen);
}

void CheckDoorTriggerTime()
{
  if ( now() >= (DoorTriggerTime + DoorTriggerWaitTime))
  {
    Serial.println("Use Servo to Close the Door");
    DoorTriggerTime = now() + 28000000;  // reset Trigger time to a year ahead so it is a valid time but way off
    BackGroundColor = BLUE;
    M5.Lcd.fillScreen(TFT_BLUE); // Black screen fill
    //M5.Lcd.println("Servo to Close position");
    M5.Lcd.drawXBitmap(50,6, DoorServo, DoorServoWidth, DoorServoHeight, TFT_RED);
    servo1.write(ServoClosed);
    delay(1000);
    //M5.Lcd.println("Servo to Open position");
    servo1.write(ServoOpen);
    delay(1000);
    M5.Lcd.drawXBitmap(50,6, DoorServo, DoorServoWidth, DoorServoHeight, TFT_BLACK);
    delay(1000);
    BackGroundColor = BLACK;
    M5.Lcd.fillScreen(TFT_BLACK); // Black screen fill
    M5.Lcd.drawXBitmap(26, 8, Cat, CatWidth, CatHeight, TFT_GREEN);
    sprintf(OutputText,"%s","Cat Door Servo Triggered");  // Construct OutputText message
    Serial.println(OutputText);
    MQTTSend(OutputText, MQTTOutputDiag);
  }
}


void CheckServoButtons()
{
  // update button state
  M5.update();
  
  if (M5.BtnA.wasReleased()) 
  {
    M5.Lcd.println("Servo to Open position");
    servo1.write(ServoOpen);
  }
  else if (M5.BtnB.wasReleased()) 
  {
    M5.Lcd.println("Toggle Debug Display");
    DebugDisplay = !DebugDisplay;
  } 
  else if (M5.BtnC.wasReleased()) 
  {
    M5.Lcd.println("Servo to Close position");
    servo1.write(ServoClosed);
  }
}

void LoopCounterDisplay()
{
  LoopCounter++;
  if ( LoopSecond + LoopDuration <= now())
  {
    PreviousCounter = LoopCounter;
    LoopCounter = 1;
    LoopSecond = now();
    sprintf(OutputText,"Loop Count %i" , PreviousCounter );  // Construct OutputText message
    Serial.println(OutputText);
  }
}

void HeartBeatMQTTMsg()
{
  int TargetMin = CurrentMin + MinToWaitFor;
  if ( TargetMin >= 60 ) { TargetMin = TargetMin - 60; }   // make sure its in the 0 to 59 range
  if ( TargetMin == minute() ) 
  {
    sprintf(OutputText,"Cat Door Running: %s Dist Sensor: %i" , Version, SensorDist);  // Construct OutputText message
    Serial.println(OutputText);
    MQTTSend(OutputText, MQTTOutputDiag);
    CurrentMin = minute();            //reset minute tracker
  }
}

void CheckDoor()
{
  DebugSensorDisplay();
  
  if (TOFSensor == true)      // only read if sensor was found at startup
  {
    SensorDist = VL53L0Xsensor.readRangeSingleMillimeters();
    if (VL53L0Xsensor.timeoutOccurred())
    {
      Serial.println("VL53L0X TIMEOUT");
      SensorDist = 50;            // default sensor distance if it timeouts
    }
    else
    {
      if ( SensorDist <= AlertDist )         // Door triggered
      {
        DoorTriggerOpen = true;
        DoorStatus = "OPEN";
      }
      else
      {
        DoorTriggerOpen = false;
        DoorStatus = "CLOSED";    
      }
    }
  }
  else
  {
    DoorStatus = "#TOF#";
    SensorDist = 9999;
  }
}


void OuputDoorStatus()
{

  if ( PrevDoorStatus != DoorStatus )
  {
    Serial.print("DoorStatus=");
    Serial.println(DoorStatus);
    Serial.print("Dist=");
    Serial.println(SensorDist);

    sprintf(OutputText,"CAT DOOR %s %i" , DoorStatus, SensorDist);  // Construct OutputText message
    Serial.println(OutputText);
    
    if ( DoorStatus == "OPEN" )
    {
      NeoAllPulse(0,0,0,10);
      pixels.SetPixelColor(0, RgbColor(NeoBrightness, 0, 0));
      pixels.SetPixelColor(5, RgbColor(NeoBrightness, 0, 0));
      pixels.SetPixelColor(6, RgbColor(NeoBrightness, 0, 0));
      pixels.SetPixelColor(10, RgbColor(NeoBrightness, 0, 0));
      pixels.Show();

      //Set time so auto close will be triggered
      DoorTriggerTime = now();
      Serial.println(DoorTriggerTime);
    }
    else
    {
      NeoAllPulse(0,0,0,10);
    }

    if (M5stack == true )
    {
      if ( DoorStatus == "OPEN" )
      {
        BackGroundColor = BLACK;
        M5.Lcd.fillScreen(TFT_BLACK); // Black screen fill
        M5.Lcd.drawXBitmap(40, 0, BadCat, BadCatWidth, BadCatHeight, TFT_RED);
      }
      else
      {
        BackGroundColor = BLACK;
        M5.Lcd.fillScreen(TFT_BLACK); // Black screen fill
        M5.Lcd.drawXBitmap(26, 8, Cat, CatWidth, CatHeight, TFT_GREEN);
      }
      if ( DoorStatus == "#TOF#" )
      {
        BackGroundColor = YELLOW;
        M5.Lcd.clear(BackGroundColor);
        M5.Lcd.setCursor(0,20);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setTextColor(WHITE, BackGroundColor);
        M5.Lcd.print("DoorStatus=");
        M5.Lcd.println(DoorStatus);
        M5.Lcd.print("Dist=");
        M5.Lcd.println(SensorDist);
      }
    }
    MQTTSend(OutputText, MQTTOutputTopic);

    sprintf(OutputText,"Sensor Dist %i" , SensorDist);  // Construct OutputText message
    Serial.println(OutputText);
    MQTTSend(OutputText, MQTTOutputDiag);

  }
  
  PrevDoorStatus = DoorStatus;

  // Show a beating heart symbol every loop
  if (M5stack == true )
  {
    if( TickTock == false )
    {
      M5.Lcd.drawXBitmap(1,195, Heart, HeartWidth, HeartHeight, TFT_RED);
    }
    else
    {
      M5.Lcd.drawXBitmap(1,195, Heart, HeartWidth, HeartHeight, TFT_BLACK);
    }
    TickTock = !TickTock;
  }

}


void SetupVL53L0X()
{
  // Check VL53L0X TOF and set for accurate scanning
  Wire.begin();
  Wire.beginTransmission (VL53L0XAddress);
  if (Wire.endTransmission() == 0)
  {
    Serial.print (F("VL53L0X device found at 0x"));
    Serial.println (VL53L0XAddress, HEX);
    
    // only set parameters if sensor was detected
    VL53L0Xsensor.init();
    VL53L0Xsensor.setTimeout(500);
    //VL53L0Xsensor.setMeasurementTimingBudget(200000);
  }
  else
  {
    Serial.println("######  VL53L0X device NOT found   ######");
    TOFSensor = false;
  }
}

void SetupMutiWifi()
{
  Serial.println("Starting wifi");
  if (M5stack == true )
  {
    M5.Lcd.clear(BackGroundColor);
    M5.Lcd.setCursor(0,50);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println("Starting wifi");
  }
  
  WiFi.setHostname(HOSTNAME);
  wifiMulti.addAP(NETWORKNAME1, NETWORKPASS1);
  wifiMulti.addAP(NETWORKNAME2, NETWORKPASS2);
  wifiMulti.addAP(NETWORKNAME3, NETWORKPASS3);
  wifiMulti.addAP(NETWORKNAME4, NETWORKPASS4);
    
  ConnectWiFi(9);        // restarts after 10 tries

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  WiFiIP = WiFi.localIP().toString().c_str();
  WiFiSID = WiFi.SSID();
  Serial.println(WiFiSID);
  Serial.println(WiFiIP);
  if (M5stack == true )
  {
    M5.Lcd.clear(BackGroundColor);
    M5.Lcd.setCursor(5,20);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println(WiFiSID);
    M5.Lcd.setCursor(5,50);
    M5.Lcd.setTextSize(3);
    M5.Lcd.println(WiFiIP);   
  }

  delay(STARTUPMSGDELAY);
}


void OutputBuildInfo()
{
  sprintf(Build, "%s %s", BuildTime, BuildDate );
  Serial.print(SKETCH);
  Serial.print(" ");
  Serial.println(Version);
  Serial.print("Build ");
  Serial.println(Build);
  Serial.print("ESP Name ");
  Serial.println(HOSTNAME);

  if (M5stack == true )
  {
    M5OutputBuildInfo();
  }

  
}

/** START - OAT Code **/
void SetupOTAFunctions()
{  
  // Start of OTA section

  Serial.println("OTA Setup Starting");
  if (M5stack == true )
  {
    BackGroundColor = TFT_BLACK;
    M5.Lcd.clear(BackGroundColor);
    M5.Lcd.setCursor(20,20);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println("OTA Setup Starting");
  }

    
  ArduinoOTA.setPort(OTAPort);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(HOSTNAME);

  // Hostname defaults to esp3232-[MAC]               // ArduinoOTA.setHostname("myesp32");
  // No authentication by default                     // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    if (M5stack == true )
    {
      BackGroundColor = TFT_BLACK;
      M5.Lcd.clear(BackGroundColor);
      M5.Lcd.setCursor(50,40);
      M5.Lcd.setTextSize(3);
      M5.Lcd.setTextColor(WHITE, BackGroundColor);
      M5.Lcd.println("OTA updating");
    }

    //sprintf(MQTTMsg, "%s: OTA update starting", HOSTNAME);  // Construct MQTT message
    delay(100);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    int DProgress = round((progress / (total / 100)));
    Serial.printf("%i%%\n", DProgress);
    if (M5stack == true )
    {
      BackGroundColor = TFT_BLACK;
      M5.Lcd.setCursor(120,100);
      M5.Lcd.setTextSize(6);
      M5.Lcd.setTextColor(WHITE, BackGroundColor);
      M5.Lcd.print(DProgress);
      M5.Lcd.print("%");
    }
  });
  ArduinoOTA.onEnd([]()
  {
    Serial.println("\nOTA Download Completed");
    //sprintf(MQTTMsg, "%s: OTA update completed successfully", HOSTNAME);  // Construct MQTT message
    //client.publish(StatusTopic, MQTTMsg);
    if (M5stack == true )
    {
      BackGroundColor = TFT_GREEN;
      M5.Lcd.clear(BackGroundColor);
      M5.Lcd.setCursor(0,20);
      M5.Lcd.setTextSize(3);
      M5.Lcd.setTextColor(BLACK, BackGroundColor);
      M5.Lcd.println("OTA Download Completed");
    }
    delay(STARTUPMSGDELAY);
  });
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    //sprintf(MQTTMsg, "%s: OTA update FAILED", HOSTNAME);  // Construct MQTT message
    //client.publish(StatusTopic, MQTTMsg);
    if (M5stack == true )
    {
      BackGroundColor = TFT_RED;
      M5.Lcd.clear(BackGroundColor);
      M5.Lcd.setCursor(0,20);
      M5.Lcd.setTextSize(3);
      M5.Lcd.setTextColor(WHITE, BackGroundColor);
      M5.Lcd.println("OTA update FAILED");
    }
    
    delay(STARTUPMSGDELAY);

    if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  
  Serial.println("OTA System Ready");
  Serial.println("Checking for OTA Update");

  // End of OTA section
  if (M5stack == true )
  {
    BackGroundColor = TFT_BLUE;
    M5.Lcd.clear(BackGroundColor);
    M5.Lcd.setCursor(0,20);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println("OTA System Ready");
    M5.Lcd.println("Checking for OTA Update");
  }


  for (int i = 2; i < 13; i++) {
    if (i % 2) {  // if progess is even then turn on blue pixel, if odd then turn it off
      Serial.println("CHECK FOR OTA");
    } else {
      Serial.println("|||||||||||||");
    }

    // Start of OTA section
    ArduinoOTA.handle();
    yield();
    // End of OTA section
    delay(750);       // delay checking loop by 500ms
  }
  
  Serial.println("OTA Setup Completed");
  if (M5stack == true )
  {
    BackGroundColor = TFT_BLUE;
    M5.Lcd.clear(BackGroundColor);
    M5.Lcd.setCursor(0,20);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println("OTA System Ready");
    M5.Lcd.println("OTA Setup Completed");
  }

  delay(STARTUPMSGDELAY);    // delay so that no issues with ezM5 library occurs

}
/** END - OAT Code **/

/** START - MQTT Code **/
void MQTTSetup()
{
  if (M5stack == true )
  {
    BackGroundColor = TFT_BLUE;
    M5.Lcd.clear(BackGroundColor);
    M5.Lcd.setCursor(0,20);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextColor(WHITE, BackGroundColor);
    M5.Lcd.println("Starting MQTT");
  }

  MQTTclient.setServer(MQTT_SERVER_CentralHub, 1883);
  Serial.println("MQTT Server Set");
  //MQTTclient.setConnectionTimeout(-1);
  //MQTTclient.loop();
  
  Serial.println("Sending MQTT Start Msg");
  MQTTSend("M5 Stack started", MQTTOutputDiag);             //(re)connects, subscribes and sends message


}



/*
 * ###################################################
 * #              START SETUP FUNCTION               #############################################
 * ###################################################
*/

void setup()
{
	Serial.begin(115200);

  SetupServo();
  
  SetupNeoPixels();
  NeoAllPulse(255,255,255,20);
  
  if (M5stack == true )
  {
    M5.begin();
  }

  SetupVL53L0X();
  
  OutputBuildInfo();
  
  SetupMutiWifi();

  SetupOTAFunctions();

  MQTTSetup();

  LoopSecond = now();                //intial i2c timer

  CurrentMin = minute();

  NeoAllPulse(0,255,0,20);
}


/*
 * ###################################################
 * #                 MAIN LOOP                       ##############################################
 * ###################################################
*/
void loop()
{
  delay(MAINLOOPDELAY);

  ArduinoOTA.handle();      // Check for an OTA update request
  yield();

  CheckDoor();

  OuputDoorStatus();

  MQTTclient.loop();

  HeartBeatMQTTMsg();

  CheckServoButtons();

  CheckDoorTriggerTime();
  
  //LoopCounterDisplay();
}
/*
 * ###################################################
 * #               END MAIN LOOP                     #
 * ###################################################
*/