#include <Arduino.h>

#include <FS.h>
#include <LittleFS.h>

// Load the TFT definitions before including the library main header
#include <User_Setups/Setup25_TTGO_T_Display.h>
#define USER_SETUP_LOADED 1
#include <TFT_eSPI.h>

#include <Wire.h>
#include <MAX44009.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <Adafruit_PM25AQI.h>

#include <MHZ19.h>

#include <ESP_WiFiManager.h>

#include <PubSubClient.h>

#include <Ticker.h>

#include "PMS5003.h"

#include <ArduinoJson.h>

#include "assets/icons.h"
#include "ValueHistory.h"

#define VALUE_FONT &Orbitron_Light_24

ESP_WiFiManager wifiManager;
WiFiClient client;
char mqtt_server[40] = "192.168.178.150";
char room[40] = "";

PubSubClient mqtt(client);

const static uint8_t resetButton = 0;   //GPIO 0
const static uint8_t portalButton = 35; //GPIO 35

const static uint16_t pmWarnThreshold = 10;
const static uint16_t pmDangerThreshold = 25;

SoftwareSerial pmsSerial(15, 17);
PMS5003 pms = PMS5003();

SoftwareSerial co2Serial(13, 12);
MHZ19 co2;
MAX44009 brightness;

TFT_eSPI display = TFT_eSPI();

//flag for saving data
bool shouldSaveConfig = false;

void setupOTA();
void delayWhileCheckingButtons(uint32_t time);
void saveConfigCallback();
void checkButtons();
void loadWLANConfig();
void saveWLANConfig();
void setupWLAN();
void displayPrintCenterln(const char *text, uint8_t y);
void createInfluxMessage(char *dst, uint8_t len, const char *topic, float value);
void createParticleMessage(char *dst, uint8_t len, const char *topic, uint16_t value, float size);
void displayMessage(uint16_t duration, const uint16_t *icon, const char *message1, const char *message2 = "");
void displayParticleCount();
void displayConnectInfo(String ssid, String passphrase, uint16_t duration = 5000);

ValueHistory<uint16_t> pm010History;
ValueHistory<uint16_t> pm025History;
ValueHistory<uint16_t> pm100History;
ValueHistory<uint16_t> co2History;
ValueHistory<float> brightnessHistory;

void setup()
{
  pinMode(resetButton, INPUT);
  pinMode(portalButton, INPUT);

  // start the serial connection
  Serial.begin(115200);
  // wait for serial monitor to open
  while (!Serial)
    ;

  display.init();
  display.setRotation(1);

  display.setTextColor(TFT_BLACK, TFT_WHITE);
  display.fillScreen(TFT_WHITE);
  delay(500);

#ifndef OFFLINE_MODE
  loadWLANConfig();
  setupWLAN();

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveWLANConfig();
  }

  setupOTA();

  mqtt.setServer(mqtt_server, 1883);
#endif

  if (!(pms.begin(&pmsSerial) == PMS5003::readSuccess))
  {
    displayMessage(5000, warningIcon, "PMS Sensor Error", "restarting");
    display.fillScreen(TFT_WHITE);
    ESP.restart();
    delay(5000);
  }

  Wire.begin();

  delay(500);

  if (brightness.begin())
  {
    Serial.println("Could not find a valid MAX44009 sensor, check wiring!");
    displayMessage(5000, warningIcon, "Brightness Sensor Error", "restarting");
    display.fillScreen(TFT_WHITE);
    ESP.restart();
    delay(5000);
  }

  co2Serial.begin(9600);
  co2.begin(co2Serial);
  Serial.print("CO2 Sensor initialized");
  co2.autoCalibration(false);
  Serial.print("ABC Status: ");
  co2.getABC() ? Serial.println("ON") : Serial.println("OFF");

  pms.setMode(PmsMode::passive);
  pms.reset();
}

void loop()
{
  uint16_t loopStart = millis();

#ifndef OFFLINE_MODE
  if (WiFi.status() != WL_CONNECTED)
  {
    displayMessage(5000, connectedIcon, "disconnected", "resetting");
    ESP.restart();
  }

  if (!mqtt.connected())
  {
    String clientId = String("AtmoNode-") + room;
    if (!mqtt.connect(clientId.c_str()))
    {
      displayMessage(5000, connectedIcon, "MQTT failed", "retrying");
      return;
    }
  }
#endif

  Serial.print("Sensing for room ");
  Serial.println(room);

  PMSResult pmsData;
  uint8_t err = pms.getReading(&pmsData);

  Serial.println("AQI Reding result = " + String(err));
  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: "));
  Serial.print(pmsData.pm10_standard);
  Serial.print(F("\t\tPM 2.5: "));
  Serial.print(pmsData.pm25_standard);
  Serial.print(F("\t\tPM 10: "));
  Serial.println(pmsData.pm100_standard);
  Serial.println(F("Concentration Units (environmental)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: "));
  Serial.print(pmsData.pm10_env);
  Serial.print(F("\t\tPM 2.5: "));
  Serial.print(pmsData.pm25_env);
  Serial.print(F("\t\tPM 10: "));
  Serial.println(pmsData.pm100_env);
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Particles > 0.3um / 0.1L air:"));
  Serial.println(pmsData.particles_03um);
  Serial.print(F("Particles > 0.5um / 0.1L air:"));
  Serial.println(pmsData.particles_05um);
  Serial.print(F("Particles > 1.0um / 0.1L air:"));
  Serial.println(pmsData.particles_10um);
  Serial.print(F("Particles > 2.5um / 0.1L air:"));
  Serial.println(pmsData.particles_25um);
  Serial.print(F("Particles > 5.0um / 0.1L air:"));
  Serial.println(pmsData.particles_50um);
  Serial.print(F("Particles > 10 um / 0.1L air:"));
  Serial.println(pmsData.particles_100um);
  Serial.println(F("---------------------------------------"));

  int currentCo2 = co2.getCO2();
  int co2Temp = co2.getTemperature();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("CO2 (PPM):"));
  Serial.println(F("---------------------------------------"));
  Serial.println(currentCo2);
  Serial.print(F("Sensor Temperature:"));
  Serial.println(co2Temp);
  Serial.println(F("---------------------------------------"));

  float currentLux = brightness.get_lux();
  Serial.print("Light (lux): ");
  Serial.println(currentLux);

  yield();

  co2History.addMeasurement(currentCo2);
  pm010History.addMeasurement(pmsData.pm10_standard);
  pm025History.addMeasurement(pmsData.pm25_standard);
  pm100History.addMeasurement(pmsData.pm100_standard);
  brightnessHistory.addMeasurement(currentLux);

#ifndef OFFLINE_MODE
  // send data to the server
  String baseTopic = String("atmonode/") + room + "/";
  mqtt.publish(String(baseTopic + "co2").c_str(), String(currentCo2).c_str());
  mqtt.publish(String(baseTopic + "pm10").c_str(), String(pmsData.pm10_standard).c_str());
  mqtt.publish(String(baseTopic + "pm25").c_str(), String(pmsData.pm25_standard).c_str());
  mqtt.publish(String(baseTopic + "pm100").c_str(), String(pmsData.pm100_standard).c_str());
  yield();

  // messages for storing the data in influxdb
  const char *persistentTopic = "atmonode";
  char messageBuffer[50] = {0};
  createInfluxMessage(messageBuffer, 50, "co2", currentCo2);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pm10_std", pmsData.pm10_standard);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pm25_std", pmsData.pm25_standard);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pm100_std", pmsData.pm100_standard);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pm10_env", pmsData.pm10_env);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pm25_env", pmsData.pm25_env);
  mqtt.publish(persistentTopic, messageBuffer);

  createInfluxMessage(messageBuffer, 50, "pm100_env", pmsData.pm100_env);
  mqtt.publish(persistentTopic, messageBuffer);

  createParticleMessage(messageBuffer, 50, "particles", pmsData.particles_03um, 0.3);
  Serial.println(messageBuffer);
  mqtt.publish(persistentTopic, messageBuffer);
  createParticleMessage(messageBuffer, 50, "particles", pmsData.particles_05um, 0.5);
  mqtt.publish(persistentTopic, messageBuffer);
  createParticleMessage(messageBuffer, 50, "particles", pmsData.particles_10um, 1.0);
  mqtt.publish(persistentTopic, messageBuffer);
  createParticleMessage(messageBuffer, 50, "particles", pmsData.particles_25um, 2.5);
  mqtt.publish(persistentTopic, messageBuffer);
  createParticleMessage(messageBuffer, 50, "particles", pmsData.particles_50um, 5.0);
  mqtt.publish(persistentTopic, messageBuffer);
  createParticleMessage(messageBuffer, 50, "particles", pmsData.particles_100um, 10.0);
  mqtt.publish(persistentTopic, messageBuffer);
#endif

  yield();

  displayParticleCount();
  uint16_t loopDuration = millis() - loopStart;
  if (loopDuration < 60 * 1000)
  {
    //make sure to run the loop every 60s
    delayWhileCheckingButtons(60 * 1000 - loopDuration);
  }
}

void setupOTA()
{
  ArduinoOTA.onStart([]()
                     { displayMessage(1, warningIcon, "update in", "progress"); });
  ArduinoOTA.onEnd([]()
                   { displayMessage(1000, warningIcon, "done", "restarting"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
                          Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

                          displayMessage(0, warningIcon, "Progress", String(String(progress / (total / 100), 10) + "%").c_str());
                        });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         Serial.println("Auth Failed");
                       else if (error == OTA_BEGIN_ERROR)
                         Serial.println("Begin Failed");
                       else if (error == OTA_CONNECT_ERROR)
                         Serial.println("Connect Failed");
                       else if (error == OTA_RECEIVE_ERROR)
                         Serial.println("Receive Failed");
                       else if (error == OTA_END_ERROR)
                         Serial.println("End Failed");
                     });
  ArduinoOTA.begin();
}

void delayWhileCheckingButtons(uint32_t time)
{
  uint32_t start = millis();
  while (millis() - start < time)
  {
    // Handle OTA update server
    ArduinoOTA.handle();

    checkButtons();
    delay(5);
  }
}

void checkButtons()
{
  if (!digitalRead(resetButton))
  {
    ESP.restart();
  }

  if (!digitalRead(portalButton))
  {
    wifiManager.resetSettings();
    ESP.restart();
  }
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void loadWLANConfig()
{
  if (LITTLEFS.begin(true))
  {
    Serial.println("mounted file system");
    if (LITTLEFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LITTLEFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(size);
        auto error = deserializeJson(doc, buf.get());
        serializeJson(doc, Serial);
        if (error)
        {
          Serial.println("failed to load json config");
          Serial.println(error.c_str());
        }
        else
        {
          Serial.println("\nparsed json");
          if (doc.containsKey("mqtt_server"))
          {
            strcpy(mqtt_server, doc["mqtt_server"]);
          }
          strcpy(room, doc["room"]);
        }
        configFile.close();
      }
    }
    else
    {
      wifiManager.resetSettings();
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    wifiManager.resetSettings();
  }
}

void saveWLANConfig()
{
  Serial.println("saving config");
  DynamicJsonDocument doc(512);
  doc["mqtt_server"] = mqtt_server;
  doc["room"] = room;

  File configFile = LITTLEFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(doc, Serial);
  serializeJson(doc, configFile);
  configFile.close();
}

void setupWLAN()
{
  // auto-manage wifi configuration
  ESP_WMParameter mqtt_server_param("MQTT Server", "IP or hostname", mqtt_server, 32);
  ESP_WMParameter room_param("Room", "room", room, 32);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&mqtt_server_param);
  wifiManager.addParameter(&room_param);

  // create a unique SSID
  String apSSID = String("AtmoNode") + String(random(10, 100), DEC);
  Serial.print("Random accesspoint SSID is: ");
  Serial.println(apSSID);

  // create a random numeric 4-digit password
  String apPasscode = String(random(1000, 10000), DEC);
  apPasscode += String(random(1000, 10000), DEC);
  Serial.print("Random accesspoint password is: ");
  Serial.println(apPasscode);

  if (!MDNS.begin("atmo"))
  {
    Serial.println("Error setting up mDNS responder!");
    displayMessage(2000, wlanIcon, "mDNS fail");
  }

  displayConnectInfo(apSSID, apPasscode);
  wifiManager.setTimeout(60);
  bool success = wifiManager.autoConnect(apSSID.c_str(), apPasscode.c_str());

  if (!success)
  {
    Serial.println("failed to connect and hit timeout");

    displayMessage(5000, wlanIcon, "WiFi failed", "restarting");
    display.fillScreen(TFT_WHITE);
    ESP.restart();
    delay(5000);
  }

  //read updated parameters
  strcpy(mqtt_server, mqtt_server_param.getValue());
  strcpy(room, room_param.getValue());

  Serial.println("connected...");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  displayMessage(2000, wlanIcon, "connected");
  displayMessage(2000, wlanIcon, WiFi.SSID().c_str());

  display.fillScreen(TFT_WHITE);
}

void createInfluxMessage(char *dst, uint8_t len, const char *topic, float value)
{
  uint8_t pos = 0;
  strncpy(dst + pos, topic, len - pos);
  pos += strlen(topic);
  strncpy(dst + pos, ",site=", len - pos);
  pos += 6;
  strncpy(dst + pos, room, len - pos);
  pos += strlen(room);
  strncpy(dst + pos, " value=", len - pos);
  pos += 7;
  String val_str = String(value);
  strncpy(dst + pos, val_str.c_str(), len - pos);
}

void createParticleMessage(char *dst, uint8_t len, const char *topic, uint16_t value, float size)
{
  uint8_t pos = 0;

  strncpy(dst + pos, topic, len - pos);
  pos += strlen(topic);

  strncpy(dst + pos, ",site=", len - pos);
  pos += 6;

  strncpy(dst + pos, room, len - pos);
  pos += strlen(room);

  strncpy(dst + pos, ",size=", len - pos);
  pos += 6;

  String sizeStr = String(size, 1);
  strncpy(dst + pos, sizeStr.c_str(), len - pos);
  pos += sizeStr.length();

  strncpy(dst + pos, " value=", len - pos);
  pos += 7;

  String val_str = String(value);
  strncpy(dst + pos, val_str.c_str(), len - pos);
}

void displayMessage(uint16_t duration, const uint16_t *icon, const char *message1, const char *message2)
{
  display.setTextFont(2);
  display.fillScreen(TFT_WHITE);
  display.pushImage(
      (display.width() - iconWidth) / 2, 0,
      iconWidth, iconHeight, icon);
  displayPrintCenterln(message1, iconHeight + 5);
  displayPrintCenterln(message2, display.getCursorY());
  delay(duration);
}

void displayParticleCount()
{
  const uint8_t paddingL = 25;
  const uint8_t paddingR = 10;
  const uint8_t paddingT = 35;
  const uint8_t paddingB = 20;

  display.fillScreen(0x10A3);
  display.setTextFont(2);

  const uint8_t hourBarWidth = (display.width() - 60 - (paddingL + paddingR)) / (ValueHistory<uint16_t>::hourlyBufferLength);
  const uint32_t maxParticleVal = pm010History.getMaxValue() + pm025History.getMaxValue() + pm100History.getMaxValue();
  const uint32_t minParticleVal = pm010History.getMinValue() + pm025History.getMinValue() + pm100History.getMinValue();
  const uint32_t graphLowerBound = (minParticleVal / 10) * 10;
  const uint32_t graphUpperBound = ((maxParticleVal / 10) + 1) * 10;
  auto particleCountToBarHeight = [graphLowerBound, graphUpperBound](uint16_t value)
  {
    auto unit = ((display.height() - (paddingT + paddingB)) / ((float)graphUpperBound - graphLowerBound));
    return (uint16_t)(paddingB + (value - graphLowerBound) * unit);
  };

  auto textColorValue = [](uint16_t value)
  {
    if (value >= pmDangerThreshold)
    {
      return TFT_RED;
    }
    else if (value >= pmWarnThreshold)
    {
      return TFT_YELLOW;
    }
    else
    {
      return TFT_GREEN;
    }
  };

  uint16_t xPos = paddingL;
  auto yMax = display.height();

  // draw a grid line dividing 6 hour steps
  display.setTextColor(TFT_DARKGREY, 0x10A3);
  display.setTextDatum(TC_DATUM);
  for (uint8_t lx = 0; lx <= ValueHistory<uint16_t>::hourlyBufferLength / 6; lx++)
  {
    auto legendX = xPos + ((lx)*hourBarWidth * 6);
    display.drawLine(legendX, paddingT, legendX, display.height() - paddingB, TFT_DARKGREY);
    auto timeOffset = ValueHistory<uint16_t>::hourlyBufferLength - (6 * lx) + 1;
    String label = String("-") + String(timeOffset) + "h";
    display.drawString(label, legendX, (display.height() - paddingB) + 2);
  }

  //draw horizontal grid lines to divide the chart in 5 spaces
  auto chartHeight = display.height() - (paddingT + paddingB);
  display.setTextDatum(MR_DATUM);
  for (uint8_t ly = 0; ly < 5; ly++)
  {
    auto lineY = paddingT + (chartHeight / 5) * ly;
    display.drawLine(paddingL, lineY, display.width() - paddingR, lineY, TFT_DARKGREY);
    uint16_t value = (((graphUpperBound - graphLowerBound) / 5) * (5 - ly)) + graphLowerBound;
    display.drawString(String(value), paddingL - 2, lineY + 5);
  }

  for (int8_t hourIdx = ValueHistory<uint16_t>::hourlyBufferLength - 2; hourIdx >= 0; hourIdx--)
  {
    auto pm10Height = particleCountToBarHeight(pm010History.hourlyData[hourIdx]);
    auto pm25Height = particleCountToBarHeight(pm025History.hourlyData[hourIdx]);
    auto pm100Height = particleCountToBarHeight(pm100History.hourlyData[hourIdx]);

    display.fillCircle(xPos, yMax - pm10Height, 1, 0x854E);
    display.fillCircle(xPos, yMax - pm25Height, 1, 0xDDAA);
    display.fillCircle(xPos, yMax - pm100Height, 1, 0x865A);
    xPos += hourBarWidth;
  }

  for (int8_t minIdx = ValueHistory<uint16_t>::lastHourBufferLength - 1; minIdx >= 0; minIdx--)
  {
    auto pm10Height = particleCountToBarHeight(pm010History.lastHourData[minIdx]);
    auto pm25Height = particleCountToBarHeight(pm025History.lastHourData[minIdx]);
    auto pm100Height = particleCountToBarHeight(pm100History.lastHourData[minIdx]);
    display.fillCircle(xPos, yMax - pm10Height, 1, 0x854E);
    display.fillCircle(xPos, yMax - pm25Height, 1, 0xDDAA);
    display.fillCircle(xPos, yMax - pm100Height, 1, 0x865A);
    xPos += 1;
  }

  display.setTextDatum(TC_DATUM);
  display.setTextColor(0x854E, 0x10A3);
  display.drawString("PM", 13, 0);
  display.drawString("1.0", 13, display.fontHeight());

  display.setTextDatum(TL_DATUM);
  display.setTextColor(textColorValue(pm010History.lastData), 0x10A3);
  display.setFreeFont(VALUE_FONT);
  display.drawString(String(pm010History.lastData), 30, 1);

  auto pm025Value = String(pm025History.lastData);
  auto pm025Width = display.textWidth(pm025Value);

  display.setTextFont(2);
  display.setTextDatum(TC_DATUM);
  auto pm025labelX = (display.width() - (pm025Width + 30)) / 2;

  display.setTextColor(0xDDAA, 0x10A3);
  display.drawString("PM", pm025labelX, 0);
  display.drawString("2.5", pm025labelX, display.fontHeight());

  display.setFreeFont(VALUE_FONT);
  display.setTextColor(textColorValue(pm025History.lastData), 0x10A3);
  display.drawString(pm025Value, pm025labelX + 30, 1);

  auto pm100Value = String(pm100History.lastData);
  auto pm100Width = display.textWidth(pm100Value);

  display.setTextFont(2);
  display.setTextDatum(TC_DATUM);
  display.setTextColor(0x865A, 0x10A3);
  auto pm100labelX = (display.width() - (pm100Width + 30));
  display.drawString("PM", pm100labelX, 0);
  display.drawString("10", pm100labelX, display.fontHeight());

  display.setFreeFont(VALUE_FONT);
  display.setTextColor(textColorValue(pm100History.lastData), 0x10A3);
  display.drawString(pm100Value, pm100labelX + 30, 1);
}

void displayPrintCenterln(const char *text, uint8_t y)
{
  int16_t width = display.textWidth(text);
  display.setCursor((display.width() / 2) - (width / 2), y);
  display.println(text);
}

void displayConnectInfo(String ssid, String passphrase, uint16_t duration)
{
  uint16_t posY = 5;
  display.setTextColor(TFT_BLACK, TFT_WHITE);
  display.fillScreen(TFT_WHITE);

  display.setTextFont(4);
  display.setTextDatum(TC_DATUM);
  display.drawString("Connect to WLAN", display.width() / 2, posY);
  posY += display.fontHeight();
  display.pushImage((display.width() / 2) - (iconWidth / 2), posY, iconWidth, iconHeight, wlanIcon);
  posY += iconHeight;

  display.setTextFont(2);
  display.setTextDatum(TL_DATUM);
  display.drawString("Wlan SSID:", 10, posY);
  display.setTextDatum(TR_DATUM);
  display.drawString(ssid, display.width() - 10, posY);
  posY += display.fontHeight();

  display.setTextDatum(TL_DATUM);
  display.drawString("Passphrase:", 10, posY);
  display.setTextDatum(TR_DATUM);
  display.drawString(passphrase, display.width() - 10, posY);

  delay(duration);
}
