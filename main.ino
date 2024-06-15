#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <NTPClient.h>
#include <WebSerial.h>
#include <ESPDash.h>
#include <Preferences.h>
#include <Ticker.h>
#include <Servo.h>

// include MDNS
#ifdef ESP8266
#include <ESP8266mDNS.h>
#elif defined(ESP32)
#include <ESPmDNS.h>
#endif

#define SERVO_GPIO 4
#define SHAKER_GPIO 5

#define SERVO_INIT_POS 0
#define SERVO_OPEN_POS 180
#define SHAKER_ON 0
#define SHAKER_OFF 1
#define SHAKER_DUR 100
#define SHAKER_GAP 100

#define WIFI_RESET_GPIO 14
#define WIFI_RESET_TRIG 0
#define WIFI_RESET_NORMAL 1

WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "pool.ntp.org");
AsyncWebServer server(80);
DNSServer dns;
AsyncWiFiManager wm(&server, &dns);
ESPDash dashboard(&server);
Ticker ticker;
Servo servo;
Preferences pref;

// Dashboard things
Card elBtnShaker(&dashboard, BUTTON_CARD, "Shaker");
Card elServoSlider(&dashboard, SLIDER_CARD, "Servo", "", 0, 180, 0);
Card elCurrentTime(&dashboard, GENERIC_CARD, "Current Time");
Card elHourSet(&dashboard, SLIDER_CARD, "Hour", "", 0, 23, 0);
Card elMinuteSet(&dashboard, SLIDER_CARD, "Minute", "", 0, 59, 0);
Card elOpenDuration(&dashboard, SLIDER_CARD, "Open Duration", "", 1, 60);
Card elOpenServo(&dashboard, SLIDER_CARD, "Open Servo Degree", "", 0, 180);

int servoPos;
bool ntpBegan;

void setup() {
  Serial.begin(115200);

  pinMode(BUILTIN_LED, OUTPUT);
  ticker.attach(0.5, tick);  // boot up indicator

  pinMode(WIFI_RESET_GPIO, INPUT_PULLUP);

  servo.attach(SERVO_GPIO);
  moveServoDelay(0);  // initial position
  pinMode(SHAKER_GPIO, OUTPUT);
  digitalWrite(SHAKER_GPIO, SHAKER_OFF);

  // preferences
  pref.begin("dma", false);

  WiFi.mode(WIFI_STA);
  wm.setConfigPortalTimeout(180);
  wm.setSaveConfigCallback(wmConfigSetCallback);
  wm.setAPCallback(wmAPCallback);
  wm.setAPStaticIPConfig(IPAddress(192, 168, 88, 1), IPAddress(192, 168, 88, 1), IPAddress(255, 255, 255, 0));
  wm.autoConnect();

  ticker.detach();
  digitalWrite(BUILTIN_LED, HIGH);

  // Initialize WebSerial
  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);

  // Start the server
  server.begin();
  setupOTA();
  dashboardSetup();
}

void loop() {
  wm.loop();
  wifiResetLoop();
  ntpLoop();
  WebSerial.loop();
  dashboardLoop();
  ArduinoOTA.handle();
  servoLoop();
  // no interrupt
  while (openLoop()) {
    servoLoop();
  }
}

/* Message callback of WebSerial */
void recvMsg(uint8_t* data, size_t len) {
  Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  Serial.println();
  WebSerial.println("Received Data...");
  String str = "";
  for (size_t i = 0; i < len; i++) {
    str += char(data[i]);
  }
  WebSerial.println(str);

  if (str.compareTo("wifi reset") == 0) {
    wm.resetSettings();
    ESP.restart();
  }

  if (str.compareTo("time") == 0) {
    WebSerial.println(ntp.getFormattedTime());
  }

  if (str.compareTo("reboot") == 0) {
    WebSerial.println("rebooting...");
    ticker.attach(0.1, tick);
    delay(500);
    ESP.restart();
  }
}

int openLoop() {
  static int open = 0;
  static int lastOpen = 0;

  const int runDelay = 100;
  static int lastRun = 0;
  if (!((millis() - lastRun) > runDelay)) {
    return open;
  }

  int dur = pref.getInt("opendur") * 1000;
  static int lastOpenDay = -1;
  static int lastOpenHours = -1;
  static int lastOpenMinutes = -1;
  if (!open && ntp.getHours() == pref.getInt("hour") && ntp.getMinutes() == pref.getInt("minute")) {
    if (!(lastOpenDay == ntp.getDay() && lastOpenHours == ntp.getHours() && lastOpenMinutes == ntp.getMinutes())) {
      lastOpenDay = ntp.getDay();
      lastOpenHours = ntp.getHours();
      lastOpenMinutes = ntp.getMinutes();

      ticker.attach(0.4, tick);
      open = 1;
      servoPos = pref.getInt("opensrv");
      lastOpen = millis();
    }
  }
  if (open && (millis() - lastOpen >= dur)) {
    open = 0;
    ticker.detach();

    shakerSwitch(SHAKER_OFF);
    digitalWrite(BUILTIN_LED, HIGH); 
    servoPos = SERVO_INIT_POS;
    return open;
  }

  static int lastShake = 0;
  if (open) {
    if (millis() > (lastShake + SHAKER_DUR)) {
      shakerSwitch(SHAKER_OFF);
    }
    if (millis() > (lastShake + SHAKER_DUR + SHAKER_GAP)) {
      shakerSwitch(SHAKER_ON);
      lastShake = millis();
    }
  }

  lastRun = millis();
  return open;
}

void tick() {
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}


void dashboardSetup() {
  elBtnShaker.attachCallback([&](int value) {
    if (value == 1) {
      shakerSwitch(SHAKER_ON);
    } else {
      shakerSwitch(SHAKER_OFF);
    }
    elBtnShaker.update(value);
    dashboard.sendUpdates();
  });

  elServoSlider.attachCallback([&](float value) {
    servoPos = (int)value;
    elServoSlider.update(servoPos);
    dashboard.sendUpdates();
  });

  elHourSet.attachCallback([&](int value) {
    pref.putInt("hour", value);
    elHourSet.update(pref.getInt("hour"));
    dashboard.sendUpdates();
  });

  elMinuteSet.attachCallback([&](int value) {
    pref.putInt("minute", value);
    elMinuteSet.update(pref.getInt("minute"));
    dashboard.sendUpdates();
  });

  elOpenDuration.attachCallback([&](int value) {
    pref.putInt("opendur", value);
    elOpenDuration.update(pref.getInt("opendur"));
    dashboard.sendUpdates();
  });

  elOpenServo.attachCallback([&](int value) {
    pref.putInt("opensrv", value);
    elOpenServo.update(pref.getInt("opensrv"));
    dashboard.sendUpdates();
  });
}

void dashboardLoop() {
  const long delay = 500;
  static int lastRun = 0;
  if ((millis() - lastRun) >= delay) {
    // send current time
    elCurrentTime.update(ntp.getFormattedTime());
    elHourSet.update(pref.getInt("hour"));
    elMinuteSet.update(pref.getInt("minute"));
    elOpenDuration.update(pref.getInt("opendur"));
    elOpenServo.update(pref.getInt("opensrv"));

    dashboard.sendUpdates();
    lastRun = millis();
  }
}

void ntpLoop() {
  // no need high accuracy
  const long delay = 60 * 1000;
  static int last = 0;

  if ((millis() - last) >= delay) {
    last = millis();
    if (WiFi.status() == WL_CONNECTED && !ntpBegan) {
      ntp.begin();
      ntp.setTimeOffset(25200);  // gmt +7
      ntpBegan = true;
      return;
    }

    ntp.update();
  }
}

void servoLoop() {
  const long servoDelay = 10;
  static int lastRun = 0;
  if (servo.read() != servoPos && (millis() - lastRun) >= servoDelay) {
    int incr = 1;
    if (servoPos < servo.read()) { incr = -1; }

    int next = servo.read() + incr;
    if (next > 180) { next = 180; }
    if (next < 0) { next = 0; }

    servo.write(next);
    lastRun = millis();
  }
}

void moveServoDelay(int pos) {
  int incr = 1;
  if (pos > 180) { pos = 180; }
  if (pos < 0) { pos = 0; }
  if (pos < servo.read()) { incr = -1; }
  for (int i = servo.read(); i != pos + incr; i += incr) {
    servo.write(pos);
    delay(15);
  }
}

void shakerSwitch(int state) {
  digitalWrite(SHAKER_GPIO, state);
}

void wmConfigSetCallback() {
  ESP.restart();
}

void wmAPCallback(AsyncWiFiManager* myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void setupOTA() {
  ArduinoOTA.setHostname("dmacatfeeder");
  ArduinoOTA.setPassword("catfeederbydma");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void wifiResetLoop() {
  const int wifiResetHold = 5000;
  const int wifiResetCheck = 1000;
  static int lastResetCheck = 0;
  static int lastResetHold = 0;
  static int lastResetHoldVal = 0;

  int currentResetVal = digitalRead(WIFI_RESET_GPIO);

  if ((millis() - lastResetCheck) >= wifiResetCheck) {
    if (lastResetHoldVal == WIFI_RESET_NORMAL && currentResetVal == WIFI_RESET_TRIG) {
      WebSerial.println("Reset pressed");
      ticker.attach(0.5, tick);
      lastResetHold = millis();
    }
    if (lastResetHoldVal == WIFI_RESET_TRIG && currentResetVal == WIFI_RESET_NORMAL) {
      WebSerial.println("Reset released");
      ticker.detach();
      digitalWrite(BUILTIN_LED, HIGH);
    }
    lastResetHoldVal = currentResetVal;
  }

  if (currentResetVal == WIFI_RESET_TRIG && (millis() - lastResetHold) >= wifiResetHold) {
    wm.resetSettings();
    ESP.restart();
  }
}
