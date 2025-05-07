/* --------------------------------------------------------------------------------
Controller for 12 volt generator based on Honda GX25 four-stroke engine and
a belt driven BLDC (brushless) motor as dynamo through a 3-phase to DC rectifier.

The controller uses two servos, a hall sensor and three INA226 voltage monitors to monitor and govern the engine.
One servo controls the throttle to generate approx 20 volts 3-phase current. One INA226 sensor reads the rectified 3-phase voltage from the dynamo and adjust the servo accoringly. Another INA226 sensor monitors the regulated 12 volt output voltage and a hall sensor reads the engine RPM. These measurements are mainly for display and fault detection. 
The other servo operates the engine choke below 3000 RPM. The choke can be engaged by a button on the generator or through a web-interface.
The generator also has an internal batterypack consisting of three 18650 3.7 volt batteries, and an Uninterupted Power Supply/Charger. Total voltage of batteries are monitored with one INA226 voltage sensor, and outputs from charger circuit status LEDs (through opto-couplers on pin 32 and 33). 

The generator can connect to an existing wifi network or set up it own Wifi Access Point. Through wifi the generator serve a basic website to monitor and control the operation.

One webpage displays output voltage, dynamo voltage, battery status, RPM and a STOP button and a CHOKE button and. Below 3000 RPM, pressing the CHOKE button reveal slider controls for the choke- and the throttle servo. When the choke-panel is open the throttle is also fully manual.
Pressing the CHOKE button on the generator will set choke to 80% and open the choke-panel in the web interface.
Pressing the real hardware buttons buttons on the generator will also be reflected in the web-interface.

A Settings page shows the current network status and options to set up credetials to a local wifi-network. There are also a button to SCAN for local networks and a button to DELETE stored credentials. The web interface use WebSockets to update between the webpage and the generator. Several clients can be connected simultaniously and get updates in realtime. 
In the web interface you can also restart the controller, even while the generator is running. Current throttle setting will be saved and reloaded on RESTART if the engine is running. Holding both buttons on the generator till the end of "sensor restart" sequence will trigger the same soft restart.

There is a small display on the generator showing basic status and current networ name (SSID) and current IP-address. By default the generator will try to connect o a local WiFi with stored credentials. If connecting to local WiFi fails it will set up its own Wifi Access Point called GENERATOR. Credentials for local wifi can be set up on the preference page by connecting to the network and IP address shown in the display.

Pressing the CHOKE button on the generator when turned on and holding until CHOKE light is lit, will skip connecting to local WiFi and set up its own Access Point. The generator will also set up an Access Point if connecting to local WiFi fails.

Pressing the STOP and CHOKE button at the same time will force the generator to run a simple self-diagonse and restart its sensors. 

Pressing the STOP button at any time (also when not running) will short out the ignition and stop engine. A red light on the generator indicate STOP button is engaged. Pressing the STOP button again will disengage the "stopped" mode. 
A realay is driven by pin 14 through a level shifter to 5 volt, and connected to the ignition (shortcircuit) wire. Whenever STOP-light is on solid the relay is activated and ignition is grounded. 

Since buttons and web inteface are not directly controlling the engine computer, but rather putting factors into its conrol cycle, there may be up to one second delay between input and confitmation of input. Especially buttons on the generator should be pressed for at least half a second to register.

The process is not using threading (multitasking). It is controlling a physical engine with inherent inertia. Adjustments of throttle and choke is not absolutely time critical, neighter is updating display and web interface.

Lights on generator have several fault modes with both lights blinking:
Both lights on with short cut-off every second:      Display could not be initialized.
Both lights giving short burst every second:         Fault in a voltage sensor. Error in display.
Both light blinking on and of every second:          Filesystem could not be initialized. Error in display.
Both light alernating on and of every second:        Emergency stop. Error in display.
Lights givig short burst in succession every second: Voltage sensors are resetting. Message in display.
                                                     Continuing to press both buttons will soft restart the controller (se restart button above).





Tested on
ESP32-D0WD-V3 (revision 3) Chip
Features: Wi-Fi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: f4:65:0b:47:e1:08
--------------------------------------------------------------------------------
*/


#include <Wire.h>
#include <INA226_WE.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DEBUG_ON 1 
// 0 will remove all debug output from compiled code

#if DEBUG_ON == 1
#define debug(x) Serial.print(x)
#define debugf(x,y) Serial.printf(x,y)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugf(x,y)
#define debugln(x)
#endif

// defining fixed stuff to reduce number of program varables
#define I2C_ADDRESS_GEN 0x41
#define I2C_ADDRESS_OUT 0x40
#define I2C_ADDRESS_BAT 0x44
#define I2C_ADDRESS_SCREEN 0x3C
#define SERVO_MIN_US 550
#define SERVO_MAX_US 2350
#define SERVO_HERTZ 50
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define READY_LIGHT_OUT 5
#define MOTOR_STOP_OUT 4
#define ENG_IGNITION_OUT 14

#define ENG_SERVO_PIN 25 
#define ENG_CHOKE_PIN 26
#define RPM_PULSE_IN 27 
#define BAT_CHARGE_IN 32
#define BAT_READY_IN 33
#define BUTTON_01_IN 34 
#define BUTTON_02_IN 35

#define SERVO_FALLBACK_PCT 80
#define SERVO_FALLBACK_DEG 144

#define ENG_FALLOFF_RPM 500 // cut off at falling RPM
#define ENG_RUNNING_RPM 1500 // RPM deemed running
#define ENG_CHOKEOFF_RPM 3000 // RPM to start retarding CHOKE
#define ENG_OPERARION_RPM 4500 // RPM at which CHOKE must off

#define GEN_TARGET_VOLT 20.0
#define GEN_TARGET_SLACK 1.0
#define BAT_TARGET_VOLT 1280 // max battery volt * 100
#define LOOP_WEB_DELAY 500
#define LOOP_ENG_DELAY 2000
#define LOOP_BAT_DELAY 300000 // 5 minutes
#define LOOP_ENG_TIME 3200

#define LOGO_HEIGHT   50
#define LOGO_WIDTH    50

/* 'generator-icon', 50x50px 
    Generated using https://mischianti.org/images-to-byte-array-online-converter-cpp-arduino/ from SVG-file 
    with output setting 'Arduino code', 'Add size to array', 'Use Identifier' and 'Horisontal - 1 bit per pixel'
    */
const unsigned char logo_bitmap [350] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x7f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 
	0xf8, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x00, 
	0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x01, 
	0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x07, 0xff, 0xfc, 
	0x00, 0x7f, 0xf8, 0x00, 0x07, 0xff, 0xfc, 0x00, 0xff, 0xf8, 0x00, 0x0f, 0xff, 0xf8, 0x00, 0xff, 
	0xfc, 0x00, 0x1f, 0xff, 0xf8, 0x01, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xf8, 0x03, 0xff, 0xfe, 0x00, 
	0x1f, 0xff, 0xf0, 0x07, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xf0, 0x07, 0xff, 0xff, 0x00, 0x3f, 0xff, 
	0xe0, 0x0f, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xe0, 0x1f, 
	0xff, 0xff, 0x80, 0x7f, 0xff, 0xc0, 0x00, 0x7f, 0xff, 0x80, 0x7f, 0xff, 0xc0, 0x00, 0xff, 0xff, 
	0x80, 0x7f, 0xff, 0xc0, 0x01, 0xff, 0xff, 0x80, 0x7f, 0xff, 0x80, 0x03, 0xff, 0xff, 0x80, 0x7f, 
	0xff, 0x80, 0x07, 0xff, 0xff, 0x80, 0x7f, 0xff, 0x00, 0x07, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xfc, 
	0x0f, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xfc, 0x1f, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xf8, 0x3f, 0xff, 
	0xff, 0x80, 0x3f, 0xff, 0xf8, 0x7f, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xf8, 0xff, 0xff, 0xff, 0x00, 
	0x3f, 0xff, 0xf1, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xff, 0xf1, 0xff, 0xff, 0xfe, 0x00, 0x1f, 0xff, 
	0xe3, 0xff, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xe7, 0xff, 0xff, 0xfe, 0x00, 0x0f, 0xff, 0xcf, 0xff, 
	0xff, 0xfc, 0x00, 0x07, 0xff, 0xdf, 0xff, 0xff, 0xf8, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xf8, 
	0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 
	0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x3f, 0xff, 
	0xff, 0xff, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xf8, 
	0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


/* network credentials are kept in strings to easily move them around and store and read from file.
   must be converted to char before use with WiFi.begin and WiFi.softAPIP.
   char *cstr = str.data(); OR const char *cstr = str.c_str();
   I HATE C++ @!#*!!
   */
// Access Point:
String assid     = "GENERATOR";
String apassword = "generator";  //can be NULL for no password
// Default WIFI CLIENT. Can be overwritten from prefs file.
String cssid     = "localwifi";
String cpassword = "localpassord";
bool wifiConnect = false; // if connected as WiFi client
// config-files to be stored in SPIFFS flash memory
const char* config_file = "/config.txt";
const char* temp_file   = "/temp.txt";
bool was_sw_reset = false;

// generator variables are integers for easy transport via WebSockets to javascript
// bracketed comment numbers shows 'sens_vals_8[8]' and 'welcome_vals_12[12]' array number
int engRpm      = 0;  // [0][0]
int realEngRpm  = 0;  // [7][11] temporary actual RPM read from hall effect sensor
int engRun      = 1;  // [1][1]  motor is running, e.g. RPM > 2000;
int engStop     = 0;  // [-][4]  if motor is stopped
int engPct      = SERVO_FALLBACK_PCT; // [-][5] manual motor setting
int engChoke    = 0;  // [-][2]  choke off (0) on (1);
int engChokePct = SERVO_FALLBACK_PCT; // [-][3] manual choke setting
int voltGen     = 0;  // [2][6]  voltage * 10 for WebSocket
int voltOut     = 0;  // [3][7]  voltage * 10 for WebSocket
int voltBat     = 0;  // [4][8]  voltage * 10 for WebSocket
int batStat     = 0;  // [5][9]  0=not carging, 1=charging, 2=full, 3=fault
int batPct      = 0;  // [6][10] battery percentage calculated from BAT_TARGET_VOLT ( /10 )
int auxStat     = 0;  // [-] extra channel // temporary used by actual RPM read

// measurements from generator
float targetVolt  = GEN_TARGET_VOLT; // target voltage for 3-phase generator output
float targetSlack = GEN_TARGET_SLACK;  // dead-zone around target voltage
float genBusVolt  = 0.0;  // voltage read by INA226 sensor from dynamo
float outBusVolt  = 0.0;  // output voltage read from 12 volt voltage regulator
float batBusVolt  = 0.0;

int lastRpm = 0;
int lastBatStat = 0;
int webInterval = LOOP_WEB_DELAY;  // interval in milliseconds for WebSockets update
unsigned long batInterval = LOOP_BAT_DELAY;
unsigned long previousWebNow = 0;
unsigned long previousServoNow = 0;
unsigned long perviousBatCheck = 0;
unsigned long batPulseDur = 0; // duration of pulse from battery pack (on fault)
unsigned long rpmPulseDur = 0; // duration of one revolution of the engine

int engServoStep  = 0; // steps in degrees for servo
int engLoopTime   = LOOP_ENG_TIME; // servo update time is engLoopTime - engLoopDelay
int engLoopDelay  = LOOP_ENG_DELAY; // variable loop delay defined by difference in volt vs target volt
int engServoPos   = SERVO_FALLBACK_DEG; // initial engine servo position in degrees (80%  = ±144°)

bool button01State     = HIGH;;
bool lastButton01State = HIGH; // for debounce
bool button02State     = HIGH;
bool lastButton02State = HIGH; // for debounce

// arrays to send to client
String sens_vals_8[8];       // 8 vals to be sendt at webIntervals
String welcome_vals_12[12];  // complete set of vals for init

Servo motorServo; // ESP32Servo
Servo chokeServo; // ESP32Servo
ESP32PWM pwm;     // Extended ESP32 PWM protocol
INA226_WE ina226_gen = INA226_WE(I2C_ADDRESS_GEN);  // dynamo voltage
INA226_WE ina226_out = INA226_WE(I2C_ADDRESS_OUT);  // output voltage
INA226_WE ina226_bat = INA226_WE(I2C_ADDRESS_BAT);  // battery voltage
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Adafruit SSD1306 display

AsyncWebServer server(80);  // using ESPAsyncWebServer @ port 80
WebSocketsServer webSocket = WebSocketsServer(81); // @ port 81

void setup() {
  // Allocate two timers for Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  Serial.begin(115200); // start serial debug communication
  Wire.begin();  // start I2C communicatiom

  // setup pins for buttons, inputs and lights
  pinMode(BUTTON_01_IN, INPUT); // no internal pullup
  pinMode(BUTTON_02_IN, INPUT); // no internal pullup
  pinMode(RPM_PULSE_IN, INPUT_PULLUP);
  pinMode(BAT_CHARGE_IN, INPUT_PULLUP);
  pinMode(BAT_READY_IN, INPUT_PULLUP);
  pinMode(MOTOR_STOP_OUT, OUTPUT);
  digitalWrite(MOTOR_STOP_OUT, LOW);
  pinMode(ENG_IGNITION_OUT, OUTPUT);
  digitalWrite(ENG_IGNITION_OUT, LOW);
  pinMode(READY_LIGHT_OUT, OUTPUT);
  digitalWrite(READY_LIGHT_OUT, LOW);

  // Adafruit SSD1306 OLED-display
  if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS_SCREEN)) {
    debugln(F("SSD1306 screen allocation failed"));
    while (1) {
      //no display so just blinking
      digitalWrite(MOTOR_STOP_OUT, HIGH);
      digitalWrite(READY_LIGHT_OUT, HIGH);
      delay(900);
      digitalWrite(READY_LIGHT_OUT, LOW);
      digitalWrite(MOTOR_STOP_OUT, LOW);
      delay(100);
    }
  } else {
    display.clearDisplay();
    display.drawBitmap( // display startup logo
      (display.width()  - LOGO_WIDTH ) / 2,
      (display.height() - LOGO_HEIGHT) / 2,
      logo_bitmap, LOGO_WIDTH, LOGO_HEIGHT, WHITE);
    display.display();
  }
  if (!ina226_gen.init()) {  // try to initiate INA226 sensor for dynamo
    debugln(F("Failed to init generator voltage sensor."));
    stopAlert ("Dynamo sensor fault!", "Check connections.", 100, 900);
  }
  if (!ina226_out.init()) {  // try to initiate INA226 sensor for output
    debugln(F("Failed to init output voltage sensor."));
    stopAlert ("Output sensor fault!", "Check connections.", 100, 900);
  }
  if (!ina226_bat.init()) {  // try to initiate INA226 sensor for battery
    Serial.println(F("Failed to init battery voltage sensor."));
    stopAlert ("Battery sensor fault!", "Check connections.", 100, 900);
  }
  // set prefs for all INA226 sensors
  ina226_gen.setAverage(AVERAGE_128);
  ina226_gen.setConversionTime(CONV_TIME_8244);
  ina226_gen.waitUntilConversionCompleted();
  ina226_out.setAverage(AVERAGE_128);
  ina226_out.setConversionTime(CONV_TIME_8244);
  ina226_out.waitUntilConversionCompleted();
  ina226_bat.setAverage(AVERAGE_128);
  ina226_bat.setConversionTime(CONV_TIME_8244);
  ina226_bat.waitUntilConversionCompleted();

  // internal SPIFFS flash memory filesystem
  if (!SPIFFS.begin()) {
    debugln(F("SPIFFS could not initialize"));
    stopAlert ("FS couldn't initiate.", "YOU ARE FUCKED!", 500, 500);
  }

  button02State = digitalRead(BUTTON_02_IN);
  if (button02State == HIGH) {

    /* Set up wifi as wifi client (STA) mode when button 2 is HIGH (not pressed).
       and then in Access Point (AP) mode if fail.  
       */

    // try to load prefs from config-file
    readPrefs(SPIFFS, config_file);
    const char *ssid = cssid.c_str();
    const char *password = cpassword.c_str();

    debugln("SSID: " + cssid);
    debugln("Password: " + cpassword);

    WiFi.begin(ssid, password);  // start WiFi interface
    debugln("Establishing connection to WiFi with SSID: " + String(ssid));
    screenPrintAt ("Connecting to WiFi", 1, 0, 0, true);
    
    display.setCursor(0, 11);
    for (int c = 0; c < 20;) {
      delay(1000);
      debug(".");
      display.print(".");
      display.display();
      if (WiFi.status() != WL_CONNECTED) {
        c++;
      } else {
        c = 20;
      }
    }
    if (WiFi.status() != WL_CONNECTED) {
      debugln("");
      debugln(F("WiFi connection failed!"));
      debug(F("Trying to set up WiFi Access Point."));
      screenPrintAt ("Failed to connect!", 1, 0, 0, true);
      screenPrintAt ("Setting up AP mode.", 1, 0, 11, false);
      delay(2000);
      wifiConnect = false;
    } else {
      debugln("");
      debug(F("Connected to network with IP address: "));
      debugln(WiFi.localIP());
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 47);
      display.print("SSID: " + String(ssid));
      display.setCursor(0, 57);
      display.print("IP:   ");
      display.print(WiFi.localIP());
      display.display();
      wifiConnect = true;
    }
  } 
  
  if (wifiConnect == false || button02State == LOW) {

    /* Set up wifi in Access Point (AP) mode when button 2 is LOW (pressed),
       or if wifi client mode failed.
       */

    // WiFi setup
    const char *ssid = assid.c_str();
    const char *password = apassword.c_str();
    IPAddress local_IP(192, 168, 1, 2);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);

    // Access Point setup
    debug("Setting up Access Point ... ");
    screenPrintAt ("Setting up AP: ", 1, 0, 0, true);
    if (WiFi.softAPConfig(local_IP, gateway, subnet)) {
      debugln(F("Ready."));
      display.print("Ready.");
      display.display();
    } else {
      debugln(F("Failed!"));
      display.print("Failed!");
      display.display();
      while(1);
    }
    
    debug(F("Starting Access Point ... "));
    screenPrintAt ("Starting AP: ", 1, 0, 11, false);
    if (!WiFi.softAP(ssid, password)) {
      debugln(F("Failed!"));
      display.print("Failed!");
      display.display();
      while (1);
    } else {
      debugln(F("Ready."));
      display.print("Ready.");
      display.display();
      debugln("Wifi network: " + String(ssid));
      debug(F("IP address: "));
      debugln(WiFi.softAPIP());
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 47);
      display.print("SSID: " + String(ssid));
      display.setCursor(0, 57);
      display.print("IP:   ");
      display.print(WiFi.softAPIP());
      display.display();
    }
    button02State == HIGH;
  }

  // webserver setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html"); // initial page
  });
  server.on("/net", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/network.html", "text/html"); //setup page
  });
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "File not found"); // file not found
  });
  server.serveStatic("/", SPIFFS, "/"); // set SPIFFS as webserver root

  webSocket.begin(); // websockets begin and eventlistener
  webSocket.onEvent(webSocketEvent); 
  server.begin(); // must start after websocket

  // confirm button 2 press with light
  while (digitalRead(BUTTON_02_IN) == LOW) {
    digitalWrite(READY_LIGHT_OUT, HIGH);
  }
  digitalWrite(READY_LIGHT_OUT, LOW);



  // setup servos
  /*
    
  }
 */
  motorServo.setPeriodHertz(SERVO_HERTZ);  // Standard 50hz servo
  chokeServo.setPeriodHertz(SERVO_HERTZ);
  motorServo.attach(ENG_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  chokeServo.attach(ENG_CHOKE_PIN, SERVO_MIN_US, SERVO_MAX_US);

  /* Check for saved state of throttle servo if engine is running.
     Clean up and delete saved throttle file.
     */

  if (esp_reset_reason() == 3 || esp_reset_reason() == 12) {
    //1: ESP_RST_POWERON, 3: ESP_RST_SW, 5: ESP_RST_DEEPSLEEP, 12: SW_CPU_RESET, 15: RTCWDT_BROWN_OUT_RESET
    was_sw_reset = true;
  }
  rpmPulseDur = pulseIn(RPM_PULSE_IN, HIGH, 200000); 

  if (rpmPulseDur == 0 || !was_sw_reset) {
    engRun = 0;
    engStop = 1;
    motorServo.write(map(engPct, 0, 100, 0, 180));
    chokeServo.write(map(engChokePct, 0, 100, 0, 180));
    deleteFile(SPIFFS, temp_file);
  } else {
    engRun = 1;
    engStop = 0;
    readMotor(SPIFFS, temp_file);
    deleteFile(SPIFFS, temp_file);
  }
}

void loop() {
  // ESPAsyncWebServer runs its own loop and don't need to be called
  webSocket.loop(); // websockets loop call

  
  /* Services
     Handeling pressed buttons on the generator, battery status, updating display on
     generator and sending updates via WebSocket to clients connected through WiFi. 
     Returns from web clients are handled by WebSocketEvents.
    */
  unsigned long webNow = millis();
  if ((unsigned long)(webNow - previousWebNow) > webInterval) {
    /* SERVICE LOOP */

    voltGen = floor(genBusVolt * 10);  // int (volt * 10) for websocket
    voltOut = floor(outBusVolt * 10);  // int (volt * 10) for websocket 
    batPct  = map(floor(batBusVolt * 100), 900, BAT_TARGET_VOLT, 0, 100);
    batPct = constrain(batPct, 0, 110);

    if (batStat != 3) {
      if (outBusVolt < 9) {
        batStat = 0; // running on battery
      } else if (digitalRead(BAT_READY_IN) == LOW || (digitalRead(BAT_READY_IN) == HIGH && digitalRead(BAT_CHARGE_IN) == HIGH)) {
        if (batPct < 85) {
          batStat = 1;
        } else {
          batStat = 2; // battery full
        }
      } else {
        batStat = 1; // charging
      }
    }
   
    
    if (batStat != lastBatStat || ((unsigned long)(webNow - perviousBatCheck) > batInterval)) { 
      // check battery only if battery status changes or batInterval
      batPulseDur = pulseIn(BAT_READY_IN, LOW, 5000000);
      if (batPulseDur != 0) {
        debugln(F("Battery fault!"));
        batStat = 3; // battery falt
      } 
      perviousBatCheck = webNow;
    }
    lastBatStat = batStat; 

    // -------- DEMO DEMO DEMO --------
    // demo values based on voltage read from INA226_gen
    // temporary replaces RPM from hall effects sensor
    engRpm = floor((voltGen - 30) * 32.1);
    if (engRpm < 1) {
      engRpm = 0;
    }
    // -------- DEMO DEMO DEMO --------

    ////////TO BE IMPLEMENTED/////////
    /* RPM from hall effect sensor : Replaces DEMO */
    rpmPulseDur = pulseIn(RPM_PULSE_IN, HIGH, 200000); 
    realEngRpm = floor(60000.0/rpmPulseDur*1000);
    if (realEngRpm > 9999) realEngRpm = 0; // rule out faulty read at low RPM
    //debugln("PD: " + String(rpmPulseDur) + " RPM: " + String(realEngRpm));
    ////////TO BE IMPLEMENTED/////////

    if (engChoke == 1) { 
      if ((engRpm > ENG_CHOKEOFF_RPM && engRpm > lastRpm) || engRpm > ENG_OPERARION_RPM) {
        // disengage choke above ENG_CHOKEOFF_RPM and rising RPM, or above ENG_OPERARION_RPM
        sendJson("CHOKE", "0"); // close choke panel in web interface
        if (engChokePct > 15) {
          engChokePct = engChokePct - int((engRpm - ENG_CHOKEOFF_RPM) * 0.0075); // ease off choke
          sendJson("CSET", String(engChokePct));
          digitalWrite(READY_LIGHT_OUT, HIGH);
        } else {
          engChoke = 0;
          engChokePct = 0;
          sendJson("CSET", String(engChokePct));
          digitalWrite(READY_LIGHT_OUT, LOW);
        }
      }
      chokeServo.write(map(engChokePct, 0, 100, 0, 180));
      digitalWrite(READY_LIGHT_OUT, HIGH);
    } else {
      // set choke off
      chokeServo.write(0);
      digitalWrite(READY_LIGHT_OUT, LOW);
    }

    if (engRpm <= ENG_FALLOFF_RPM && lastRpm > ENG_FALLOFF_RPM && engRun != 0) {
      // engine STOP below ENG_FALLOFF_RPM and falling RPM, if engine has been above ENG_RUNNING_RPM
      debugln(F("Stopped due to low RPM"));
      emergency_stop("Low RPM!");
    }
    if (engRpm > ENG_OPERARION_RPM && genBusVolt < 5) {
      debugln(F("Stopped due to low voltage"));
      emergency_stop("Low voltage!");
    }
    if (genBusVolt > 28) {
      debugln(F("Stopped due to high voltage"));
      emergency_stop("High voltage!");      
    }
    if (engRpm > ENG_RUNNING_RPM && engRpm > lastRpm) {
      engRun = 1; 
    }
    lastRpm = engRpm;


    /* Update display on generator using Adafruit SSD1306 OLED-display
       Display require char* arrays, some conversion is needed
       */
    char outVoltChar[5]; //display require char* ouput
    dtostrf(outBusVolt, 4, 1, outVoltChar); // converting float into XX.X* char*
    char genVoltChar[5];  //display require char* ouput
    dtostrf(genBusVolt, 4, 1, genVoltChar); // converting float into XX.X* char*
    char rpmChar[5];  //display require char* ouput
    String stringRpm = String(engRpm); // long way home.. via string
    stringRpm.toCharArray(rpmChar, 5); // string to char*
    char bpctChar[4];
    String stringBatpct = String(batPct); // long way home.. via string
    stringBatpct.toCharArray(bpctChar, 4);

    // -------- DEMO DEMO DEMO --------
    char rrpmChar[5];
    String stringRrpm = String(realEngRpm); // long way home.. via string
    stringRrpm.toCharArray(rrpmChar, 5); // string to char* 
    // -------- DEMO DEMO DEMO --------
   
    display.fillRect(0, 0, 128, 46, BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.write(outVoltChar);
    display.write(" Volt");
    display.setTextSize(1);
    display.setCursor(0, 17);
    display.write(genVoltChar);
    display.write("V @ ");
    if (realEngRpm != 0) {
      display.write(rrpmChar);
      display.write("*");
    } else {
      display.write(rpmChar);
    }
    display.write(" RPM");
    display.setCursor(0, 27);
    if (batStat == 1) {
      display.write("Charging: ");
      display.write(bpctChar);
      display.write("%");
    } else if (batStat == 2) {
      display.write("Fully charged");
    } else if (batStat == 3) {
      display.write("Battery fault!");
    } else {
      display.write("On battery: ");
      display.write(bpctChar);
      display.write("%");
    }
    
    display.display();


    /* Handle button press on the generator
       There are two buttons, STOP and CHOKE. Buttons are momentary buttons, 
       but a button press toggles states on and off and status is indicated with LED-lights
       Pressing both buttons invoke a simple internal self service and both lights
       blink for a few seconds.
       Buttons are debounced with an extra 'webInterval' loop.
       */
    button01State = digitalRead(BUTTON_01_IN);
    button02State = digitalRead(BUTTON_02_IN);
    if ((button01State == LOW && lastButton01State == LOW) && (button02State == LOW && lastButton02State == LOW)) {
      self_service();
    }
    if (button01State == LOW && lastButton01State == LOW && button02State == HIGH) {
      debugln(F("Button 1 pressed"));
      if (engStop == 0) {
        engStop = 1;
        digitalWrite(MOTOR_STOP_OUT, HIGH);
        digitalWrite(ENG_IGNITION_OUT, HIGH);
      } else {
        engStop = 0;
        digitalWrite(MOTOR_STOP_OUT, LOW);
        digitalWrite(ENG_IGNITION_OUT, LOW);
      }
      sendJson("STOP", String(engStop));
      lastButton01State = HIGH;
    } else {
      lastButton01State = button01State;
    }
    if (button02State == LOW && lastButton02State == LOW && button01State == HIGH) {
      debugln(F("Button 2 pressed"));
      if (engRpm < ENG_CHOKEOFF_RPM) {
        if (engChoke == 0) {
          engChoke = 1;
          engChokePct = SERVO_FALLBACK_PCT;
          sendJson("CSET", String(engChokePct));
          digitalWrite(READY_LIGHT_OUT, HIGH);
        } else {
          engChoke = 0;
          digitalWrite(READY_LIGHT_OUT, LOW);
        }
        sendJson("CHOKE", String(engChoke));
        lastButton02State = HIGH;
      } 
    } else {
      lastButton02State = button02State;
    }

    /* Sending values to web clients through Web Sockets
       Values are broadcast as as a nested array with value "type"="sensors" and
       an array of values "value" = array[8];
       */
    sens_vals_8[0] = engRpm;
    sens_vals_8[1] = engRun;
    sens_vals_8[2] = voltGen;
    sens_vals_8[3] = voltOut;
    sens_vals_8[4] = voltBat;
    sens_vals_8[5] = batStat;
    sens_vals_8[6] = batPct;
    sens_vals_8[7] = realEngRpm;
    sendJsonArray("sensors", sens_vals_8, 8);

    previousWebNow = webNow;
  }

  /* Engine throttle control
     Throttle control runs on a variable interval whitch length is determined
     by the difference between the dynamo output and the target voltage.
     In essence a larger difference makes for bigger steps and more frequent update.
    */
  unsigned long servoNow = millis();
  if ((unsigned long)(servoNow - previousServoNow) > engLoopDelay) {
    /* ENGINE LOOP */

    // get voltages from dynamo, battery and output, and multipy with calibration
    genBusVolt = ina226_gen.getBusVoltage_V() * 1.025;
    outBusVolt = ina226_out.getBusVoltage_V() * 1.025;
    batBusVolt = ina226_bat.getBusVoltage_V() * 1.025;
    if (engStop == 1) {
      digitalWrite(MOTOR_STOP_OUT, HIGH);
      digitalWrite(ENG_IGNITION_OUT, HIGH);
    } else {
      digitalWrite(MOTOR_STOP_OUT, LOW);
      digitalWrite(ENG_IGNITION_OUT, LOW);
    }
    if (engChoke == 1) {
      engServoPos = map(engPct, 0, 100, 0, 180);
    } else if (engStop == 0 && genBusVolt >= 5) {
      // can not calculate engServoPos for <5 volt.
      engServoStep = abs(targetVolt - int(trunc(genBusVolt)));
      if (genBusVolt > (targetVolt + targetSlack)) {
        engServoPos -= engServoStep;
      } else if (genBusVolt < (targetVolt - targetSlack)) {
        engServoPos += engServoStep;
      }
      engLoopDelay = engLoopTime - (engServoStep * 200);
      engServoPos = constrain(engServoPos, 0, 180);
    } else if (genBusVolt < 5) {
      // set throttle to 80% if voltage drops below 5 volt
      // fault in voltage system will be handled in SERVICE LOOP 
      engServoPos = SERVO_FALLBACK_DEG;  // 144° (deg) = 80%
    }
    motorServo.write(engServoPos);

    previousServoNow = servoNow;
  }
}

// on reception of data from client via WebSocket
void webSocketEvent(byte num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {  // switch on the type of information received
    case WStype_DISCONNECTED:
      debugln("Client " + String(num) + " disconnected");
      break;
    case WStype_CONNECTED:
      // send full set of vals to new client
      debugln("Client " + String(num) + " connected");
      debugln("Sending values to client " + String(num));
      welcome_vals_12[0] = engRpm;
      welcome_vals_12[1] = engRun;
      welcome_vals_12[2] = engChoke;
      welcome_vals_12[3] = engChokePct;
      welcome_vals_12[4] = engStop;
      welcome_vals_12[5] = engPct;
      welcome_vals_12[6] = voltGen;
      welcome_vals_12[7] = voltOut;
      welcome_vals_12[8] = voltBat;
      welcome_vals_12[9] = batStat;
      welcome_vals_12[10] = batPct;
      welcome_vals_12[11] = realEngRpm;
      sendJsonArray("welcome", welcome_vals_12, 12);
      break;
    case WStype_TEXT:  // if a client has sent data, then type == WStype_TEXT
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        debug(F("deserializeJson() failed: "));
        debugln(error.f_str());
        return;
      } else {
        // JSON string was received correctly, so information can be retrieved:
        const char *l_type = doc["type"];
        const int l_value = doc["value"];
        if (String(l_type) == "CHOKE") {
          // all handeling of CHOKE should be done  by loop
          // toggeling 'engChoke' on received CHOKE 
          engChoke = (engChoke == 1) ? 0 : 1;
          sendJson("CHOKE", String(engChoke));
          debugln("CHOKE: " + String(engChoke));
        }
        if (String(l_type) == "STOP") {
          // all handeling of STOP should be done by loop
          // toggeling 'engStop' on received STOP 
          engStop = (engStop == 1) ? 0 : 1;
          sendJson("STOP", String(engStop));
          debugln("STOP: " + String(engStop));
        }
        if (String(l_type) == "CSET") {
          // all handeling of CHOKE should be done  by loop
          // storing received value in 'engChokePct'
          const int l_value = doc["value"];
          engChokePct = int(l_value);
          sendJson("CSET", String(engChokePct));
          debugln("CSET: " + String(engChokePct));
        }
        if (String(l_type) == "MSET") {
          // all handeling of throttle should be done  by loop
          // storing received value in 'engPct'
          const int l_value = doc["value"];
          engPct = int(l_value);
          sendJson("MSET", String(engPct));
          debugln("MSET: " + String(engPct));
        }
        if (String(l_type) == "SAVE") {
          // saving wifi credentials received from web client
          String l_ssid = doc["ssid"];
          String l_pswd = doc["pswd"];
          listDir(SPIFFS, "/", 0);
          debugln("");
          deleteFile(SPIFFS, config_file);
          File file = SPIFFS.open(config_file, FILE_WRITE);
          if (!file) {
            debugln(F("Failed to create file"));
            sendJson("FILE", String("file_0"));  // Failed to save preferences!
          } else {
            JsonDocument doc_fw;
            doc_fw["essid"] = l_ssid;
            doc_fw["epswd"] = l_pswd;
            if (serializeJson(doc_fw, file) == 0) {
              debugln(F("Failed to write to file"));
              sendJson("FILE", String("file_1"));  // Failed to write to preference file!
            } else {
              debugln(F("Preferences saved"));
              sendJson("FILE", String("file_2"));  // Preferences saved.
            }
            file.close();
          }
          readFile(SPIFFS, config_file);
          debugln("");
          listDir(SPIFFS, "/", 0);
        }
        if (String(l_type) == "DELETE") {
          // deleting wifi credentials file from SPIFFS flash memory
          listDir(SPIFFS, "/", 0);
          debugln("");
          deleteFile(SPIFFS, config_file);
          File file = SPIFFS.open(config_file);
          if (!file) {
            sendJson("FILE", String("file_3"));  //Could not delete preference file!
          } else {
            sendJson("FILE", String("file_4"));  // Preferences deleted.
            file.close();
          }
          debugln("");
          listDir(SPIFFS, "/", 0);
        }
        if (String(l_type) == "SCAN") {
          // scanning for local wifi networks and broadcasting a list (upon request)
          int n = WiFi.scanNetworks();
          String jsonString = "";
          JsonDocument doc;
          JsonObject object = doc.to<JsonObject>();
          object["type"] = "SCAN";
          JsonArray value = object["value"].to<JsonArray>();
          for (int i = 0; i < n; i++) {
            value.add(WiFi.SSID(i));
          }
          serializeJson(doc, jsonString);
          webSocket.broadcastTXT(jsonString);
        }
        if (String(l_type) == "CURRENT") {
          // broadcasting information about current WiFi (upon request)
          String jsonString = "";
          JsonDocument doc;
          JsonObject object = doc.to<JsonObject>();
          object["type"] = "CURRENT";
          JsonArray value = object["value"].to<JsonArray>();
          if (WiFi.status() == WL_CONNECTED) {
            value.add(WiFi.SSID());
            value.add(WiFi.localIP());
          } else {
            value.add(assid);
            value.add(WiFi.softAPIP());
          }
          File file = SPIFFS.open(config_file);
          JsonDocument doc_fr;
          DeserializationError error = deserializeJson(doc_fr, file);
          if (error) {
            value.add(String("0"));
            value.add(String("0"));
            Serial.write("Failed to load document");
          } else {
            value.add(String(doc_fr["essid"]));
            value.add(String(doc_fr["epswd"]));
          }
          file.close();
          serializeJson(doc, jsonString);      // convert JSON object to string
          webSocket.broadcastTXT(jsonString);  // send JSON string to all clients
        }
        if (String(l_type) == "RESTART") {
          soft_reset();
        }
      }
      debugln("");
      break;
  }
}

/* filesystem operators.
  USAGE: 
  listDir(SPIFFS, "/", 0);
  readFile(SPIFFS, "/hello.txt");
  deleteFile(SPIFFS, "/foo.txt");
*/

void readFile(fs::FS &fs, const char *path) {
  debugf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    debugln(F("- failed to open file for reading"));
    return;
  }
  debugln(F("- read from file:"));
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}
void readPrefs(fs::FS &fs, const char *path) {
    File config = fs.open(path);
    JsonDocument doc_fr;
    DeserializationError error = deserializeJson(doc_fr, config);
    if (error) {
      debugln(F("Failed to read file, using default configuration"));
    } else {
      debugln(F("Loaded configuration from file..."));
      cssid = String(doc_fr["essid"]);
      cpassword = String(doc_fr["epswd"]);
    }
    config.close();
}
void readMotor(fs::FS &fs, const char *path) {
    File config = fs.open(path);
    JsonDocument doc_fr;
    DeserializationError error = deserializeJson(doc_fr, config);
    if (error) {
      debugln(F("Failed to read file, using default throttle"));
    } else {
      debug(F("Loaded throttle from file... "));
      String eng = String(doc_fr["motor"]);
      engServoPos = eng.toInt();
      debugln(engServoPos);
    }
    config.close();

}
void deleteFile(fs::FS &fs, const char *path) {
  debugf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    debugln(F("- file deleted"));
  } else {
    debugln(F("- delete failed"));
  }
}
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  debugf("Listing directory: %s\r\n", dirname);
  File root = fs.open(dirname);
  if (!root) {
    debugln(F("- failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    debugln(F(" - not a directory"));
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      debug("  DIR : ");
      debugln(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      debug("  FILE: ");
      debug(file.name());
      debug("\tSIZE: ");
      debugln(file.size());
    }
    file = root.openNextFile();
  }
}


/* WebSocket functions */
// Simple function to send single value to the web clients
void sendJson(String l_type, String l_value) {
  String jsonString = "";
  JsonDocument doc;
  JsonObject object = doc.to<JsonObject>();
  object["type"] = l_type;
  object["value"] = l_value;
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
}
// Simple function to send array to the web clients
void sendJsonArray(String l_type, String l_array_values[], int a_length) {
  String jsonString = "";
  //const size_t CAPACITY = JSON_ARRAY_SIZE(l_length) + 100;
  JsonDocument doc;

  JsonObject object = doc.to<JsonObject>();
  object["type"] = l_type;
  JsonArray value = object["value"].to<JsonArray>();
  for (int i = 0; i < a_length; i++) {
    value.add(l_array_values[i]);
  }
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
}


/* Service functions */
void self_service () {
  restart_sensors ();
  //if buttons are still pressed
  if (digitalRead(BUTTON_01_IN) == LOW && digitalRead(BUTTON_02_IN) == LOW) {
    soft_reset();
  }
}
void emergency_stop (String str) {
  engRun = 0;  
  engStop = 1;
  engPct = 0;
  engServoPos = 0;
  motorServo.write(engServoPos);
  digitalWrite(MOTOR_STOP_OUT, HIGH);
  digitalWrite(ENG_IGNITION_OUT, HIGH);
  sendJson("STOP", String(engStop));
  delay(1000);
  sendJson("message", "EMERGENCY STOP:  " + str);
  screenPrintAt ("EMERGENCY STOP", 2, 0, 0, true);
  screenPrintAt (str.c_str(), 1, 0, 40, false);
  while(1) {
    digitalWrite(MOTOR_STOP_OUT, HIGH);
    digitalWrite(READY_LIGHT_OUT, LOW);
    delay(500);
    digitalWrite(READY_LIGHT_OUT, HIGH);
    digitalWrite(MOTOR_STOP_OUT, LOW);
    delay(500);
  }
}
void restart_sensors () {
  // restart of INA226 voltage sensors and zero battery fault
  ina226_gen.powerDown();
  ina226_out.powerDown();
  ina226_bat.powerDown();
  display.fillRect(0, 27, 128, 9, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(0, 27);
  display.write("Resetting sensors!");
  display.display();
  debugln(F("Resetting sensors!"));
  sendJson("message", "Sensors was reset!");
  pause_blipblop_blink(9);
  ina226_gen.powerUp();
  ina226_out.powerUp();
  ina226_bat.powerUp();
  batStat = 0;
  lastButton01State = HIGH;
  lastButton02State = HIGH;
} 
void soft_reset() {
  // saving throttle settings in case engine is running
  display.fillRect(0, 37, 128, 9, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(0, 37);
  display.write("Restarting...");
  display.display();
  sendJson("message", "HOLD MY BEER!<br>...I'm rebooting.");
  listDir(SPIFFS, "/", 0);
  debugln("");
  deleteFile(SPIFFS, temp_file);
  File file = SPIFFS.open(temp_file, FILE_WRITE);
  if (!file) {
    debugln(F("Failed to create file"));
  } else {
    JsonDocument doc_fw;
    doc_fw["motor"] = engServoPos;
    if (serializeJson(doc_fw, file) == 0) {
      debugln(F("Failed to write to file"));
    } else {
      debugln(F("Throttle setting saved"));
    }
    file.close();
  }
  sendJson("RELOAD", "20000");
  readFile(SPIFFS, temp_file);
  debugln("");
  listDir(SPIFFS, "/", 0);
  delay(1000);
  ESP.restart();
}
void screenPrintAt (const char* msg1, int textsize, int left, int top, bool clear) {
  if (clear) display.clearDisplay();
  display.setTextSize(textsize);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(left, top);
  display.print(msg1);
  display.display();
}
void stopAlert (const char *msg1, const char *msg2, int ontime, int offtime) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print(msg1);
    display.setCursor(0, 22);
    display.print(msg2);
    display.display();
    while(1) {
      digitalWrite(MOTOR_STOP_OUT, HIGH);
      digitalWrite(READY_LIGHT_OUT, HIGH);
      delay(ontime);
      digitalWrite(READY_LIGHT_OUT, LOW);
      digitalWrite(MOTOR_STOP_OUT, LOW);
      delay(offtime);
    }
}
// pause with both lights blinking in rapid sucession every 500 ms, 'blink' number of times.
void pause_blipblop_blink (int blink) {
  int i = blink;
  while (i > 0) {
    digitalWrite(MOTOR_STOP_OUT, HIGH);
    delay(100);
    digitalWrite(MOTOR_STOP_OUT, LOW);
    digitalWrite(READY_LIGHT_OUT, HIGH);
    delay(100);
    digitalWrite(READY_LIGHT_OUT, LOW);
    digitalWrite(MOTOR_STOP_OUT, LOW);
    delay(300);
    i--;
  }
  if (engChoke == 1) digitalWrite(READY_LIGHT_OUT, HIGH);
  if (engStop == 1) digitalWrite(MOTOR_STOP_OUT, HIGH);
}
