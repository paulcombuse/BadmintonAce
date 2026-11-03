#include <WiFi.h>              // Wifi access point configuration
#include <SPIFFS.h>            // SPIFFS library to allow web page to be uploaded as seperate document
#include <ESPAsyncWebServer.h> // Web server
#include <WebSocketsServer.h>  // Library for web-sockets to msg with front-end
#include <ArduinoJson.h>       // Needed for JSON encapsulation (send multiple variables with one string)
#include <ESP32Servo.h>        // Servo library to control ESC's and servos
#include <Preferences.h>       // Library to allow saving/loading settings data
#include <Math.h>              // Library for maths functions
#include <NewPing.h>           // Sonar libary for obstruction sensor
#include <ESPmDNS.h>           // Library to allow .local instead of IP address to access page.
#include <PNGdec.h>            // Include the PNG decoder library

// Published values for SG90 servos; adjust if needed.
// QY3225 or MS62B Digital Servo - 50-333hz 500-2500 usec
#define servoMinPulseLength 500  // Minimum pulse length in µs
#define servoMaxPulseLength 2500 // Maximum pulse length in µs

// ESC min max
#define escMinPulseLength 1000 // Minimum pulse length in µs
#define escMaxPulseLength 2500 // Maximum pulse length in µs

#define motorKV 750         // Motor KV
#define batteryVoltage 11.1 // Battery voltage for 3 cell lipo

// Recommended pins include 2,4,12-19,21-23,25-27,32-33  //not pin 2 as that's the LED on the DEV BOARD
// When using pins 34,35,36 or 39 take care that these are only inputs and need EXTERNAL pullup resistors (where necessitated, e.g. for our buttons)
#define startPin 14               // pull-up 34/14
#define stopPin 12                // pull-up 35/12
#define throughBeamPin 25         // pull-up
#define throughBeamIndicatorPin 2 // Built in LED
#define servoTrajectoryPin 16
#define servoRotationPin 17
#define servoPusherPin 32
#define servoGripperPin 33
#define servoLeftESCPin 21
#define servoRightESCPin 22
#define ultrasonicTriggerPin 27   // White Wire >>> 32?
#define ultrasonicEchoPin 39      // Grey Wire  >>> 33?
#define ultrasonicMaxDistance 100 // Maximum distance we want to measure (in centimeters). Badminton Service line is 198cm from net.

// Create a state engine variables for the feeding sequence.
#define ST_IDLE 0
#define ST_START 1
#define ST_GRIPPER_OPEN 2
#define ST_GRIPPER_CLOSED 3
#define ST_SHUTTLE_CHECK 4
#define ST_PREP_LAUNCH 5
#define ST_LAUNCH 6
#define ST_RESET 7
static byte system_state = ST_IDLE;

/////////////////////// TFT ///////////////////////////
#include <TFT_eSPI.h>
static bool LCDconnected = true;

// TFT display instance
TFT_eSPI tft = TFT_eSPI();
// Create objects for decoding PNG
PNG png;
File pngfile;
int16_t xpos = 10;
int16_t ypos = 0;
const char *pngFilename = "/TSV_ACE_Logo.png";
#define TFT_BKGND_COLOR TFT_BLACK
#define TFT_TEXT_COLOR TFT_WHITE

// This is the file name used to store the touch calibration data
// The SPIFFS file name must start with "/".
#define CALIBRATION_FILE "/TouchCalData"
#define REPEAT_CAL false // Set to true to force recalibration

// Keypad start position, key sizes and spacing
#define BTN_X 80 // Centre of key
#define BTN_Y 200
#define BTN_WIDTH 140 // Width and height
#define BTN_HEIGHT 60
#define BTN_SPACING_X 20 // X and Y gap
#define BTN_SPACING_Y 20
const uint8_t BTN_TEXTSCALING = 1; // Font size scaling factor for button labels

// Using two fonts
#define LABEL1_FONT &FreeSans9pt7b // Key label font 1
#define LABEL2_FONT &FreeMono9pt7b // Key label font 2

// We have a status line for messages
#define STATUS_WIDTH 300
#define STATUS_X 160 // Centred on this
#define STATUS_Y 130

// Create keys for the keypad
#define BTN_COUNT 8
#define BTN_ROWS 4
#define BTN_COLS 2
// char tftBtnLabel[BTN_COUNT][16] = {"Single\nServe", "Double\nServe", "Single\nFull", "Double\nFull", "Single\nNoRot", "Double\nNoRot", "Calib\nShuttle", "Calib\nWheel"};
char tftMultilineBtnLabel[BTN_COUNT][2][12] = {{"Single", "LH Service"}, {"Double", "LH Service"}, {"Single", "RH Service"}, {"Double", "RH Service"}, {"Single", "Full"}, {"Double", "Full"}, {"Custom", "User Preset"}, {"Double", "No Rotation"}};
uint16_t tftBtnPos[BTN_COUNT][2] = {};
uint16_t tftBtnColor[BTN_COUNT] = {
    TFT_DARKCYAN, TFT_BLUE,
    TFT_DARKCYAN, TFT_BLUE,
    TFT_DARKCYAN, TFT_BLUE,
    TFT_DARKGREEN, TFT_BLUE};

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button tftBtn[BTN_COUNT];

/////////////////////// TFT END //////////////////////

// Constants
const char *mDnsName = "ACE";
const char *ssid = "ACE_1";
const char *password = "Launcher"; // Password must be >=8 characters
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 81;

// global variables
Preferences preferences;

byte sequence_length = 0;
int current_sequence_index = 0;

struct shotDetails
{
  float court_x_pos;
  float court_y_pos;
  float trajectory;
  float rotation;
  float v0;
  int left_motor_rpm;
  int right_motor_rpm;
};
shotDetails active_shots[10];
int active_shot_array_length = 0;
float default_delay = 3;

struct shotEntry
{
  int ID;
  float delay;
};
shotEntry shot_sequence[50];

enum PLAY_TYPE
{
  PLAY_ONCE,
  REPEAT_SHOTS,
  RANDOM_ORDER,
  RANDOM_SHOTS
};
PLAY_TYPE ACTIVE_PLAY_TYPE = PLAY_ONCE;
static const char SinglesLHServicePreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":0.5,"y":2.5,"rotation":15,"trajectory":10,"lm_speed":2935,"rm_speed":958},{"x":2.05,"y":2.5,"rotation":15,"trajectory":10,"lm_speed":961,"rm_speed":3382},{"x":2.05,"y":6.18,"rotation":15,"trajectory":30,"lm_speed":4421,"rm_speed":3966},{"x":0.5,"y":6.18,"rotation":15,"trajectory":30,"lm_speed":6787,"rm_speed":1301}]})json";
static const char SinglesRHServicePreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":-0.5,"y":2.5,"rotation":-15,"trajectory":10,"lm_speed":958,"rm_speed":2935},{"x":-2.05,"y":2.5,"rotation":-15,"trajectory":10,"lm_speed":3382,"rm_speed":961},{"x":-2.05,"y":6.18,"rotation":-15,"trajectory":30,"lm_speed":3966,"rm_speed":4421},{"x":-0.5,"y":6.18,"rotation":-15,"trajectory":30,"lm_speed":1301,"rm_speed":6787}]})json";
static const char DoublesLHServicePreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":0.5,"y":2.5,"rotation":15,"trajectory":10,"lm_speed":2935,"rm_speed":958},{"x":2.51,"y":2.5,"rotation":15,"trajectory":10,"lm_speed":370,"rm_speed":4272},{"x":2.51,"y":5.42,"rotation":15,"trajectory":30,"lm_speed":2881,"rm_speed":4458},{"x":0.5,"y":5.42,"rotation":15,"trajectory":30,"lm_speed":5583,"rm_speed":1157}]})json";
static const char DoublesRHServicePreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":-0.5,"y":2.5,"rotation":-15,"trajectory":10,"lm_speed":958,"rm_speed":2935},{"x":-2.51,"y":2.5,"rotation":-15,"trajectory":10,"lm_speed":4272,"rm_speed":370},{"x":-2.51,"y":5.42,"rotation":-15,"trajectory":30,"lm_speed":4458,"rm_speed":2881},{"x":-0.5,"y":5.42,"rotation":-15,"trajectory":30,"lm_speed":1157,"rm_speed":5583}]})json";
static const char SinglesFullCourtPreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":-2.05,"y":1.3,"rotation":-20,"trajectory":5,"lm_speed":2927,"rm_speed":517},{"x":0,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":1422,"rm_speed":1422},{"x":2.05,"y":1.3,"rotation":20,"trajectory":5,"lm_speed":517,"rm_speed":2927},{"x":-2.05,"y":4,"rotation":-20,"trajectory":20,"lm_speed":2552,"rm_speed":2914},{"x":0,"y":4,"rotation":0,"trajectory":20,"lm_speed":2546,"rm_speed":2546},{"x":2.05,"y":4,"rotation":20,"trajectory":20,"lm_speed":2914,"rm_speed":2552},{"x":-2.05,"y":6.18,"rotation":-20,"trajectory":30,"lm_speed":2741,"rm_speed":5646},{"x":0,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":3969,"rm_speed":3969},{"x":2.05,"y":6.18,"rotation":20,"trajectory":30,"lm_speed":5646,"rm_speed":2741}]})json";
static const char DoublesFullCourtPreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":-2.51,"y":1.3,"rotation":-20,"trajectory":5,"lm_speed":3810,"rm_speed":-66},{"x":0,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":1422,"rm_speed":1422},{"x":2.51,"y":1.3,"rotation":20,"trajectory":5,"lm_speed":-66,"rm_speed":3810},{"x":-2.51,"y":4,"rotation":-20,"trajectory":20,"lm_speed":3293,"rm_speed":2398},{"x":0,"y":4,"rotation":0,"trajectory":20,"lm_speed":2546,"rm_speed":2546},{"x":2.51,"y":4,"rotation":20,"trajectory":20,"lm_speed":2398,"rm_speed":3293},{"x":-2.51,"y":6.18,"rotation":-20,"trajectory":30,"lm_speed":3599,"rm_speed":5087},{"x":0,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":3969,"rm_speed":3969},{"x":2.51,"y":6.18,"rotation":20,"trajectory":30,"lm_speed":5087,"rm_speed":3599}]})json";
static const char SinglesNoRotationPreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":-2.05,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":5289,"rm_speed":-1844},{"x":0,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":1422,"rm_speed":1422},{"x":2.05,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":-1844,"rm_speed":5289},{"x":-2.05,"y":4,"rotation":0,"trajectory":20,"lm_speed":5846,"rm_speed":-379},{"x":0,"y":4,"rotation":0,"trajectory":20,"lm_speed":2546,"rm_speed":2546},{"x":2.05,"y":4,"rotation":0,"trajectory":20,"lm_speed":-379,"rm_speed":5846},{"x":-2.05,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":7697,"rm_speed":690},{"x":0,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":3969,"rm_speed":3969},{"x":2.05,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":690,"rm_speed":7697}]})json";
static const char DoublesNoRotationPreset[] = R"json({"command":"update_shots","height":"1.5","velocity_correction":"0","flight_correction":"0","default_delay":"3","shots":[{"x":-2.51,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":6619,"rm_speed":-2874},{"x":0,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":1422,"rm_speed":1422},{"x":2.51,"y":1.3,"rotation":0,"trajectory":5,"lm_speed":-2874,"rm_speed":6619},{"x":-2.51,"y":4,"rotation":0,"trajectory":20,"lm_speed":6814,"rm_speed":-1122},{"x":0,"y":4,"rotation":0,"trajectory":20,"lm_speed":2546,"rm_speed":2546},{"x":2.51,"y":4,"rotation":0,"trajectory":20,"lm_speed":-1122,"rm_speed":6814},{"x":-2.51,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":8786,"rm_speed":-98},{"x":0,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":3969,"rm_speed":3969},{"x":2.51,"y":6.18,"rotation":0,"trajectory":30,"lm_speed":-98,"rm_speed":8786}]})json";
int gripper_open_pos = 90;
int gripper_closed_pos = 90;
int pusher_catch_pos = 90;
int pusher_push_pos = 90;
int left_rotation_limit = 90;
int right_rotation_limit = 90;
int maxSpeed = 20000; // motorKV * batteryVoltage; 750kV Motor * 11.1V 3S Battery = 8,325. However that's not the ESC max...
// ESC max for a 12 pole motor is 40,000 RPM.
// If there is a linear response and 0 RPM is low, and 40K is high... & i've been using 7500K as max... then i'd expext
int escLowPulseSetPoint = map(2500, 0, maxSpeed, escMinPulseLength, escMaxPulseLength);
int escHighPulseSetPoint = map(5000, 0, maxSpeed, escMinPulseLength, escMaxPulseLength);
int rpmLowSetPoint = 2500;
int rpmHighSetPoint = 5000;
float traj_servo_correction = 0;
float traj_correction = 0;
static bool stop_launcher = true;
bool shuttle_check = true;
bool obstacle_check = true;

// Setup Servo Objects
Servo servoPusher;
Servo servoGripper;
Servo servoTrajectory;
Servo servoRotation;
Servo servoLeftESC;
Servo servoRightESC;

// Setup Sonar Object
NewPing sonar(ultrasonicTriggerPin, ultrasonicEchoPin, ultrasonicMaxDistance); // Setup the ultrasonic sensor pins and max distance.

// The JSON library uses static memory, so this will need to be allocated:
StaticJsonDocument<4096> doc_tx;
StaticJsonDocument<4096> doc_rx;

// Setup the web server
AsyncWebServer server(http_port);
WebSocketsServer webSocket = WebSocketsServer(ws_port);
char msg_buf[10];

void sendStatus(String status_text, String status_color);
void printStatusOnTFT(const char *msg, uint16_t color = TFT_TEXT_COLOR);
void sendParameters();
void stopAll();
int getTrajectoryServoPos(float inputAngle);
int getRotationServoPos(float inputAngle);
bool loadAndStartUserPreset();

/***********************************************************
   Functions
*/

bool parsePlayType(const char *playType, PLAY_TYPE &parsedType)
{
  if (playType == nullptr)
    return false;
  if (strcmp(playType, "once") == 0)
    parsedType = PLAY_ONCE;
  else if (strcmp(playType, "repeat") == 0)
    parsedType = REPEAT_SHOTS;
  else if (strcmp(playType, "random_order") == 0)
    parsedType = RANDOM_ORDER;
  else if (strcmp(playType, "random_shots") == 0)
    parsedType = RANDOM_SHOTS;
  else
    return false;
  return true;
}

void beginPlayback(PLAY_TYPE playType)
{
  if (playType != RANDOM_SHOTS && sequence_length == 0)
  {
    sendStatus("Shot pattern is empty", "red");
    return;
  }
  ACTIVE_PLAY_TYPE = playType;
  current_sequence_index = 0;
  system_state = ST_START;
}

void sendUserPreset()
{
  String jsonString;
  doc_tx.clear();
  JsonObject object = doc_tx.to<JsonObject>();
  object["command"] = "custom_preset";

  if (!preferences.begin("CustomPattern", true))
  {
    sendStatus("Unable to read custom preset storage", "red");
    return;
  }
  int activeSlot = preferences.getInt("active", -1);
  if (activeSlot < 0 || activeSlot > 1)
  {
    preferences.end();
    object["available"] = false;
  }
  else
  {
    String suffix = String(activeSlot);
    object["available"] = true;
    object["name"] = preferences.getString((String("name") + suffix).c_str(), "Custom User Preset");
    object["play_type"] = preferences.getString((String("mode") + suffix).c_str(), "once");
    object["default_delay"] = preferences.getFloat((String("delay") + suffix).c_str(), 3.0f);
    object["shots_json"] = preferences.getString((String("shots") + suffix).c_str(), "[]");
    object["sequence_json"] = preferences.getString((String("sequence") + suffix).c_str(), "[]");
    preferences.end();
  }

  serializeJson(doc_tx, jsonString);
  webSocket.broadcastTXT(jsonString);
}

bool loadAndStartUserPreset()
{
  if (!preferences.begin("CustomPattern", true))
  {
    sendStatus("Unable to read custom preset storage", "red");
    return false;
  }
  int activeSlot = preferences.getInt("active", -1);
  if (activeSlot < 0 || activeSlot > 1)
  {
    preferences.end();
    sendStatus("No custom preset saved", "red");
    return false;
  }

  String suffix = String(activeSlot);
  String shotsJson = preferences.getString((String("shots") + suffix).c_str(), "[]");
  String sequenceJson = preferences.getString((String("sequence") + suffix).c_str(), "[]");
  String playTypeName = preferences.getString((String("mode") + suffix).c_str(), "once");
  float presetDelay = preferences.getFloat((String("delay") + suffix).c_str(), 3.0f);
  String presetName = preferences.getString((String("name") + suffix).c_str(), "Custom User Preset");
  preferences.end();

  PLAY_TYPE playType;
  if (!parsePlayType(playTypeName.c_str(), playType) || !isfinite(presetDelay) || presetDelay <= 0)
  {
    sendStatus("Custom preset settings are invalid", "red");
    return false;
  }

  DeserializationError error = deserializeJson(doc_rx, shotsJson);
  if (error || !doc_rx.is<JsonArray>())
  {
    sendStatus("Custom preset shots are invalid", "red");
    return false;
  }
  JsonArray shots = doc_rx.as<JsonArray>();
  if (shots.size() == 0 || shots.size() > 10)
  {
    sendStatus("Custom preset shot count is invalid", "red");
    return false;
  }

  int shotCount = 0;
  for (JsonObject shot : shots)
  {
    if (!shot["x"].is<float>() || !shot["y"].is<float>() ||
        !shot["rotation"].is<float>() || !shot["trajectory"].is<float>() ||
        !shot["lm_speed"].is<int>() || !shot["rm_speed"].is<int>())
    {
      sendStatus("Custom preset shot data is invalid", "red");
      return false;
    }
    active_shots[shotCount].court_x_pos = shot["x"].as<float>();
    active_shots[shotCount].court_y_pos = shot["y"].as<float>();
    active_shots[shotCount].rotation = shot["rotation"].as<float>();
    active_shots[shotCount].trajectory = shot["trajectory"].as<float>();
    active_shots[shotCount].left_motor_rpm = shot["lm_speed"].as<int>();
    active_shots[shotCount].right_motor_rpm = shot["rm_speed"].as<int>();
    shotCount++;
  }

  error = deserializeJson(doc_rx, sequenceJson);
  if (error || !doc_rx.is<JsonArray>())
  {
    sendStatus("Custom preset sequence is invalid", "red");
    return false;
  }
  JsonArray sequence = doc_rx.as<JsonArray>();
  if (sequence.size() > 50 || (playType != RANDOM_SHOTS && sequence.size() == 0))
  {
    sendStatus("Custom preset sequence length is invalid", "red");
    return false;
  }

  int sequenceIndex = 0;
  for (JsonObject entry : sequence)
  {
    int shotId = entry["shotID"].as<int>();
    float delay = entry["delay"].as<float>();
    if (!entry["shotID"].is<int>() || !entry["delay"].is<float>() ||
        shotId < 0 || shotId >= shotCount || !isfinite(delay) || delay <= 0 || delay > 60)
    {
      sendStatus("Custom preset sequence entry is invalid", "red");
      return false;
    }
    shot_sequence[sequenceIndex].ID = shotId;
    shot_sequence[sequenceIndex].delay = delay;
    sequenceIndex++;
  }

  active_shot_array_length = shotCount;
  sequence_length = sequenceIndex;
  default_delay = presetDelay;
  sendStatus(presetName, "green");
  beginPlayback(playType);
  return system_state == ST_START;
}

bool saveUserPreset()
{
  JsonArray shots = doc_rx["shots"].as<JsonArray>();
  JsonArray sequence = doc_rx["shot_pattern"].as<JsonArray>();
  const char *playTypeValue = doc_rx["play_type"];
  PLAY_TYPE playType;
  String presetName = doc_rx["name"].as<String>();
  presetName.trim();
  float presetDelay = doc_rx["default_delay"].as<float>();

  if (!doc_rx["shots"].is<JsonArray>() || shots.size() == 0 || shots.size() > 10)
  {
    sendStatus("Custom preset needs 1 to 10 shots", "red");
    return false;
  }
  if (!doc_rx["shot_pattern"].is<JsonArray>() || sequence.size() > 50)
  {
    sendStatus("Custom preset sequence is invalid", "red");
    return false;
  }
  if (!parsePlayType(playTypeValue, playType))
  {
    sendStatus("Custom preset play mode is invalid", "red");
    return false;
  }
  if (presetName.length() == 0 || presetName.length() > 31)
  {
    sendStatus("Preset name must be 1 to 31 characters", "red");
    return false;
  }
  if (!isfinite(presetDelay) || presetDelay <= 0 || presetDelay > 60)
  {
    sendStatus("Custom preset delay is invalid", "red");
    return false;
  }
  if (playType != RANDOM_SHOTS && sequence.size() == 0)
  {
    sendStatus("Add shots to the pattern before saving", "red");
    return false;
  }

  for (JsonObject shot : shots)
  {
    if (!shot["x"].is<float>() || !shot["y"].is<float>() ||
        !shot["rotation"].is<float>() || !shot["trajectory"].is<float>() ||
        !shot["lm_speed"].is<int>() || !shot["rm_speed"].is<int>())
    {
      sendStatus("Custom preset shot data is invalid", "red");
      return false;
    }
  }
  for (JsonObject entry : sequence)
  {
    int shotId = entry["shotID"].as<int>();
    float delay = entry["delay"].as<float>();
    if (!entry["shotID"].is<int>() || !entry["delay"].is<float>() ||
        shotId < 0 || shotId >= shots.size() || !isfinite(delay) || delay <= 0 || delay > 60)
    {
      sendStatus("Custom preset sequence entry is invalid", "red");
      return false;
    }
  }

  String shotsJson;
  String sequenceJson;
  serializeJson(shots, shotsJson);
  serializeJson(sequence, sequenceJson);
  if (shotsJson.length() > 1800 || sequenceJson.length() > 1800)
  {
    sendStatus("Custom preset is too large to save", "red");
    return false;
  }

  if (!preferences.begin("CustomPattern", false))
  {
    sendStatus("Unable to open custom preset storage", "red");
    return false;
  }
  int activeSlot = preferences.getInt("active", -1);
  int targetSlot = activeSlot == 0 ? 1 : 0;
  String suffix = String(targetSlot);
  bool saved =
      preferences.putString((String("name") + suffix).c_str(), presetName) > 0 &&
      preferences.putString((String("mode") + suffix).c_str(), playTypeValue) > 0 &&
      preferences.putFloat((String("delay") + suffix).c_str(), presetDelay) > 0 &&
      preferences.putString((String("shots") + suffix).c_str(), shotsJson) > 0 &&
      preferences.putString((String("sequence") + suffix).c_str(), sequenceJson) > 0;
  if (saved)
    saved = preferences.putInt("active", targetSlot) > 0;
  preferences.end();

  if (!saved)
  {
    sendStatus("Failed to save custom preset", "red");
    return false;
  }

  sendStatus("Custom preset saved", "green");
  sendUserPreset();
  return true;
}

// Handle commands from incoming JSON payloads
void handlePayload(const char *payload, size_t length)
{
  DeserializationError error = deserializeJson(doc_rx, payload, length);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    sendStatus("Invalid command payload", "red");
    return;
  }

  const char *command = doc_rx["command"];
  if (command == nullptr)
  {
    sendStatus("Command is missing", "red");
    return;
  }

  if (strcmp(command, "start") == 0)
  {
    const char *play_type = doc_rx["play_type"];
    PLAY_TYPE playType;
    if (!parsePlayType(play_type, playType))
    {
      sendStatus("Play mode is invalid", "red");
      return;
    }
    if (active_shot_array_length <= 0 || active_shot_array_length > 10)
    {
      sendStatus("No shots are loaded", "red");
      return;
    }
    if (!doc_rx["shot_pattern"].is<JsonArray>())
    {
      sendStatus("Shot pattern is invalid", "red");
      return;
    }
    JsonArray pattern = doc_rx["shot_pattern"].as<JsonArray>();
    if (pattern.size() > 50 || (playType != RANDOM_SHOTS && pattern.size() == 0))
    {
      sendStatus("Shot pattern length is invalid", "red");
      return;
    }

    int array_index = 0;
    for (JsonObject shot_pattern : pattern)
    {
      int shotId = shot_pattern["shotID"].as<int>();
      float delay = shot_pattern["delay"].as<float>();
      if (!shot_pattern["shotID"].is<int>() || !shot_pattern["delay"].is<float>() ||
          shotId < 0 || shotId >= active_shot_array_length ||
          !isfinite(delay) || delay <= 0 || delay > 60)
      {
        sendStatus("Shot pattern entry is invalid", "red");
        return;
      }
      shot_sequence[array_index].ID = shotId;
      shot_sequence[array_index].delay = delay;
      array_index++;
    }

    sequence_length = array_index;
    beginPlayback(playType);
  }
  else if (strcmp(command, "save_user_preset") == 0)
  {
    saveUserPreset();
  }
  else if (strcmp(command, "get_user_preset") == 0)
  {
    sendUserPreset();
  }
  else if (strcmp(command, "save_globals") == 0)
  {
    // Example JSON: {"command":"save_globals","height":"1.5","velocity_correction":"0","flight_correction":"0"}

    Serial.println("Save globals.");
    sendStatus("Saving Globals", "orange");
    preferences.begin("Globals", false);
    preferences.putFloat("velocity_adj", doc_rx["velocity_correction"].as<float>());
    preferences.putFloat("flight_adj", doc_rx["flight_correction"].as<float>());
    preferences.putFloat("launcher_y", doc_rx["height"].as<float>());
    preferences.end();
    sendStatus("Ready", "green");
  }
  else if (strcmp(command, "update_shots") == 0)
  {
    if (!doc_rx["shots"].is<JsonArray>() || doc_rx["shots"].as<JsonArray>().size() == 0 ||
        doc_rx["shots"].as<JsonArray>().size() > 10)
    {
      sendStatus("Shot count must be between 1 and 10", "red");
      return;
    }
    // Example JSON: {"command":"update_shots","height":"1.5","velocity_correction":"0","shots":[{"x":-2.5,"y":1.5,"rotation":-19,"trajectory":25},{"x":-0.937876256651103,"y":0.9950842255448553,"rotation":0,"trajectory":25},{"x":2.5,"y":1.5,"rotation":19,"trajectory":25},{"x":-2.5,"y":4,"rotation":-19,"trajectory":30},{"x":-1.0410451154325955,"y":3.5397617551513334,"rotation":0,"trajectory":30},{"x":2.5,"y":4,"rotation":19,"trajectory":30},{"x":-1.913775113193595,"y":5.877384048675795,"rotation":-15,"trajectory":40},{"x":0,"y":5.5,"rotation":0,"trajectory":40},{"x":2.5,"y":5.5,"rotation":15,"trajectory":40},{"x":0,"y":3,"rotation":0,"trajectory":35}]}
    default_delay = doc_rx["default_delay"].as<float>();

    int array_index = 0;
    for (JsonObject settings_shot : doc_rx["shots"].as<JsonArray>())
    {
      active_shots[array_index].court_x_pos = settings_shot["x"].as<float>();
      active_shots[array_index].court_y_pos = settings_shot["y"].as<float>();
      active_shots[array_index].trajectory = settings_shot["trajectory"].as<float>();
      active_shots[array_index].rotation = settings_shot["rotation"].as<float>();
      active_shots[array_index].left_motor_rpm = settings_shot["lm_speed"].as<int>();
      active_shots[array_index].right_motor_rpm = settings_shot["rm_speed"].as<int>();

      Serial.printf("index:%i   x: %f   y: %f   traj: %f   rot: %f   lm: %i   rm: %i", array_index, active_shots[array_index].court_x_pos, active_shots[array_index].court_y_pos, active_shots[array_index].trajectory, active_shots[array_index].rotation, active_shots[array_index].left_motor_rpm, active_shots[array_index].right_motor_rpm);
      Serial.println("");
      array_index++;
    }
    active_shot_array_length = array_index;
  }
  else if (strcmp(command, "update_setup_values") == 0)
  {
    // Example JSON
    gripper_open_pos = doc_rx["gripper_open_pos"].as<int>();         // "0"
    gripper_closed_pos = doc_rx["gripper_closed_pos"].as<int>();     // "80"
    pusher_catch_pos = doc_rx["pusher_catch_pos"].as<int>();         // "90"
    pusher_push_pos = doc_rx["pusher_push_pos"].as<int>();           // "35"
    left_rotation_limit = doc_rx["left_rotation_limit"].as<int>();   // "60"
    right_rotation_limit = doc_rx["right_rotation_limit"].as<int>(); // "120"
  }
  else if (strcmp(command, "stop") == 0)
  {
    // Example JSON
    printStatusOnTFT("Stop command received");
    stopAll();
  }
  else if (strcmp(command, "setup_gripper_open") == 0)
  {
    // Example JSON {"command":"setup_gripper_open"}
    Serial.println("Gripper Opening");
    sendStatus("Opening Gripper", "orange");
    servoGripper.write(gripper_open_pos);
  }
  else if (strcmp(command, "setup_gripper_close") == 0)
  {
    // Example JSON {"command":"setup_gripper_close"}
    Serial.println("Gripper Closing");
    sendStatus("Closing Gripper", "orange");
    servoGripper.write(gripper_closed_pos);
  }
  else if (strcmp(command, "setup_pusher_catch") == 0)
  {
    // Example JSON {"command":"setup_pusher_catch"}
    Serial.println("Moving to Catch Position");
    sendStatus("Moving Pusher to Catch Position", "orange");
    servoPusher.write(pusher_catch_pos);
  }
  else if (strcmp(command, "setup_pusher_push") == 0)
  {
    // Example JSON {"command":"setup_pusher_push"}
    Serial.println("Moving to Push Position");
    sendStatus("Moving Pusher to Push Position", "orange");
    servoPusher.write(pusher_push_pos);
  }
  else if (strcmp(command, "setup_left_rotation") == 0)
  {
    // Example JSON {"command":"setup_left_rotation"}
    Serial.println("Turning to left rotation limit " + String(left_rotation_limit));
    sendStatus("Turning to left rotation limit", "orange");
    servoRotation.write(left_rotation_limit);
  }
  else if (strcmp(command, "setup_right_rotation") == 0)
  {
    // Example JSON {"command":"setup_right_rotation"}
    Serial.println("Turning to right rotation limit " + String(right_rotation_limit));
    sendStatus("Turning to right rotation limit", "orange");
    servoRotation.write(right_rotation_limit);
  }
  else if (strcmp(command, "setup_save_servo") == 0)
  {
    // Example JSON  {"command":"setup_save"}
    Serial.println("Save calibration.");
    sendStatus("Saving Settings", "orange");
    preferences.begin("Servo_Settings", false);
    preferences.putInt("gripper_open", gripper_open_pos);
    preferences.putInt("gripper_closed", gripper_closed_pos);
    preferences.putInt("pusher_catch", pusher_catch_pos);
    preferences.putInt("pusher_push", pusher_push_pos);
    preferences.putInt("left_limit", left_rotation_limit);
    preferences.putInt("right_limit", right_rotation_limit);
    preferences.end();
    sendStatus("Ready", "green");
  }
  else if (strcmp(command, "setup_esc_max") == 0)
  {
    // Example JSON {"command":"setup_esc_max"}
    Serial.println("Sending max throttle");
    sendStatus("Calibration - Max Throttle", "orange");
    servoLeftESC.writeMicroseconds(escMaxPulseLength);
    servoRightESC.writeMicroseconds(escMaxPulseLength);
  }
  else if (strcmp(command, "setup_esc_min") == 0)
  {
    // Example JSON {"command":"setup_esc_min"}
    Serial.println("Sending min throttle");
    sendStatus("Calibration - Min Throttle", "orange");
    servoLeftESC.writeMicroseconds(escMinPulseLength);
    servoRightESC.writeMicroseconds(escMinPulseLength);
  }
  else if (strcmp(command, "setup_rpm_run") == 0)
  {
    // Example JSON {"command":"setup_run","setup_rpm":"500"}
    Serial.println("Calibration Run Motor.");
    sendStatus("Continuous Run", "orange");
    int setup_rpm = doc_rx["setup_rpm"].as<int>(); // "50"

    // At lower speeds first start with a higher value to get the motor spinning.
    if (setup_rpm < 2500)
    {
      servoLeftESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
      servoRightESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
      delay(100);
    }

    // Run the motor based on the current set points as mapped.
    servoLeftESC.writeMicroseconds(map(setup_rpm, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
    servoRightESC.writeMicroseconds(map(setup_rpm, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
  }
  else if (strcmp(command, "setup_stop") == 0)
  {
    // Example JSON {"command":"setup_stop"}
    Serial.println("Calibration Stop Motor.");
    sendStatus("Stop Motor", "orange");
    servoLeftESC.writeMicroseconds(escMinPulseLength);
    servoRightESC.writeMicroseconds(escMinPulseLength);
    sendStatus("Ready", "green");
  }
  else if (strcmp(command, "setup_pos") == 0)
  {
    // Example JSON {"command":"setup_pos"}
    Serial.println("Moving to setup pose.");
    sendStatus("Calibration - Setup Pose", "orange");

    servoPusher.write(90);
    servoGripper.write(90);
    servoTrajectory.write(0);
    servoRotation.write(90);
  }
  else if (strcmp(command, "setup_high_speed") == 0)
  {
    // Example JSON {"command":"setup_high_speed"}
    Serial.println("Going to high speed.");
    sendStatus("Going to high speed.", "orange");
    // Go to 5000RPM based on original scaling
    servoLeftESC.writeMicroseconds(map(5000, 0, maxSpeed, escMinPulseLength, escMaxPulseLength));
    servoRightESC.writeMicroseconds(map(5000, 0, maxSpeed, escMinPulseLength, escMaxPulseLength));
  }
  else if (strcmp(command, "setup_low_speed") == 0)
  {
    // Example JSON {"command":"setup_low_speed"}
    Serial.println("Going to low speed.");
    sendStatus("Going to low speed.", "orange");

    // Get the motors spinning, then after we'll slow to 2500
    servoLeftESC.writeMicroseconds(map(3000, 0, maxSpeed, escMinPulseLength, escMaxPulseLength));
    servoRightESC.writeMicroseconds(map(3000, 0, maxSpeed, escMinPulseLength, escMaxPulseLength));
    delay(100);

    // Go to 2500RPM based on original scaling
    servoLeftESC.writeMicroseconds(map(2500, 0, maxSpeed, escMinPulseLength, escMaxPulseLength));
    servoRightESC.writeMicroseconds(map(2500, 0, maxSpeed, escMinPulseLength, escMaxPulseLength));
  }
  else if (strcmp(command, "setup_adjust_rpm") == 0)
  {
    // Example JSON {"command":"setup_adjust_rpm","setup_rpm_low":"2500","setup_rpm_high":"5000"}
    Serial.println("Adjusting RPM Values");
    sendStatus("Adjusting RPM Values", "orange");

    // Use the pulse values at 2500 & 5000 along with the measured speeds at those pulse lengths to set speed maping during feeding.
    rpmLowSetPoint = doc_rx["setup_rpm_low"].as<int>();   // "2500"
    rpmHighSetPoint = doc_rx["setup_rpm_high"].as<int>(); // "5000"

    // Print out the maping points. Perhaps knowing these we can improve the base values.
    Serial.printf("Low Point - Pulse Width:%i   RPM: %i", escLowPulseSetPoint, rpmLowSetPoint);
    Serial.println("");
    Serial.printf("High Point - Pulse Width:%i   RPM: %i", escHighPulseSetPoint, rpmHighSetPoint);
    Serial.println("");

    // Save
    preferences.begin("Servo_Settings", false);
    preferences.putInt("rpmLowSetPoint", rpmLowSetPoint);
    preferences.putInt("rpmHighSetPoint", rpmHighSetPoint);
    preferences.end();
  }
  else if (strcmp(command, "setup_traj_zero") == 0)
  {
    // Example JSON {"command":"setup_traj_zero","setup_traj_correction":"2"}
    Serial.println("Adjusting Trajectory Zero");
    sendStatus("Adjusting Trajectory Zero", "orange");

    traj_correction = doc_rx["setup_traj_correction"].as<float>(); // "0"

    // zero out the servo correction so that it's not affecting itself then find the correction in servo degrees.
    traj_servo_correction = 0;
    traj_servo_correction = getTrajectoryServoPos(traj_correction) - getTrajectoryServoPos(0); // "500"
    Serial.println(traj_servo_correction);
    // Go to the zero position with the correction factored in.
    servoTrajectory.write(getTrajectoryServoPos(0));

    // Save
    preferences.begin("Servo_Settings", false);
    preferences.putFloat("traj_servo_adj", traj_servo_correction);
    preferences.putFloat("traj_correction", traj_correction);
    preferences.end();
  }
  else if (strcmp(command, "setup_update_settings") == 0)
  {
    // Example JSON {"command":"setup_update_settings","shuttle_check":true,"obstacle_check":true}
    Serial.println("updating settings");
    sendStatus("Updating Settings", "orange");
    shuttle_check = doc_rx["shuttle_check"].as<bool>();   // "0"
    obstacle_check = doc_rx["obstacle_check"].as<bool>(); // "0"
  }
  else if (strcmp(command, "get_preset") == 0)
  {
    // Example JSON {"command":"get_preset","preset":"Doubles - Service Return"}
    Serial.println("Request Preset Load.");
    sendStatus("Loading Preset.", "orange");
    // sendShots();
  }
  else
  {
    Serial.println("Command not recognized.");
    sendStatus("Command Not Recognized", "red");
  }
}

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length)
{
  // Figure out the type of WebSocket event
  switch (type)
  {
  // Client has disconnected
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", client_num);
    break;

  // New client has connected
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(client_num);
    Serial.printf("[%u] Connection from ", client_num);
    Serial.println(ip.toString());
    sendStatus("Ready", "green");
    sendParameters();
    sendUserPreset();
  }
  break;

  // Handle messages from client
  case WStype_TEXT:
  {
    // Stop current feeding.
    // stopAll();

    // Print out raw message
    Serial.printf("[%u] Received text: %s\n", client_num, payload);
    handlePayload(reinterpret_cast<const char *>(payload), length);
    Serial.println("");
    break;
  }

  // For everything else: do nothing
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (index) of " + request->url());
  request->send(SPIFFS, "/ace.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (css) of " + request->url());
  request->send(SPIFFS, "/simple.css", "text/css");
}

// Callback: send logo image
void onLogoRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (image) of " + request->url());
  request->send(SPIFFS, pngFilename, "image/png");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (404) of " + request->url());
  request->send(404, "text/plain", "Not found");
}

// Send status update
void sendStatus(String status_text, String status_color)
{
  String jsonString = "";
  JsonObject object = doc_tx.to<JsonObject>();
  object["command"] = "status";
  object["statusText"] = status_text;
  object["statusColor"] = status_color;
  serializeJson(doc_tx, jsonString);
  // Serial.printf("Sending: %s\n", jsonString);
  // webSocket.sendTXT(client_num, msg_buf);
  webSocket.broadcastTXT(jsonString); // send JSON string to all clients
  // additionally send to TFT display
  uint16_t text_color = TFT_TEXT_COLOR;
  if (status_color == "red")
    text_color = TFT_RED;
  else if (status_color == "green")
    text_color = TFT_GREEN;
  else if (status_color == "lightblue")
    text_color = TFT_DARKCYAN;
  else if (status_color == "blue")
    text_color = TFT_BLUE;
  else if (status_color == "orange")
    text_color = TFT_ORANGE;
  else if (status_color == "yellow")
    text_color = TFT_YELLOW;
  printStatusOnTFT(status_text.c_str(), text_color);
}
// Send parameters
void sendParameters()
{
  String jsonString = "";
  JsonObject object = doc_tx.to<JsonObject>();

  object["command"] = "load_setup_values";

  // send servo positions
  object["gripper_open"] = gripper_open_pos;
  object["gripper_closed"] = gripper_closed_pos;
  object["pusher_catch"] = pusher_catch_pos;
  object["pusher_push"] = pusher_push_pos;
  object["left_limit"] = left_rotation_limit;
  object["right_limit"] = right_rotation_limit;
  object["rmpLowSetPoint"] = rpmLowSetPoint;
  object["rmpHighSetPoint"] = rpmHighSetPoint;
  object["traj_correction"] = traj_correction;

  // send global values
  Serial.println("Load Globals.");
  preferences.begin("Globals", false);
  object["velocity_correction"] = preferences.getFloat("velocity_adj", 0);
  object["flight_correction"] = preferences.getFloat("flight_adj", 0);
  object["height"] = roundf(preferences.getFloat("launcher_y", 1.5) * 100) / 100;
  preferences.end();

  serializeJson(doc_tx, jsonString);
  Serial.printf("Sending: %s\n", jsonString);
  // webSocket.sendTXT(client_num, msg_buf);
  webSocket.broadcastTXT(jsonString); // send JSON string to all clients
}

// Send Shots
void sendShots()
{
  String jsonString = "";
  JsonObject object = doc_tx.to<JsonObject>();

  object["command"] = "load_shot_values";
  JsonObject Shots = object.createNestedObject("shots");

  for (int i = 0; i < 10; i++)
  {

    JsonObject Shot = Shots.createNestedObject(String(i));
    Shot["x"] = active_shots[i].court_x_pos;
    Shot["y"] = active_shots[i].court_y_pos;
    Shot["trajectory"] = active_shots[i].trajectory;
    Shot["rotation"] = active_shots[i].rotation;
  }

  // object["shots"] = active_shots;
  // myJSON = '["Ford", "BMW", "Fiat"]';

  // JsonArray

  /* FROM RECEIPT OF SHotS
   for (JsonObject settings_shot : doc_rx["shots"].as<JsonArray>()) {
            active_shots[array_index].court_x_pos = settings_shot["x"].as<float>();
            active_shots[array_index].court_y_pos = settings_shot["y"].as<float>();
            active_shots[array_index].trajectory = settings_shot["trajectory"].as<float>();
            active_shots[array_index].rotation = settings_shot["rotation"].as<float>();
            active_shots[array_index].left_motor_rpm = settings_shot["lm_speed"].as<int>();
            active_shots[array_index].right_motor_rpm = settings_shot["rm_speed"].as<int>();*/

  // DynamicJsonDocument doc(1024);
  // JsonObject wifi  = doc.createNestedObject("wifi");
  // wifi["SSID"] = "TheBatCave";

  serializeJson(doc_tx, jsonString);
  Serial.printf("Sending: %s\n", jsonString);
  // webSocket.sendTXT(client_num, msg_buf);
  webSocket.broadcastTXT(jsonString); // send JSON string to all clients
}

void drawMultilineBtnLabel(int x, int y, int w, int h, uint16_t textColor, uint8_t textSizeFactor, uint8_t i, bool invert = false)
{
  uint16_t fontH = tft.fontHeight() * textSizeFactor;
  int startY = y - fontH;
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(BTN_TEXTSCALING);

  for (int line = 0; line < 2; line++)
  {
    int tw = tft.textWidth(tftMultilineBtnLabel[i][line]);
    if (invert)
    {
      tft.setTextColor(TFT_BKGND_COLOR);
    }
    else
    {
      tft.setTextColor(textColor);
    }
    tft.drawString(tftMultilineBtnLabel[i][line], x - (tw / 2), startY + line * fontH);
  }
}

// Draw the keypad on the TFT display
void drawKeypad()
{
  // Draw the keys
  for (uint8_t row = 0; row < BTN_ROWS; row++)
  {
    for (uint8_t col = 0; col < BTN_COLS; col++)
    {
      uint8_t b = col + row * BTN_COLS;

      if (b < 6)
        tft.setFreeFont(LABEL1_FONT);
      else
        tft.setFreeFont(LABEL2_FONT);

      tftBtnPos[b][0] = BTN_X + col * (BTN_WIDTH + BTN_SPACING_X);
      tftBtnPos[b][1] = BTN_Y + row * (BTN_HEIGHT + BTN_SPACING_Y);

      tftBtn[b].initButton(&tft, tftBtnPos[b][0], tftBtnPos[b][1], // x, y, w, h, outline, fill, text
                           BTN_WIDTH, BTN_HEIGHT, TFT_TEXT_COLOR, tftBtnColor[b], TFT_TEXT_COLOR,
                           (char *)"", BTN_TEXTSCALING);
      tftBtn[b].drawButton();
      drawMultilineBtnLabel(tftBtnPos[b][0], tftBtnPos[b][1], BTN_WIDTH, BTN_HEIGHT, TFT_TEXT_COLOR, BTN_TEXTSCALING, b);
    }
  }
}

// Perform TFT touch screen calibration and store the data in SPIFFS
void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE))
  {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f)
      {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL)
  {
    // calibration data valid
    tft.setTouch(calData);
  }
  else
  {
    Serial.println(F("Waiting for touch screen calibration!"));
    // data not valid so recalibrate
    tft.fillScreen(TFT_BKGND_COLOR);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_TEXT_COLOR, TFT_BKGND_COLOR);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL)
    {
      tft.setTextColor(TFT_RED, TFT_BKGND_COLOR);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BKGND_COLOR, 15);

    tft.setTextColor(TFT_GREEN, TFT_BKGND_COLOR);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f)
    {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

// Print something in the mini status bar of the TFT display
void printStatusOnTFT(const char *msg, uint16_t color)
{
  Serial.println("Status message updated: " + String(msg));
  tft.setTextPadding(STATUS_WIDTH);
  tft.setTextColor(color, TFT_BKGND_COLOR, true);
  tft.setFreeFont(LABEL1_FONT);
  // tft.setTextFont(0);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(1);
  tft.drawString(msg, STATUS_X, STATUS_Y);
}

// These are callback functions for handling png decoding callbacks.
// Callback function to draw pixels to the display
int pngDraw(PNGDRAW *pDraw)
{
  uint16_t lineBuffer[TFT_WIDTH]; // Buffer to hold one line of the image
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, TFT_BKGND_COLOR);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
  return 1;
}
void *pngOpen(const char *filename, int32_t *size)
{
  Serial.printf("Attempting to open %s\n", filename);
  pngfile = SPIFFS.open(filename, "r");
  *size = pngfile.size();
  return &pngfile;
}

void pngClose(void *handle)
{
  File pngfile = *((File *)handle);
  if (pngfile)
    pngfile.close();
}

int32_t pngRead(PNGFILE *page, uint8_t *buffer, int32_t length)
{
  if (!pngfile)
    return 0;
  page = page; // Avoid warning
  return pngfile.read(buffer, length);
}

int32_t pngSeek(PNGFILE *page, int32_t position)
{
  if (!pngfile)
    return 0;
  page = page; // Avoid warning
  return pngfile.seek(position);
}

// Open a PNG image and draw it on the TFT display
void drawPNG(int x = 0, int y = 0)
{
  // File file = SPIFFS.open(pngFilename, "r")
  if (SPIFFS.exists(pngFilename))
  {
    xpos = x;
    ypos = y;
    // Pass support callback function names to library
    int16_t rc = png.open(pngFilename, pngOpen, pngClose, pngRead, pngSeek, pngDraw);
    if (rc == PNG_SUCCESS)
    {
      tft.startWrite();
      Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
      if (png.getWidth() > TFT_WIDTH)
        Serial.println("Image too wide for allocated line buffer size!");
      else
      {
        rc = png.decode(NULL, 0);
        png.close();
      }

      tft.endWrite();
    }
  }
}

/***********************************************************
   Main
*/

void setup()
{
  // Setup the servo pins
  pinMode(servoPusherPin, OUTPUT);
  pinMode(servoGripperPin, OUTPUT);
  pinMode(servoTrajectoryPin, OUTPUT);
  pinMode(servoRotationPin, OUTPUT);
  pinMode(servoLeftESCPin, OUTPUT);
  pinMode(servoRightESCPin, OUTPUT);

  // Setup button IO
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);

  // Setup Through Beam Sensor
  pinMode(throughBeamPin, INPUT_PULLUP);

  // Setup onboard LED for indication
  pinMode(throughBeamIndicatorPin, OUTPUT);

  // Setup Ultrasonic Sesnor
  pinMode(ultrasonicTriggerPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT_PULLUP);

  // Start Serial port
  Serial.begin(115200);

  // Make sure we can read the file system
  if (!SPIFFS.begin())
  {
    Serial.println("Error mounting SPIFFS");
    while (1)
      ;
  }
  /**/
  // STAND-ALONE ACCESS POINT FOR DIRECT CONNECTION
  //  Start access point
  WiFi.softAP(ssid, password);
  // Print our IP address
  Serial.println();
  Serial.println("AP running");
  Serial.print("My IP address: ");
  Serial.println(WiFi.softAPIP());
  /**/
  /*
    //CONNECT TO EXISTING NETWORK FOR TESTING
    //Connect to Network
    ssid = "";
    password = "";
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
    Serial.println(WiFi.localIP());
    */

  if (!MDNS.begin(mDnsName))
  {
    Serial.println("Error starting mDNS");
    while (1)
      delay(1000);
  }
  Serial.println("mDNS Responder Started");

  // Add service to MDNS-SD
  MDNS.addService("https", "tcp", 80); // this catches https://mDnsName.local
  MDNS.addService("http", "tcp", 80);  // this catches http://mDnsName.local
  MDNS.addService("", "tcp", 80);      // this catches mDnsName.local

  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide style.css
  server.on("/simple.css", HTTP_GET, onCSSRequest);

  // On Logo request provide logo
  server.on(pngFilename, HTTP_GET, onLogoRequest);

  // Start web server
  server.begin();

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start WebSocket server and assign callback
  webSocket.begin();

  webSocket.onEvent(onWebSocketEvent);

  // Load calibration settings.
  Serial.println("Load calibration.");
  preferences.begin("Servo_Settings", false);
  gripper_open_pos = preferences.getInt("gripper_open", 90);
  gripper_closed_pos = preferences.getInt("gripper_closed", 90);
  pusher_catch_pos = preferences.getInt("pusher_catch", 90);
  pusher_push_pos = preferences.getInt("pusher_push", 90);
  left_rotation_limit = preferences.getInt("left_limit", 90);
  right_rotation_limit = preferences.getInt("right_limit", 90);

  rpmLowSetPoint = preferences.getInt("rpmLowSetPoint", 2500);
  rpmHighSetPoint = preferences.getInt("rpmHighSetPoint", 5000);
  traj_servo_correction = preferences.getFloat("traj_servo_adj", 0);
  traj_correction = preferences.getFloat("traj_correction", 0);
  preferences.end();

  // Setup servos
  servoPusher.attach(servoPusherPin, servoMinPulseLength, servoMaxPulseLength);
  servoGripper.attach(servoGripperPin, servoMinPulseLength, servoMaxPulseLength);
  servoTrajectory.attach(servoTrajectoryPin, servoMinPulseLength, servoMaxPulseLength);
  servoRotation.attach(servoRotationPin, servoMinPulseLength, servoMaxPulseLength);
  servoLeftESC.attach(servoLeftESCPin, escMinPulseLength, escMaxPulseLength);
  servoRightESC.attach(servoRightESCPin, escMinPulseLength, escMaxPulseLength);

  // Go to default servo positions
  servoGripper.write(gripper_closed_pos);
  servoPusher.write(pusher_catch_pos);
  servoTrajectory.write(0);
  servoRotation.write(90);

  // Stop the esc's.
  servoLeftESC.writeMicroseconds(escMinPulseLength);
  servoRightESC.writeMicroseconds(escMinPulseLength);

  // TFT setup
  if (LCDconnected)
  {
    uint16_t calibrationData[5];
    uint8_t calDataOK = 0;
    tft.init();
    tft.setRotation(2);
    tft.setTextWrap(true, true); // horizontal wrapping and vertical wrapping
    touch_calibrate();
    tft.fillScreen(TFT_BKGND_COLOR); // Clear the screen and draw logo and buttons
    drawPNG(xpos, ypos);
    tft.fillRect(0, 110, TFT_WIDTH, TFT_HEIGHT, TFT_BKGND_COLOR);
    delay(1000);
    drawKeypad();
  }

  Serial.println(F("Finished setup"));
}

void loop()
{
  static int last_time = millis();
  static int last_sonar_time = millis();
  static int last_through_beam_time = millis();
  static int last_state_time = millis();
  static int start_time = millis();
  static int blocked_counts = 0;
  static int count_down = 5;
  static bool countdown_required = true;

  int current_time = millis();

  static int startStatus = -1;
  static int stopStatus = -1;
  startStatus = digitalRead(startPin);
  // startStatus = analogRead(startPin);
  stopStatus = digitalRead(stopPin);
  // stopStatus = analogRead(stopPin);

  // Serial.print("Button Values (start, stop): ");
  // Serial.print(startStatus);
  // Serial.print(" | ");
  // Serial.println(stopStatus);

  // If the physical start button is pressed then start.
  if (startStatus == 0)
  {
    printStatusOnTFT("Start button pressed");
    static const char startPayload[] = R"json({
      "command": "start",
      "play_type": "random_shots",
      "shot_pattern": []
    })json";
    handlePayload(startPayload, sizeof(startPayload) - 1);
  }
  // If the physical stop button is pressed then stop.
  if (stopStatus == 0)
  {
    stopAll();
  }

  // Always start by checking the sonar sensor for any obstructions & stop imediately if detected.
  static bool sonar_long_blocked = false;

  if (current_time - last_sonar_time >= 100)
  {
    int distance = sonar.ping_cm();
    // Serial.print("Distance:" );
    // Serial.print(distance);
    // Serial.println("cm");
    last_sonar_time = current_time;
    // sendStatus("Distance: " + String(distance, 6) + "cm", "red"); //String(distance, 6)

    // Sensor library returns 0 for any distances greater than the max detection window set when initialized.
    // So in this case a value of zero means free from obstructions.
    if (distance > 0)
    {
      // If the sensor is blocked for 4 sonar pings (1s) then stop all.
      if (blocked_counts >= 4)
      {

        if (system_state > 0 && obstacle_check == true && sonar_long_blocked == false)
        {
          printStatusOnTFT("Obstruction detected", TFT_RED);
          stopAll();
          sonar_long_blocked = true;
        }
      }
      else
      {
        blocked_counts++;
      }
    }
    else
    {
      blocked_counts = 0;
      if (sonar_long_blocked == true)
      {
        sonar_long_blocked = false;
        sendStatus("Ready", "green");
      }
    }
  }

  // Update shuttle through-beam status.
  static int throughBeamClear = 0;
  if (current_time - last_through_beam_time >= 50)
  {
    throughBeamClear = digitalRead(throughBeamPin);
    digitalWrite(throughBeamIndicatorPin, throughBeamClear);
    // Serial.print("Beam Clear:");
    // Serial.println(throughBeamClear);
    last_through_beam_time = current_time;
  }

  // Look for and handle WebSocket data
  webSocket.loop();

  static bool skip_open_delay = false;
  static int shuttle_drop_attempts = 0;
  static int current_shot = 0;

  static int current_shot_delay = 0;
  static int current_lm_speed = 0;
  static int current_rm_speed = 0;
  static float current_trajectory = 0;
  static float current_rotation = 0;

  static float traj_servo_pos = 0;
  static float rot_servo_pos = 0;
  static int last_shot_delay = 0;

  // Do a countdown before starting feeding when first activated to allow player time to get to position.
  if (system_state > 0 && countdown_required == true && system_state != ST_RESET) //
  {
    if (current_time - last_time >= 1000)
    {

      String status_string = "Starting In ";
      status_string = status_string + count_down + "s";
      sendStatus(status_string, "green");
      last_time = millis();

      if (count_down <= 1)
      {
        countdown_required = false;
      }
      else
        count_down = count_down - 1;
    }
    return;
  }

  int open_delay = 500;
  int close_delay = 500;

  //////////////// TFT Keypad ////////////////////
  // drawPNG(xpos, ypos);
  // tft.fillRect(0, 110, TFT_WIDTH, TFT_HEIGHT, TFT_DARKGREY);
  // drawKeypad();
  uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
  bool pressed = tft.getTouch(&t_x, &t_y);
  if (pressed)
  {
    // / Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
      if (tftBtn[b].contains(t_x, t_y))
        tftBtn[b].press(true); // tell the button it is pressed
      else
        tftBtn[b].press(false); // tell the button it is NOT pressed
    }

    // Check if any tftBtn has changed state
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {

      if (b < 6)
        tft.setFreeFont(LABEL1_FONT);
      else
        tft.setFreeFont(LABEL2_FONT);

      if (tftBtn[b].justReleased())
      {
        tftBtn[b].drawButton(); // draw normal
        drawMultilineBtnLabel(tftBtnPos[b][0], tftBtnPos[b][1], BTN_WIDTH, BTN_HEIGHT, TFT_TEXT_COLOR, BTN_TEXTSCALING, b);
      }

      if (tftBtn[b].justPressed())
      {
        tftBtn[b].drawButton(true); // draw inverted
        drawMultilineBtnLabel(tftBtnPos[b][0], tftBtnPos[b][1], BTN_WIDTH, BTN_HEIGHT, TFT_TEXT_COLOR, BTN_TEXTSCALING, b, true);
        switch (b)
        {
        case 0:
        {
          sendStatus("Singles Left-Hand side Service", "lightblue");
          handlePayload(SinglesLHServicePreset, strlen(SinglesLHServicePreset));
          break;
        }
        case 1:
          sendStatus("Doubles Left-Hand side Service", "blue");
          handlePayload(DoublesLHServicePreset, strlen(DoublesLHServicePreset));
          break;
        case 2:
        {
          sendStatus("Singles Right-Hand side Service", "lightblue");
          handlePayload(SinglesRHServicePreset, strlen(SinglesRHServicePreset));
          break;
        }
        case 3:
          sendStatus("Doubles Right-Hand side Service", "blue");
          handlePayload(DoublesRHServicePreset, strlen(DoublesRHServicePreset));
          break;
        case 4:
          sendStatus("Singles Full Court", "lightblue");
          handlePayload(SinglesFullCourtPreset, strlen(SinglesFullCourtPreset));
          break;
        case 5:
          sendStatus("Doubles Full Court", "blue");
          handlePayload(DoublesFullCourtPreset, strlen(DoublesFullCourtPreset));
          break;
        case 6:
          loadAndStartUserPreset();
          break;
        case 7:
          sendStatus("Doubles No Rotation", "blue");
          handlePayload(DoublesNoRotationPreset, strlen(DoublesNoRotationPreset));
          break;
        }
      }
    }
  }

  //////////////// TFT Keypad ////////////////////

  switch (system_state)
  {
  case ST_IDLE:
    last_state_time = current_time;
    countdown_required = true;
    count_down = 5;
    current_sequence_index = 0;

    break;
  case ST_START:
    skip_open_delay = false;
    shuttle_drop_attempts = 0;
    stop_launcher = false;
    servoPusher.write(pusher_catch_pos);

    // If a countdown is required do not add additional delay before the shot.
    if (countdown_required == true)
      last_shot_delay = 0;

    // If a countdown is not required, or if we've counted down to 2s remaining (feed prep length)
    if (countdown_required == false || current_time - last_state_time > (5000 - 2000))
    {
      Serial.print("Starting");

      if (ACTIVE_PLAY_TYPE == RANDOM_SHOTS)
      {
        current_shot = random(0, active_shot_array_length); // lower bound inclusive, upper bound exclusive.
        current_shot_delay = (int)(default_delay * 1000);   // This should be based on the default shot delay.
      }
      if (ACTIVE_PLAY_TYPE == RANDOM_ORDER)
      {
        current_sequence_index = random(0, sequence_length); // lower bound inclusive, upper bound exclusive.
        current_shot = shot_sequence[current_sequence_index].ID;
        current_shot_delay = (int)(shot_sequence[current_sequence_index].delay * 1000);
      }
      if (ACTIVE_PLAY_TYPE == PLAY_ONCE || ACTIVE_PLAY_TYPE == REPEAT_SHOTS)
      {
        current_shot = shot_sequence[current_sequence_index].ID;
        current_shot_delay = (int)(shot_sequence[current_sequence_index].delay * 1000);
      }
      // Subtract the delay built into the shot sequence. Approximately 2 second delay plus the time it takes to push into the feeders.
      // current_shot_delay -= 2100;
      // current_shot_delay = max(0,current_shot_delay);

      current_trajectory = active_shots[current_shot].trajectory;
      current_rotation = active_shots[current_shot].rotation;
      current_lm_speed = active_shots[current_shot].left_motor_rpm;
      current_rm_speed = active_shots[current_shot].right_motor_rpm;

      // Serial.printf("shot index:%i   x: %f   y: %f   traj: %f   rot: %f   lm: %i   rm: %i" ,current_shot,active_shots[current_shot].court_x_pos,active_shots[current_shot].court_y_pos,active_shots[current_shot].trajectory,active_shots[current_shot].rotation,active_shots[current_shot].left_motor_rpm,active_shots[current_shot].right_motor_rpm);
      // Serial.println("");

      last_state_time = current_time;
      start_time = current_time;
      system_state = ST_GRIPPER_OPEN;
    }
    break;
  case ST_GRIPPER_OPEN:
    if (current_time - last_state_time >= 500 || skip_open_delay)
    {
      Serial.print(" -> Gripper Opening");
      servoGripper.write(gripper_open_pos);
      skip_open_delay = false;
      shuttle_drop_attempts++;
      last_state_time = current_time;
      system_state = ST_GRIPPER_CLOSED;
    }
    break;
  case ST_GRIPPER_CLOSED:
    if (current_time - last_state_time >= 500)
    {
      Serial.print(" -> Gripper Closing");
      servoGripper.write(gripper_closed_pos);
      last_state_time = current_time;
      system_state = ST_SHUTTLE_CHECK;
    }
    break;
  case ST_SHUTTLE_CHECK:
    Serial.print(" -> Shuttle Check");
    if (throughBeamClear == true && shuttle_check == true) // If the through beam sensor is not blocked and shuttle checking is enabled the either try again or stop.
    {
      // If there is no shuttle loaded try cycling the gripper.
      if (shuttle_drop_attempts < 2)
      {
        skip_open_delay = true;
        system_state = ST_GRIPPER_OPEN;
        // current_shot_delay -= 500; //Subtract the extra time to open the gripper from the current requested shot delay.
        // current_shot_delay = max(0,current_shot_delay);
      }
      else
      {
        sendStatus("Shuttle Feed Failure", "red");
        stop_launcher = true;
        system_state = ST_RESET;
      }
    }
    else
    {
      system_state = ST_PREP_LAUNCH;
    }
    last_state_time = current_time;
    break;
  case ST_PREP_LAUNCH:
    if (current_time - last_state_time >= 500 && current_time - start_time >= last_shot_delay - 1000) // wait at least 500ms, but if the delay is longer wait until 500ms before the shot.
    {
      Serial.print(" -> Launch Prep");
      sendStatus("Feeding", "green");
      // Move to the desired position & start the motors.
      // Serial.printf("Feed Shuttle   LM: %i   RM: %i   Trajectory: %f   Rotation: %f", current_lm_speed, current_rm_speed, current_trajectory, current_rotation);
      // Serial.println("");

      // At lower speeds first start with a higher value to get the motor spinning.
      if (current_lm_speed < 2500 || current_rm_speed < 2500)
      {
        servoLeftESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
        servoRightESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
        delay(100);
      }

      servoLeftESC.writeMicroseconds(map(current_lm_speed, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
      servoRightESC.writeMicroseconds(map(current_rm_speed, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint, escHighPulseSetPoint));
      traj_servo_pos = getTrajectoryServoPos(current_trajectory);
      rot_servo_pos = getRotationServoPos(current_rotation);

      // Serial.printf("Trajectory Servo: %f   Rotation Servo: %f", traj_servo_pos,rot_servo_pos);
      // Serial.println("");
      servoTrajectory.write(traj_servo_pos);
      servoRotation.write(rot_servo_pos);
      last_state_time = current_time;
      system_state = ST_LAUNCH;
    }
    break;
  case ST_LAUNCH:
    if (current_time - last_state_time >= 1000 && current_time - start_time >= last_shot_delay) // Minimum of 500 to allow motors to reach speed.
    {
      Serial.print(" -> Launching");
      servoPusher.write(pusher_push_pos);
      last_state_time = current_time;
      system_state = ST_RESET;
    }
    break;
  case ST_RESET:

    if (current_time - last_state_time >= 1000)
    {
      Serial.print(" -> Resetting");
      Serial.printf(" | Details | index:%i   x: %f   y: %f   traj: %f   traj_servo: %f   rot: %f   rot_servo: %f   lm: %i   rm: %i", current_shot, active_shots[current_shot].court_x_pos, active_shots[current_shot].court_y_pos, active_shots[current_shot].trajectory, traj_servo_pos, active_shots[current_shot].rotation, rot_servo_pos, active_shots[current_shot].left_motor_rpm, active_shots[current_shot].right_motor_rpm);
      Serial.println("");
      servoLeftESC.writeMicroseconds(escMinPulseLength);
      servoRightESC.writeMicroseconds(escMinPulseLength);
      servoPusher.write(pusher_catch_pos);
      servoGripper.write(gripper_closed_pos);

      last_state_time = current_time;

      // Store the delay attached to the current shot to time the next shot.
      last_shot_delay = current_shot_delay;

      if (stop_launcher == true)
      {
        sendStatus("Stopping", "orange");
        system_state = ST_IDLE;
      }
      else
      {
        // plan to start the next shot
        system_state = ST_START;
        current_sequence_index = current_sequence_index + 1;

        if (current_sequence_index >= sequence_length)
        {
          current_sequence_index = 0;
          // If the pattern is only to be played once then stop.
          if (ACTIVE_PLAY_TYPE == PLAY_ONCE)
          {
            sendStatus("Stopping", "orange");
            system_state = ST_IDLE;
          }
        }
      }
    }
    break;
  }
}

// Prevent any further shuttle feeds being initialized then stop the esc's.
void stopAll()
{
  // Serial.println("Stop All");
  sendStatus("Stopping", "orange");
  stop_launcher = true;
  if (system_state != ST_RESET && system_state != ST_IDLE)
  {
    system_state = ST_RESET;
  }
}

// Get the required wheel speeds for a given court position
void getWheelSpeeds(float rotation, float x, float y, float v0, float &leftWheelRPM, float &rightWheelRPM)
{
  float launcherX = 0;
  float launcherY = -2;
  float requiredRotation = atan((x - launcherX) / (y - launcherY)) / PI * 180; // In degrees
  // float shotFinalRotation = parseFloat(shots[i].rotation)+ min(max(requiredRotation-shots[i].rotation,-maxWheelCorrection),maxWheelCorrection);
  float wheelImpartAngle = requiredRotation - rotation; // In degrees
  float wheelRadius = 104 / 1000 / 2;                   // In m

  // float wheelCenterToCenter = 125;
  // float shuttleRadius = 26.5/1000/2; //25-28mm diameter
  float releaseAngle = 16.7; // Solved graphically.

  releaseAngle = releaseAngle / 180 * PI;         // Convert to radians
  wheelImpartAngle = wheelImpartAngle / 180 * PI; // Convert to radians

  // When we consider the shuttle center we must consider that a speed mismatch results in a rolling situation.
  // This gives the expected result at matched speeds as equal roller speeds will have the shuttle traveling at almost the wheel velocity.
  // float velYComponent = v1*cos(releaseAngle)/2 + v2*cos(releaseAngle)/2 //Component going directly away from the launcher.
  // float velXComponent = v1*sin(releaseAngle)/2 - v2*sin(releaseAngle)/2 //Component going awway from the launcher center-line.
  // float angle = atan(velXComponent/velYComponent);

  // Rearange
  // float velYComponent*2/cos(releaseAngle) = v1 + v2 //Component going directly away from the launcher.
  // float velXComponent*2/sin(releaseAngle) = v1 - v2 //Component going awway from the launcher center-line.

  // Rearange & Substitute
  // v1 = velYComponent*2/cos(releaseAngle) - v2
  // v2 =  v1 - velXComponent*2/sin(releaseAngle)
  // Substitute
  // v1 = velYComponent*2/cos(releaseAngle)  + velXComponent*2/sin(releaseAngle)
  // v1 = (velYComponent*2/cos(releaseAngle)  + velXComponent*2/sin(releaseAngle))/2

  // The velYComponent component must be V0 to achieve the required shuttle distance.
  float velYComponent = v0;
  float velXComponent = tan(wheelImpartAngle) * v0;
  float v1 = (velYComponent * 2 / cos(releaseAngle) + velXComponent * 2 / sin(releaseAngle)) / 2;
  float v2 = v1 - velXComponent * 2 / sin(releaseAngle);

  // This will update the variables passed to the function.
  rightWheelRPM = v1 / wheelRadius / (2 * PI) * 60;
  leftWheelRPM = v2 / wheelRadius / (2 * PI) * 60;
}

// Get the required servo angle to set the motor mount plate at the desired trajectory angle.
int getTrajectoryServoPos(float inputAngle)
{
  // Angle limits based on linkage and geometry.
  if (inputAngle < -4)
    inputAngle = -4;
  if (inputAngle > 40)
    inputAngle = 40;
  // Serial.printf("Trajectory Input: %f", inputAngle);
  // Serial.println("");

  float pivotLinkDistance = 20; // Distance from motor plate pivot to driving linkage pin.
  float pivotLinkAngle = -11.5; // Angle between motor plate horizontal and line between pivot and linkage pin. D3 was -14.5.
  float linkLength = 51;        // D3 was 45
  float servoPosX = 54;         // D3 was 43
  float servoPosY = 25;         // D3 was 30
  float servoHornLength = 7.2;  // D3 was 7.5

  float effectiveAngle = radians(pivotLinkAngle - inputAngle);
  float x1 = pivotLinkDistance * cos(effectiveAngle);
  float y1 = pivotLinkDistance * sin(effectiveAngle);
  float c = sqrt(pow(servoPosX - x1, 2) + pow(servoPosY - y1, 2)); // the distance between the motor mount linkage pin to the servo center.
  float beta = atan((servoPosX - x1) / (servoPosY - y1));          // The angle between vertical and the line C
  float alpha = acos((pow(linkLength, 2) - pow(c, 2) - pow(servoHornLength, 2)) / (-2 * c * servoHornLength));
  // float servoAngle = PI - (alpha + beta); //Servo zero is top dead center.
  float servoAngle = PI / 4 - beta + alpha; // Servo zero is 45 degrees off of vertical pointing towards the front. Approx. in line with linkage.
  // Serial.printf("x1: %f   y1: %f   c: %f   beta: %f   alpha: %f   servoAngle: %f   ", x1, y1, c, beta, alpha, servoAngle);
  // Serial.println("");
  servoAngle = degrees(servoAngle); // convert to degrees

  servoAngle += traj_servo_correction;

  // Set hard limits based on servo values.
  if (servoAngle < 0)
    servoAngle = 0;
  if (servoAngle > 180)
    servoAngle = 180;
  if (isnan(servoAngle))
    servoAngle = 0;
  // Serial.printf("Trajectory Servo: %f", servoAngle);
  // Serial.println("");
  return servoAngle;
}

// Get the required servo angle to set the chute at the desired rotation angle.
int getRotationServoPos(float inputAngle)
{

  // Angle limits based on linkage and geometry.
  if (inputAngle < -19)
    inputAngle = -19.000;
  if (inputAngle > 19)
    inputAngle = 19.000;
  // Serial.println("");
  // Serial.print(F("Rotation angle: requested "));
  // Serial.print(inputAngle);
  // Serial.print(F("°"));

  inputAngle = radians(inputAngle);                                  // calculate angle in radians
  float a1 = 73.345;                                                 // Chute rotation center to servo center
  float a2 = 24.000;                                                 // Servo horn arm controlling rotation
  float servoAngle = inputAngle - asin((a1 * sin(inputAngle)) / a2); // calculate servo angle in radians
  servoAngle = 90 + degrees(servoAngle);                             // convert to degrees
  // Serial.print(F(" | calculated "));
  // Serial.print(servoAngle);
  // Serial.print(F("°"));

  // Set hard limits based on servo values.
  if (servoAngle < right_rotation_limit)
    servoAngle = right_rotation_limit;
  if (servoAngle > left_rotation_limit)
    servoAngle = left_rotation_limit;
  if (isnan(servoAngle))
    servoAngle = 0;
  // Serial.print(F(" | limited "));
  // Serial.print(servoAngle);
  // Serial.println("°");

  return servoAngle;
}
