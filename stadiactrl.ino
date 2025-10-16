#include <TaskScheduler.h>
#include <Bluepad32.h>
#include "uni.h"

#define WEAPON_MIN 600
#define WEAPON_MAX 2400
#define FAILSAFE_TIMEOUT 2000

ControllerPtr myControllers[BP32_MAX_GAMEPADS];                   //library allows up to four controllers
static const char* controller_addr_string = "FE:AD:C2:BA:1C:DE";  //only allow specified MAC to connect, change to match your controller

HardwareSerial ibusSer(0);

Scheduler ts;

void readRadioCB();
void sendIbusCB();

Task readRadio(0, TASK_FOREVER, &readRadioCB, &ts);
Task sendIbus(7, TASK_FOREVER, &sendIbusCB, &ts); //144hz to match typical IBUS update rate

enum RadioState_t {
  RADIO_DC,
  RADIO_OK,
} radioState;

uint8_t ibusPacket[32];

void setup() {
  Serial.begin(115200);                       //debug serial
  ibusSer.begin(115200, SERIAL_8N1, -1, -1);  //hardware serial on default pins, TX == D6 on Xiao ESP32C3
  delay(1000);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  //force pairing every boot

  bd_addr_t controller_addr;
  sscanf_bd_addr(controller_addr_string, controller_addr);
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);

  ts.enableAll();
}

void loop() {
  ts.execute();
  yield();
}

void processGamepad(ControllerPtr ctl) {
  uint16_t fwd = ctl->throttle(); //right trigger
  uint16_t rev = ctl->brake();    //left trigger

  uint16_t throttle;
  if (fwd > 10) throttle = map(fwd, 10, 1020, 1500, 2000);
  else if (rev > 10) throttle = map(rev, 10, 1020, 1500, 1200);
  else throttle = 1500;

  //left stick X
  uint16_t steering = map(ctl->axisX(), -512, 512, 1200, 1800);
  if ((steering < 1550) && (steering > 1450)) steering = 1500;  //deadzone

  //weapon (servo) trim
  static uint16_t weapHome = WEAPON_MIN;
  
  //right bumper
  if (ctl->buttons() & 0x0020) {
    weapHome = weapHome + 10;
  }
  //left bumper
  if (ctl->buttons() & 0x0010) {
    weapHome = weapHome - 10;
  }

  //A button
  uint16_t weapon;
  if (ctl->buttons() & 0x0001) {
    weapon = 2400;
  } else weapon = weapHome;

 Serial.printf("S: %d T: %d W %d\n", steering, throttle, weapon);

  //header
  ibusPacket[0] = 0x20;
  ibusPacket[1] = 0x40;
  //channels 1-4 (ibus supports up to 14 channels)
  ibusPacket[2] = lowByte(steering);
  ibusPacket[3] = highByte(steering);
  ibusPacket[4] = lowByte(throttle);
  ibusPacket[5] = highByte(throttle);
  ibusPacket[6] = 0;
  ibusPacket[7] = 0;
  ibusPacket[8] = lowByte(weapon);
  ibusPacket[9] = highByte(weapon);

  uint16_t checksum = 0;

  for (uint8_t i = 0; i <= 15; i++) {
    checksum = checksum + ibusPacket[i];
  }

  checksum = 0xFFFF - checksum;
  ibusPacket[30] = lowByte(checksum);
  ibusPacket[31] = highByte(checksum);

  //dumpGamepad(ctl);
}

void sendIbusCB() {
  if (radioState == RADIO_OK) {
    for (uint8_t i = 0; i < (sizeof(ibusPacket) / sizeof(ibusPacket[0])); i++) {
      ibusSer.write(ibusPacket[i]);
    }
  }
}

void readRadioCB() {
  static uint32_t lastRadioTime;

  bool dataUpdated = BP32.update();

  if ((dataUpdated) && (myControllers[0] && myControllers[0]->isConnected() && myControllers[0]->hasData())) {
    radioState = RADIO_OK;
    lastRadioTime = millis();
    processGamepad(myControllers[0]);
  }

  if (millis() > (lastRadioTime + FAILSAFE_TIMEOUT)) {
    radioState = RADIO_DC;
    lastRadioTime = millis();
  }
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  if (myControllers[0] == nullptr) {
    Serial.printf("CALLBACK: Controller is connected, index=0\n");
    // Additionally, you can get certain gamepad properties like:
    // Model, VID, PID, BTAddr, flags, etc.
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
    myControllers[0] = ctl;
  }
}

void onDisconnectedController(ControllerPtr ctl) {

  if (myControllers[0] == ctl) {
    Serial.printf("CALLBACK: Controller disconnected from index=0\n");
    myControllers[0] = nullptr;
  }
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}


void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}
