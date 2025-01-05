#include <BLEDevice.h>  //this makes required library of functions available
#include <BLEUtils.h>
#include <BLEServer.h>
#include <FIR.h>

#define SERVICE_UUID        "ebdddb72-32f6-495f-aaf1-358ddba09f46"
#define CONTROL_UUID        "ebdddb73-32f6-495f-aaf1-358ddba09f46"
#define SOC_UUID            "ebdddb74-32f6-495f-aaf1-358ddba09f46"

#define X_PIN A10
#define Y_PIN A9
#define BUTTON_PIN A8

const unsigned int debounce_time_ms = 5;

BLECharacteristic *pControlChar;

FIR<float, 5> right_fir;
FIR<float, 5> forward_fir;

struct {
  int8_t forward = 0;
  int8_t right = 0;
} command, stick_pos;

unsigned long last_button_down;
volatile bool cruise = false;
volatile bool last_down = false;
int inactivity_timer;

void buttonChange(){
  bool button_down = !digitalRead(BUTTON_PIN);
  if(button_down == last_down) return;
  if(button_down){
    last_button_down = millis();
    last_down = true;
  }
  else{
    unsigned long duration_ms = millis()-last_button_down;
    if(duration_ms >2000) esp_restart();
    else if(duration_ms > debounce_time_ms) cruise = !cruise;
    last_down = false;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Robin starting");

  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(BUTTON_PIN, buttonChange, CHANGE);
  
  BLEDevice::init("Robin");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pControlChar = pService->createCharacteristic(
                                         CONTROL_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  BLECharacteristic *pSocChar = pService->createCharacteristic(
                                         SOC_UUID,
                                         BLECharacteristic::PROPERTY_READ
                                       );
  float soc = 3.7;
  pSocChar->setValue(String(soc));

  pControlChar->setValue("+000+000");

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Bluetooth started");

  float coef[5] = {1., 1., 1., 1., 1.};
  right_fir.setFilterCoeffs(coef);
  forward_fir.setFilterCoeffs(coef);

  esp_sleep_enable_ext0_wakeup(X_PIN,1);  //use the button pin as the external trigger for waking up
              // specify button pin as a RTC input so it can be used to awaken from deep sleep
}

void loop() {
  for(int i = 0; i < 5; i++){
    int right = (analogRead(Y_PIN)/16.0)-128;
    int forward = (analogRead(X_PIN)/16.0)-128;
    // right = right + 3;      // correct zero bias in joystick
    // forward = forward + 3;   // correct zero bias in joystick
    stick_pos.right = right_fir.processReading(right);
    stick_pos.forward = forward_fir.processReading(forward);
  }

  if (abs(stick_pos.forward > 5))  { inactivity_timer == millis() ; }   // keep track of the last stick input to determine when to go to sleep 
  if (abs(stick_pos.right) > 5)  { inactivity_timer == millis() ;}
  if (millis() > inactivity_timer + 300000) ; {
     rtc_gpio_pullup_en(BUTTON_PIN);
     esp_deep_sleep_start();
  }
  
  command.right = stick_pos.right / 2;  
  if (cruise == true) {
    if (stick_pos.forward > 64) command.forward+=2;  //increase speed 
    else if (stick_pos.forward < -64) command.forward-=2;
  }
  else {
    command.forward = stick_pos.forward / 2;
  }
  
  char buff[8];
  sprintf(buff, "%+04d%+04d", command.forward, command.right);
  Serial.println(buff);
  pControlChar->setValue(buff);
  pControlChar->notify();
  // Update battery voltage characteristic
  // Read X and Y values from TrackPoint PS/2
  // Update control string with X and Y values
  // Set and notify BLE
  // Enter deep sleep after inactivity period - wakeup on button press?
  delay(100);
}
