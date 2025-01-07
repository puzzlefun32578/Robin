#include <BLEDevice.h>  //this makes required library of functions available
#include <BLEUtils.h>
#include <BLEServer.h>
#include <FIR.h>
#include <driver/rtc_io.h>

#define SERVICE_UUID        "ebdddb72-32f6-495f-aaf1-358ddba09f46"
#define CONTROL_UUID        "ebdddb73-32f6-495f-aaf1-358ddba09f46"
#define SOC_UUID            "ebdddb74-32f6-495f-aaf1-358ddba09f46"

#define X_PIN A10
#define Y_PIN A9
#define BUTTON_PIN A8

const unsigned int debounce_time_ms = 5;
float soc = 3.7;
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
unsigned long inactivity_timer = 0;
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

  rtc_gpio_deinit(GPIO_NUM_7);
}

void loop() {
  for(int i = 0; i < 5; i++){
    int right = (analogRead(Y_PIN)/16.0)-128;
    int forward = (analogRead(X_PIN)/16.0)-128;
    stick_pos.right = right_fir.processReading(right);
    stick_pos.forward = forward_fir.processReading(forward);
  }

  command.right=stick_pos.right;
  if (cruise == true) {
    if (stick_pos.forward > 64) command.forward+=2;  //increase speed 
    else if (stick_pos.forward < -64) command.forward-=2;
  }
  else {
    command.forward=stick_pos.forward/2;
  }
  
  char buff[8];
  sprintf(buff, "%+04d%+04d", command.forward, command.right);
  Serial.println(buff);
  pControlChar->setValue(buff);
  pControlChar->notify();
  
  if (command.forward > 5 )  { inactivity_timer = millis() ; }   // keep track of the last command to determine when to go to sleep 
  Serial.println( millis() - inactivity_timer );
   if (millis() > inactivity_timer + 300000)  {
    esp_bt_controller_disable();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_7,0);  //use the button pin as the external trigger for waking up; A8 = GPIO7; it is held high, so low=pressed
    rtc_gpio_pullup_en(GPIO_NUM_7);
    //Serial.println("going to sleep now");
    esp_deep_sleep_start();
  }
  delay(100);
 
}
