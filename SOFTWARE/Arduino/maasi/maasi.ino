#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "Arduino.h"
#include "esp32-hal.h"
#include "nextion_hmi.h"
#include <elapsedMillis.h>
#include <PID_v1.h>


#define DEFAULT_DISPENSE_SPEED 600       // RPM
#define DEFAULT_COAT_SPEED 3000          
#define MIN_COAT_SPEED 300
#define MAX_COAT_SPEED 8000
#define INCREMETS_COAT_SPEED 100



#define DEFAULT_COAT_TIME 30000 // In ms
#define MIN_COAT_TIME 1000
#define MAX_COAT_TIME 80000
#define INCREMETS_COAT_TIME 1000
#define DEFAULT_RAMPUP_TIME 3000
#define MIN_RAMPUP_TIME 1000
#define MAX_RAMPUP_TIME 6000
#define DEFAULT_RAMPDOWN_TIME 1500
#define MIN_RAMPDOWN_TIME 500
#define MAX_RAMPDOWN_TIME 5000

#define MOTOR_POLE_PAIRS 7

#define GREY 54970
#define GREEN 17864


//---------------------------------------
//  Coating params
//---------------------------------------

uint16_t disp_speed, coat_speed;

int32_t rampup_time, rampdown_time, coat_time;

enum state {
  STATE_IDLE,
  STATE_DISPENSING,
  STATE_RAMPUP,
  STATE_RAMPDOWN,
  STATE_COATING
};

state current_state, previous_state = STATE_IDLE;

//----------------------------------------------------------------------------
// Espressif Infrared Remote(rmt) library is used to create DSHOT messages in a fast manner
//-----------------------------------------------------------------------------

TaskHandle_t Task1;

rmt_data_t dshotPacket[16];
rmt_obj_t* rmt_send = NULL;

hw_timer_t * timer = NULL;

HardwareSerial TelSerial(1);

// ESC Telemetry variables
uint8_t receivedBytes = 0;
volatile bool requestTelemetry = false;
int16_t ESC_telemetry[5]; // Temperature, Voltage, Current, used mAh, eRpM
//uint32_t telemetry_fails = 0;
//uint32_t telemetry_succ = 0;


uint32_t currentTime;
uint8_t temperature = 0;
uint8_t temperatureMax = 0;
float voltage = 0;
float voltageMin = 99;
uint32_t current = 0;
uint32_t currentMax = 0;
uint32_t erpm = 0;
uint32_t erpmMax = 0;
uint32_t rpm = 0;
uint32_t rpmMAX = 0;
uint32_t kv = 0;
uint32_t kvMax = 0;

//---------------------------------------
//   PID parameters
//---------------------------------------
double motor_setRPM, motor_actualRPM, pid_output = 0;
double pid_out_min = 48;
double pid_out_max = 2047;
double Kp = 0.015, Ki = 0.8 , Kd = 0.0; // Before 1,3,0
const int sample_time = 1; //ms


PID motorPID(&motor_actualRPM, &pid_output, &motor_setRPM, Kp, Ki, Kd, DIRECT);


//---------------------------------------
//   ESC Control Parameters
//---------------------------------------

uint16_t dshotUserInputValue = 0;
uint16_t dshotmin = 48;
uint16_t dshotmax = 2047;

void IRAM_ATTR getTelemetry() {
  requestTelemetry = true;
}

void startTelemetryTimer() {
  timer = timerBegin(0, 80, true); // timer_id = 0; divider=80; countUp = true;
  timerAttachInterrupt(timer, &getTelemetry, true); // edge = true
  timerAlarmWrite(timer, 1000, true);  //1000 = 1 ms
  timerAlarmEnable(timer);
}

// Second core used to send DSHOT packets
void secondCoreTask( void * pvParameters ) {
  while (1) {

    if (requestTelemetry) {
      dshotOutput(dshotUserInputValue, requestTelemetry);
      requestTelemetry = false;
      receivedBytes = 0;
    }

    delay(1);
  }
}

//---------------------------------------
//   Nextion Callbacks
//---------------------------------------


 // START
 
void b1PushCallback(void *ptr)
{
    startDispense();
}

 // STOP
 
void p2b0PushCallback(void *ptr)
{
  if (current_state != STATE_RAMPDOWN){
      startRampdown();    
    }
}

 // COAT
void p2b1PushCallback(void *ptr)
{
  if (current_state == STATE_DISPENSING){
      startRampup();    
    }
}

 // SET BUTTONS
 
void b2PushCallback(void *ptr)
{
   if (coat_time > MIN_COAT_TIME){
    coat_time -= INCREMETS_COAT_TIME;
    }
   n0.setValue(coat_time/1000);

}

void b3PushCallback(void *ptr)
{
    if (coat_time < MAX_COAT_TIME){
    coat_time += INCREMETS_COAT_TIME;
    }
    n0.setValue(coat_time/1000);
}

void b4PushCallback(void *ptr)
{
  if (coat_speed > MIN_COAT_SPEED){
    coat_speed -= INCREMETS_COAT_SPEED;
    }
    n1.setValue(coat_speed);
}

void b5PushCallback(void *ptr)
{
    if (coat_speed < MAX_COAT_SPEED){
    coat_speed += INCREMETS_COAT_SPEED;
    }
    n1.setValue(coat_speed); 
}

void b6PushCallback(void *ptr)
{
    p1.show();
    p1n0.setValue(disp_speed);
    p1n1.setValue(rampup_time/1000);     
    p1n2.setValue(rampdown_time);  
}

void p1b0PushCallback(void *ptr)
{
   if (disp_speed > MIN_COAT_SPEED){
    disp_speed -= INCREMETS_COAT_SPEED/2;
    }
   p1n0.setValue(disp_speed);

}

void p1b1PushCallback(void *ptr)
{
    if (disp_speed < MAX_COAT_SPEED){
    disp_speed += INCREMETS_COAT_SPEED/2;
    }
    p1n0.setValue(disp_speed);
}

void p1b2PushCallback(void *ptr)
{
  if (rampup_time > MIN_RAMPUP_TIME){
    rampup_time -= INCREMETS_COAT_TIME;
    }
    p1n1.setValue(rampup_time/1000);
}

void p1b3PushCallback(void *ptr)
{
    if (rampup_time < MAX_RAMPUP_TIME){
    rampup_time += INCREMETS_COAT_TIME;
    }
    p1n1.setValue(rampup_time/1000); 
}

void p1b4PushCallback(void *ptr)
{
  if (rampdown_time > MIN_RAMPDOWN_TIME){
    rampdown_time -= INCREMETS_COAT_TIME/2;
    }
    p1n2.setValue(rampdown_time);
}

void p1b5PushCallback(void *ptr)
{
  if (rampdown_time < MAX_RAMPDOWN_TIME){
    rampdown_time += INCREMETS_COAT_TIME/2;
    }
    p1n2.setValue(rampdown_time);
}

void p1b6PushCallback(void *ptr)
{

}

void p1b7PushCallback(void *ptr)
{

}
//---------------------------------------
//   Timers
//---------------------------------------
elapsedMillis time_since_display_update = 0;

elapsedMillis time_ramping = 0;
elapsedMillis time_coating = 0;

//elapsedMicros time_since_telemetry = 0;

const int display_update_time = 100;



//---------------------------------------
//   SETUP
//---------------------------------------

void setup() {

  Serial.begin(115200);
  TelSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Init DSHOT hardware
  if ((rmt_send = rmtInit(5, true, RMT_MEM_64)) == NULL) {
    Serial.println("init sender failed\n");
  }

  float realTick = rmtSetTick(rmt_send, 12.5); // 12.5ns sample rate
  //Serial.printf("rmt_send tick set to: %fns\n", realTick);

  // Output disarm signal while esc initialises and do some display stuff.
  uint8_t xbeep = random(15, 100);
  uint8_t ybeep = random(15, 50);
  uint8_t ibeep = 0;
  while (millis() < 3500) {
    dshotOutput(0, false);
    delay(1);

    /// display.clear();
    ibeep++;
    if (ibeep == 100) {
      ibeep = 0;
      xbeep = random(15, 50);
      ybeep = random(15, 50);
    }
  }


  // Empty Rx Serial of garbage telemtry
  while (TelSerial.available()){
    TelSerial.read();
    }
  requestTelemetry = true;
  startTelemetryTimer(); // Timer used to request tlm continually in case ESC rcv bad packet
  xTaskCreatePinnedToCore(secondCoreTask, "Task1", 10000, NULL, 1, &Task1, 0);


  //PID
  coat_speed = DEFAULT_COAT_SPEED;
  disp_speed = DEFAULT_DISPENSE_SPEED;
  coat_time = DEFAULT_COAT_TIME;
  rampup_time = DEFAULT_RAMPUP_TIME;
  rampdown_time = DEFAULT_RAMPDOWN_TIME;
  
  motorPID.SetOutputLimits(pid_out_min, pid_out_max);
  motorPID.SetSampleTime(sample_time);
  
  // Touchscreen init
  nexInit();
  b1.attachPush(b1PushCallback);
  b2.attachPush(b2PushCallback);
  b3.attachPush(b3PushCallback);
  b4.attachPush(b4PushCallback);
  b5.attachPush(b5PushCallback);
  b6.attachPush(b6PushCallback); 
  p1b0.attachPush(p1b0PushCallback);
  p1b1.attachPush(p1b1PushCallback);
  p1b2.attachPush(p1b2PushCallback);
  p1b3.attachPush(p1b3PushCallback);
  p1b4.attachPush(p1b4PushCallback);
  p1b5.attachPush(p1b5PushCallback);
  p1b6.attachPush(p1b6PushCallback);
  p1b7.attachPush(p1b7PushCallback); 
  p2b0.attachPush(p2b0PushCallback);
  p2b1.attachPush(p2b1PushCallback);

  initDisplay();


}


//---------------------------------------
//   LOOP
//---------------------------------------
void loop() {

  nexLoop(nex_listen_list); //Check callback list
  updateDisplay();
  if (!requestTelemetry) {
    motorPID.Compute();
    receiveTelemtry();
  }
  updateMotor();
}

void stopCoating(){
  p0.show();
  current_state = STATE_IDLE;
  }
  
void startDispense(){
  p2.show();
  p2b1.Set_background_color_bco(GREEN); //Set button Green
  t4.setText("Dispense fluid");
  current_state = STATE_DISPENSING;
  
  motor_setRPM = disp_speed;
  motorPID.SetMode(AUTOMATIC); 
  }
  
void startRampup(){
  p2b1.Set_background_color_bco(GREY); //Set button Grey
  t4.setText("Ramping up");
  current_state = STATE_RAMPUP;
  
  motorPID.SetMode(AUTOMATIC);
  time_ramping = 0;
  }
  
void startRampdown(){
  t4.setText("Ramping down");
  current_state = STATE_RAMPDOWN;
  
  motorPID.SetMode(AUTOMATIC);
  time_ramping = 0; 
  }
  
void startCoating(){
  //p2.show();
  //p2b1.Set_background_color_bco(GREY); //Set button Grey
  t4.setText("Coating");
  current_state = STATE_COATING;
 
  motor_setRPM = coat_speed; 
  motorPID.SetMode(AUTOMATIC); 
  time_coating = 0;
  }

void initDisplay(){
    n0.setValue(coat_time/1000);
    n1.setValue(coat_speed);
    p1n0.setValue(disp_speed);
    p1n1.setValue(rampup_time/1000);
    p1n2.setValue(rampdown_time);
  }

void updateDisplay() {

  if (time_since_display_update >= display_update_time) {
    time_since_display_update = 0;

    switch (current_state){
      case STATE_IDLE:  
        break;
        
      case STATE_DISPENSING:
        p2n0.setValue(coat_time/1000);
        p2n1.setValue(motor_actualRPM);
        p2j0.setValue(0);
        break;
      case STATE_RAMPUP:      
        p2n0.setValue((rampup_time - time_ramping)/1000);
        p2n1.setValue(motor_actualRPM);
        //p2j0.setValue(100-(rampup_time - time_ramping)*100/rampup_time);
        break;      
      case STATE_RAMPDOWN:      
        p2n0.setValue((rampdown_time - time_ramping)/1000);
        p2n1.setValue(motor_actualRPM);
        p2j0.setValue(0);
        break;     
      case STATE_COATING:      
        p2n0.setValue((coat_time - time_coating)/1000);
        p2n1.setValue(motor_actualRPM);
        p2j0.setValue((coat_time - time_coating)*100/coat_time); // % Progress bar
        //p2n0.setValue(abs(coat_speed - motor_actualRPM)); //Debuging RPM error 
        break;
    } 
  }
}


void updateMotor() {

    motor_actualRPM = rpm;

    
    switch (current_state){
      case STATE_IDLE:
       motorPID.SetMode(MANUAL); 
       dshotUserInputValue = 0;
       pid_output = 0;    
       break;
        
      case STATE_DISPENSING:
        dshotUserInputValue = pid_output;
        break;
        
      case STATE_RAMPUP:
        if (time_ramping < rampup_time){
          motor_setRPM = map(time_ramping, 0, rampup_time, disp_speed, coat_speed);
          dshotUserInputValue = pid_output;     
          }
        else{
          startCoating();
          }     
        break;
        
      case STATE_COATING:
        if (time_coating < coat_time){
          dshotUserInputValue = pid_output;    
          }
        else{
          startRampdown();
          }     
        break;

      case STATE_RAMPDOWN:
        int rampdown_speed = motor_actualRPM;
        if (time_ramping < rampdown_time){
          motor_setRPM = map(time_ramping, 0, rampdown_time, rampdown_speed, 0 );
          dshotUserInputValue = pid_output;     
          }
        else{
          stopCoating();
          }     
        break;

    }

}



void receiveTelemtry() {


  static uint8_t SerialBuf[10];

  if (TelSerial.available()) {
    SerialBuf[receivedBytes] = TelSerial.read();
    receivedBytes++;
  }

  if (receivedBytes > 9) { // transmission complete

    uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC

    if (crc8 != SerialBuf[9]) {
      //                Serial.println("CRC transmission failure");

      // Empty Rx Serial of garbage telemtry
      while (TelSerial.available())
        TelSerial.read();

      requestTelemetry = true;
      //telemetry_fails++;
      return; // transmission failure
    }

    // compute the received values
    ESC_telemetry[0] = SerialBuf[0]; // temperature
    ESC_telemetry[1] = (SerialBuf[1] << 8) | SerialBuf[2]; // voltage
    ESC_telemetry[2] = (SerialBuf[3] << 8) | SerialBuf[4]; // Current
    ESC_telemetry[3] = (SerialBuf[5] << 8) | SerialBuf[6]; // used mA/h
    ESC_telemetry[4] = (SerialBuf[7] << 8) | SerialBuf[8]; // eRpM *100

    //telemetry_succ++;
    requestTelemetry = true;
    // Update values and smooth

    /*
    temperature = 0.9 * temperature + 0.1 * ESC_telemetry[0];
    if (temperature > temperatureMax) {
      temperatureMax = temperature;
    }

    voltage = 0.9 * voltage + 0.1 * (ESC_telemetry[1] / 100.0);
    if (voltage < voltageMin) {
      voltageMin = voltage;
    }

    current = 0.9 * current + 0.1 * (ESC_telemetry[2] * 100);
    if (current > currentMax) {
      currentMax = current;
    }
    */
    //erpm = 0.7 * erpm + 0.3 * (ESC_telemetry[4] * 100);
    erpm = ESC_telemetry[4] * 100;
    if (erpm > erpmMax) {
      erpmMax = erpm;
    }

    rpm = erpm / (MOTOR_POLE_PAIRS);
    if (rpm > rpmMAX) {
      rpmMAX = rpm;
    }

  }

  return;
}


void dshotOutput(uint16_t value, bool telemetry) {

  uint16_t packet;

  // telemetry bit
  if (telemetry) {
    packet = (value << 1) | 1;
  } else {
    packet = (value << 1) | 0;
  }

  // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
  int csum = 0;
  int csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^=  csum_data;
    csum_data >>= 4;
  }
  csum &= 0xf;
  packet = (packet << 4) | csum;

  // durations are for dshot600
  // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
  // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
  // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
  // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
  for (int i = 0; i < 16; i++) {
    if (packet & 0x8000) {
      dshotPacket[i].level0 = 1;
      dshotPacket[i].duration0 = 100;
      dshotPacket[i].level1 = 0;
      dshotPacket[i].duration1 = 34;
    } else {
      dshotPacket[i].level0 = 1;
      dshotPacket[i].duration0 = 50;
      dshotPacket[i].level1 = 0;
      dshotPacket[i].duration1 = 84;
    }
    packet <<= 1;
  }

  rmtWrite(rmt_send, dshotPacket, 16);

  return;
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i = 0; i < 8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for ( i = 0; i < BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
