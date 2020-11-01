#include "TinyPpmGen.h"
#include "PinChangeInt.h"

#define DEBUG_RECEIVER

#define AILERON         0
#define ELEVATOR        1
#define THROTTLE        2
#define RUDDER          3

#define AILERON_IN      A1
#define ELEVATOR_IN     A2
#define THROTTLE_IN     A3
#define RUDDER_IN       A4

#define CHANNEL_NB      8

#define DEF_CH_VALUE    1100

uint8_t rc_pins[4]      = { AILERON_IN, 
                            ELEVATOR_IN, 
                            THROTTLE_IN, 
                            RUDDER_IN };
uint8_t rc_flags[4]     = { 1, 2, 4, 8  };
uint16_t rc_values[4]   = { 0, 0, 0, 0  };

volatile uint8_t        rc_shared_flags;
volatile uint16_t       rc_shared_values[4];
volatile uint32_t       rc_shared_ts[4];

uint32_t now             = 0;
uint32_t last_100hz      = 0;

void rc_channel_change(uint8_t id) {
  if (digitalRead(rc_pins[id]) == HIGH) {
    rc_shared_ts[id] = micros();
  }
  else {
    rc_shared_values[id] = (uint16_t)(micros() - rc_shared_ts[id]);
    rc_shared_flags |= rc_flags[id];
  }
}

void rc_aileron_change()  { rc_channel_change(AILERON);  }
void rc_elevator_change() { rc_channel_change(ELEVATOR); }
void rc_throttle_change() { rc_channel_change(THROTTLE); }
void rc_rudder_change()   { rc_channel_change(RUDDER);   }

void rc_setup_interrupts() {
  PCintPort::attachInterrupt(rc_pins[AILERON],  &rc_aileron_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[ELEVATOR], &rc_elevator_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[THROTTLE], &rc_throttle_change, CHANGE);
  PCintPort::attachInterrupt(rc_pins[RUDDER],   &rc_rudder_change, CHANGE);
}


void rc_process_channels() {
  static uint8_t flags;
  if (rc_shared_flags) {
    noInterrupts();
    flags = rc_shared_flags;
    
    if (flags & rc_flags[AILERON]) rc_values[AILERON]     = rc_shared_values[AILERON];
    if (flags & rc_flags[ELEVATOR]) rc_values[ELEVATOR]   = rc_shared_values[ELEVATOR];
    if (flags & rc_flags[THROTTLE]) rc_values[THROTTLE]   = rc_shared_values[THROTTLE];
    if (flags & rc_flags[RUDDER]) rc_values[RUDDER]       = rc_shared_values[RUDDER];
    rc_shared_flags = 0;
    interrupts(); 
  }
  flags = 0;
}


void rc_print_channels() {
  #ifdef DEBUG_RECEIVER
    static char str[64];
   
    sprintf(str, "AILE: %d, ELEV: %d, THRO: %d, RUDD: %d\n",
      rc_values[0], rc_values[1], rc_values[2], rc_values[3]
    );
   
    Serial.print(str); 
   #endif
}


void setup(){
  #ifdef DEBUG_RECEIVER
    Serial.begin(115200);
  #endif
  rc_setup_interrupts();
  TinyPpmGen.begin(TINY_PPM_GEN_POS_MOD, CHANNEL_NB);
}

void loop(){
  now = micros();
  rc_process_channels();
  /* 10ms ~ 100Hz */
  if( now - last_100hz >= 10000UL )
  {
    rc_print_channels();
    TinyPpmGen.setChWidth_us(1, rc_values[AILERON]); 
    TinyPpmGen.setChWidth_us(2, rc_values[ELEVATOR]); 
    TinyPpmGen.setChWidth_us(3, rc_values[THROTTLE]); 
    TinyPpmGen.setChWidth_us(4, rc_values[RUDDER]);
    TinyPpmGen.setChWidth_us(5, DEF_CH_VALUE); 
    TinyPpmGen.setChWidth_us(6, DEF_CH_VALUE);
    TinyPpmGen.setChWidth_us(7, DEF_CH_VALUE);  
    TinyPpmGen.setChWidth_us(8, DEF_CH_VALUE);
    last_100hz = now;
  }
    
  
}

