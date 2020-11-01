#include "InertialSensor.h"
#include "Compass_HMC5883L.h"
#include "DCM.h"
#include "TinyPpmReader.h"
#include "PID.h"
#include "Attitude.h"
#include "Servo.h"


#define ROLL_CH                         0
#define PITCH_CH                        1
#define COLLECTIVE_CH                   2
#define YAW_CH                          3


#define RATE_ROLL_P        		0.150f
#define RATE_ROLL_I        		0.120f
#define RATE_ROLL_D        		0.005f
#define RATE_ROLL_IMAX         	        1000

#define RATE_PITCH_P       		0.150f
#define RATE_PITCH_I       		0.120f
#define RATE_PITCH_D       		0.005f
#define RATE_PITCH_IMAX        	        1000

#define RATE_YAW_P              	0.200f
#define RATE_YAW_I              	0.020f
#define RATE_YAW_D              	0.000f
#define RATE_YAW_IMAX            	1000

#define ROLL_PITCH_INPUT_MAX            4500

#define PPM_INPUT_PIN                   10

#define MAIN_LOOP_SECONDS               0.01f

#define RC_FEEL_RP_VERY_SOFT            0
#define RC_FEEL_RP_SOFT                 25
#define RC_FEEL_RP_MEDIUM               50
#define RC_FEEL_RP_CRISP                75
#define RC_FEEL_RP_VERY_CRISP           100       

#define HELI_STAB_COLLECTIVE_MIN_DEFAULT 0
#define HELI_STAB_COLLECTIVE_MAX_DEFAULT 1000
#define ACRO_YAW_P                       4.5f

#define RC_LOW                           1100
#define RC_MID                           1500
#define RC_HIGH                          1900
#define COLLECTIVE_LOW                   0
#define COLLECTIVE_HIGH                  1000

#define PWM_LOW                          1200
#define PWM_MID                          1500
#define PWM_HIGH                         1800

InertialSensor                           ins;
Compass_HMC5883L                         compass;
DCM                                      _ahrs(ins);

HELICOPTER_PID                           pid_rate_roll(RATE_ROLL_P,RATE_ROLL_I, RATE_ROLL_D,RATE_ROLL_IMAX);
HELICOPTER_PID                           pid_rate_pitch(RATE_PITCH_P,RATE_PITCH_I,RATE_PITCH_D,RATE_PITCH_IMAX);
HELICOPTER_PID                           pid_rate_yaw(RATE_YAW_P,RATE_YAW_I,RATE_YAW_D,RATE_YAW_IMAX);

S_P                                      p_throttle_rate;
S_P                                      p_stabilize_roll;
S_P                                      p_stabilize_pitch;
S_P                                      p_stabilize_yaw;

RCServo                                  cyclic_1;
RCServo                                  cyclic_2;
RCServo                                  cyclic_3;
RCServo                                  yaw_servo;
          
uint16_t                                 myPPM[8];
uint16_t                                 yaw_raw;
float                                    SERVO_OUT[4];
uint8_t                                  temp_idx        = 0;
static uint32_t                          last_50hz       = 0;
static uint32_t                          last_100hz      = 0;
uint32_t                                 now             = 0;
float                                    heading         = 0;
int16_t                                  CCPM[4];
uint16_t                                 RC_IN           = 0;

int16_t                                  collective_pwm  = 0;
int16_t                                  roll_pwm        = 0;
int16_t                                  pitch_pwm       = 0;
int16_t                                  yaw_pwm         = 0;
int16_t                                  us_pwm          = 0;

int16_t                                  signal_1        = 0;
int16_t                                  signal_2        = 0;
int16_t                                  signal_3        = 0;
int16_t                                  signal_4        = 0;

static float                             simple_cos_yaw  = 1.0;
static float                             simple_sin_yaw  = 1.0;
float                                    rollx, pitchx;
float                                    roll_trim, pitch_trim;



Attitude                                  attitude(  p_stabilize_roll, 
                                                     p_stabilize_pitch, 
                                                     p_stabilize_yaw,
                                                     pid_rate_roll, 
                                                     pid_rate_pitch, 
                                                     pid_rate_yaw );
                    
float get_smoothing_gain()
{
    return (2.0f + (float)RC_FEEL_RP_VERY_CRISP/10.0f);
}

void get_input()
{
  for( temp_idx=0 ; temp_idx<8 ; temp_idx++ )
  {
    RC_IN = map( TinyPpmReader.width_us(temp_idx+1), PWM_LOW, PWM_HIGH, RC_LOW, RC_HIGH );
    switch(temp_idx)
    {
      case ROLL_CH: myPPM[ROLL_CH]=pwm_to_angle_dz(RC_IN, 1);break;
      case PITCH_CH: myPPM[PITCH_CH]=pwm_to_angle_dz(RC_IN, 1);break;
      case COLLECTIVE_CH: myPPM[COLLECTIVE_CH]=pwm_to_range_dz(RC_IN, 1);break;
      case YAW_CH:  myPPM[YAW_CH]=pwm_to_angle_dz(RC_IN, 1);
                    yaw_raw = RC_IN;
                    break;
    }
  }
}


void get_pilot_input(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
  roll_out = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
  pitch_out = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);  
}

static float get_pilot_yaw_rate(int16_t stick_angle)
{
  return stick_angle * ACRO_YAW_P;
}

void helicopter_stablize()
{
  int16_t target_roll, target_pitch;
  float target_yaw_rate;
  get_pilot_input( myPPM[ROLL_CH], myPPM[PITCH_CH], target_roll, target_pitch );
  target_yaw_rate = get_pilot_yaw_rate(myPPM[YAW_CH]);
  attitude.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
  
  // update the servos for CCPM Mxing
  SERVO_OUT[COLLECTIVE_CH] = myPPM[COLLECTIVE_CH];
  roll_pwm                 = angle_to_pwm( SERVO_OUT[ROLL_CH], -1 ); //SERVO_OUT[ROLL_CH];
  pitch_pwm                = angle_to_pwm( SERVO_OUT[PITCH_CH], 1 ); //SERVO_OUT[PITCH_CH];
  collective_pwm           = range_to_pwm( SERVO_OUT[COLLECTIVE_CH] ); //SERVO_OUT[COLLECTIVE_CH];
  yaw_pwm                  = angle_to_pwm( SERVO_OUT[YAW_CH], 1 ); //SERVO_OUT[YAW_CH];
}

int16_t pwm_to_angle_dz(int16_t radio_in, int8_t _reverse )
{
    if ((RC_MID - RC_LOW) == 0 || (RC_HIGH - RC_MID) == 0)
        return 0;

    if(radio_in > RC_MID) 
    {
        return _reverse * ((long)ROLL_PITCH_INPUT_MAX * (long)(radio_in - RC_MID)) / (long)(RC_HIGH  - RC_MID);
    }else if(radio_in < RC_MID) 
    {
        return _reverse * ((long)ROLL_PITCH_INPUT_MAX * (long)(radio_in - RC_MID)) / (long)(RC_MID - RC_LOW);
    }else
        return 0;
}

int16_t angle_to_pwm(int16_t servo_out, int8_t _reverse )
{
    if((servo_out * _reverse) > 0)
        return _reverse * ((long)servo_out * (long)(RC_HIGH - RC_MID)) / (long)ROLL_PITCH_INPUT_MAX;
    else
        return _reverse * ((long)servo_out * (long)(RC_MID - RC_LOW)) / (long)ROLL_PITCH_INPUT_MAX;
}

int16_t pwm_to_range_dz(uint16_t radio_in, uint8_t _reverse )
{
    int16_t r_in = constrain_int16(radio_in, RC_LOW, RC_HIGH );

    if (_reverse == -1) {
      r_in = (long)RC_HIGH - (r_in - (long)RC_LOW);
    }

    if (r_in > RC_LOW)
        return (COLLECTIVE_LOW + ((long)(COLLECTIVE_HIGH - COLLECTIVE_LOW) * (long)(r_in - RC_LOW)) / (long)(RC_HIGH - RC_LOW));
    else
        return COLLECTIVE_LOW;
}

int16_t range_to_pwm(int16_t servo_out)
{
    if (COLLECTIVE_HIGH == COLLECTIVE_LOW) {
        return RC_MID;
    }
    return ((long)(servo_out - COLLECTIVE_LOW) * (long)(RC_HIGH - RC_LOW)) / (long)(COLLECTIVE_HIGH - COLLECTIVE_LOW);
}

void cyclic_collective_pitch_mixing()
{
  // CCPM : Cyclic Collective Pitch Mixing
  // Technique used for generating the swash values of 120 deg displaced servos
  // sv_180   = -cos(0)*roll   + sin(0)*pitch   + throttle      => throttle - roll
  // sv_n60   = -cos(120)*roll + sin(120)*pitch + throttle      => throttle + 0.5*roll + 0.866*pitch
  // sv_60    = -cos(240)*roll + sin(240)*pitch + throttle      => throttle + 0.5*roll - 0.866*pitch
  //                    120 DEG SWASH REPRESENTATON
  //                               (sv_180)
  //                                  |
  //                                  |
  //                            120  / \  120
  //                                /   \
  //                               / 120 \
  //                          (sv_n60)  (sv_60)
  
  CCPM[0]          = ( ( 0.866 * roll_pwm ) + ( 0.5 * pitch_pwm ) ) + collective_pwm;
  CCPM[1]          = ( ( -0.866 * roll_pwm ) + ( 0.5 * pitch_pwm ) ) + collective_pwm;
  CCPM[2]          = ( ( -1.0 * pitch_pwm ) ) + collective_pwm;
  CCPM[3]          = ( yaw_pwm + yaw_raw );
   
}

void update_simple_mode(void)
{
  rollx             =  myPPM[ROLL_CH]*simple_cos_yaw - myPPM[PITCH_CH]*simple_sin_yaw;
  pitchx            =  myPPM[ROLL_CH]*simple_sin_yaw + myPPM[PITCH_CH]*simple_cos_yaw;
  myPPM[ROLL_CH]    =  rollx*_ahrs.cos_yaw() + pitchx*_ahrs.sin_yaw();
  myPPM[PITCH_CH]   = -rollx*_ahrs.sin_yaw() + pitchx*_ahrs.cos_yaw();
}

void helicopter_main()
{
  if( TinyPpmReader.isSynchro() )
  {
    get_input();
    ins.wait_for_sample();
    _ahrs.update();
    update_simple_mode();
    attitude.rate_controller_run();
    helicopter_stablize();
    cyclic_collective_pitch_mixing();
  }
}

void motor_main()
{
  signal_1       = angle_to_pwm(pwm_to_angle_dz(CCPM[0], -1), 1)*1.3;   
  signal_2       = angle_to_pwm(pwm_to_angle_dz(CCPM[1], -1), 1)*1.3;
  signal_3       = angle_to_pwm(pwm_to_angle_dz(CCPM[2], -1), 1)*1.3;
  signal_4       = CCPM[3];
  
  signal_2       = map(signal_2, RC_LOW, RC_HIGH, RC_HIGH, RC_LOW);// Signal inverted due to mirror
  
  if( signal_1 > RC_HIGH ) signal_1 = RC_HIGH;
  if( signal_2 > RC_HIGH ) signal_2 = RC_HIGH;
  if( signal_3 > RC_HIGH ) signal_3 = RC_HIGH;
  if( signal_4 > RC_HIGH ) signal_4 = RC_HIGH;
     
  if( signal_1 < RC_LOW ) signal_1 = RC_LOW;
  if( signal_2 < RC_LOW ) signal_2 = RC_LOW;
  if( signal_3 < RC_LOW ) signal_3 = RC_LOW;
  if( signal_4 < RC_LOW ) signal_4 = RC_LOW;
  
  cyclic_1.writeMicroseconds(signal_1);
  cyclic_2.writeMicroseconds(signal_2);
  cyclic_3.writeMicroseconds(signal_3);
  yaw_servo.writeMicroseconds(signal_4);
}

void setup()
{
  
  Serial.begin(115200);
  Serial.println(F("[o] AHRS[info]: Starting AHRS v1.0"));
  ins.init(InertialSensor::COLD_START, InertialSensor::RATE_100HZ);
  ins.set_default_filter(30);
//  ins.calibrate_accel(roll_trim, pitch_trim);
  ins.init_accel();
  compass.init();
  compass.set_declination((-12.51/1000),false); 
  _ahrs.set_compass( &compass );
  TinyPpmReader.attach(PPM_INPUT_PIN);
  attitude.update_feedforward_filter_rates( MAIN_LOOP_SECONDS );
  attitude.relax_bf_rate_controller();
  attitude.set_dt( MAIN_LOOP_SECONDS );
  
  simple_cos_yaw = _ahrs.cos_yaw();
  simple_sin_yaw = _ahrs.sin_yaw();
    
  cyclic_1.attach(8);
  cyclic_2.attach(7);
  cyclic_3.attach(3);
  yaw_servo.attach(2);
  Serial.println(F("[o] AHRS[info]: Finished initialization starting the AHRS"));
  
}

void loop()
{
    now = micros();
    
    /* AHRS lopp */
    /* 10ms ~ 100Hz */
    if( now - last_100hz >= 10000UL )
    {
      helicopter_main();
      last_100hz = now;
    }
    
    /* Servo loop */
    /* 20ms ~ 50hz */
    if( now - last_50hz >= 20000UL )
    {
      motor_main();
      last_50hz = now;
      if( true )
       {
         Serial.print( signal_1 );
         Serial.print( F(" : ") );
         Serial.print( signal_2 );
         Serial.print( F(" : ") );
         Serial.print( signal_3 );
         Serial.print( F(" : ") );
         Serial.print( signal_4 );
         Serial.println();
       }
    }
}
  
