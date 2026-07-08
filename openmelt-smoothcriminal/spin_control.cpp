//this module handles calculation and timing loop for translational drift
//direction of rotation assumed to be CLOCKWISE (but should work counter-clockwise)

#include "arduino.h"
#include "melty_config.h"
#include "motor_driver.h"
#include "rc_handler.h"
#include "spin_control.h"
#include "config_storage.h"
#include "led_driver.h"
#include "battery_monitor.h"
#include "heading_estimator.h"
#include "IR_handler.h"

#define LEFT_RIGHT_CONFIG_LED_ADJUST_DIVISOR 0.1f         //How quick LED heading is adjusted in config mode (larger values = slower)

#define MAX_TRANSLATION_ROTATION_INTERVAL_US (1.0f / MIN_TRANSLATION_RPM) * 60 * 1000 * 1000
#define MAX_TRACKING_ROTATION_INTERVAL_US MAX_TRANSLATION_ROTATION_INTERVAL_US * 2   //don't track heading if we are this slow (also puts upper limit on time spent in melty loop for safety)

static float led_offset_percent = DEFAULT_LED_OFFSET_PERCENT;         //stored in EEPROM as an INT - but handled as a float for configuration purposes

static unsigned int highest_rpm = 0;
static bool config_mode = false;   //1 if we are in config mode

//loads settings from EEPROM
void load_melty_config_settings() {
#ifdef ENABLE_EEPROM_STORAGE 
  led_offset_percent = load_heading_led_offset();
#endif  
}

//saves settings to EEPROM
void save_melty_config_settings() {
#ifdef ENABLE_EEPROM_STORAGE 
  save_settings_to_eeprom(led_offset_percent);
#endif  
}

void toggle_config_mode() {
  config_mode = !config_mode;
  
  //enterring or exiting config mode also resets highest observed RPM
  highest_rpm = 0;
}

bool get_config_mode() {
  return config_mode;
}

int get_max_rpm() {
  return highest_rpm;
}

//calculates time for this rotation of robot based on estimated RPM
static float get_rotation_interval_ms() {

  //Get rpm from heading estimator
  float rpm = getRPM();

  //Prevent divide by zero if rpm is 0
  if(rpm == 0) return MAX_TRACKING_ROTATION_INTERVAL_US / 1000.0f;

  //Record highest rpm
  if (rpm > highest_rpm || highest_rpm == 0) highest_rpm = rpm;

  //Calculate rotation interval based on rpm and adjustment factor from rc inputs
  float rotation_interval = (1.0f / rpm) * 60 * 1000;
  return rotation_interval;
}

//performs changes to melty parameters when in config mode
static struct melty_parameters_t handle_config_mode(struct melty_parameters_t melty_parameters) {

  //if forback forward - normal drive (for driver testing - no adjustment of melty parameters)

  //if forback neutral - IR Beacon calibration 
  if (melty_parameters.translate_forback == RC_FORBACK_NEUTRAL) {

    //calibration overrides steering
    melty_parameters.steering_disabled = 1;

    /*
    
    //only adjust if stick is outside deadzone    
    if (rc_get_is_lr_in_config_deadzone() == false) {
      //show that we are changing config
      melty_parameters.led_shimmer = 1;

      float adjustment_factor = (accel_mount_radius_cm * (float)(rc_get_leftright() / (float)NOMINAL_PULSE_RANGE));
      adjustment_factor = adjustment_factor / LEFT_RIGHT_CONFIG_RADIUS_ADJUST_DIVISOR;
      accel_mount_radius_cm = accel_mount_radius_cm + adjustment_factor;

      if (accel_mount_radius_cm < ACCEL_MOUNT_RADIUS_MINIMUM_CM) accel_mount_radius_cm = ACCEL_MOUNT_RADIUS_MINIMUM_CM;
    }
    */    
  }
  
  //if forback backward - do LED heading adjustment (don't translate)
  if (melty_parameters.translate_forback == RC_FORBACK_BACKWARD) {
    
    //LED heading offset adjustment overrides steering
    melty_parameters.steering_disabled = 1;
    
    //only adjust if stick is outside deadzone  
    if (rc_get_is_lr_in_config_deadzone() == false) {

      //disable translation if adjusting heading
      melty_parameters.translate_forback = RC_FORBACK_NEUTRAL;

      //show that we are changing config
      melty_parameters.led_shimmer = 1;

      //Adjustment factor to change the LED offset
      float adjustment_factor =  (float)(rc_get_leftright() / (float)NOMINAL_PULSE_RANGE);
      adjustment_factor = adjustment_factor / LEFT_RIGHT_CONFIG_LED_ADJUST_DIVISOR;
      led_offset_percent = led_offset_percent + adjustment_factor;

      if (led_offset_percent > 99) led_offset_percent = 0;
      if (led_offset_percent < 0) led_offset_percent = 99;

    }
  }    
  return melty_parameters;  
}

//Calculates all parameters need for a single rotation (motor timing, LED timing, etc.)
//This entire section takes ~1300us on an Atmega32u4 (acceptable - fast enough to not have major impact on tracking accuracy)
static struct melty_parameters_t get_melty_parameters(void) {

  struct melty_parameters_t melty_parameters = {};

  float led_offset_portion = led_offset_percent / 100.0f;

  melty_parameters.throttle_percent = rc_get_throttle_percent() / 100.0f;

  //by default motor_on_portion maps to thottle_percent input - but that can be altered
  float motor_on_portion = melty_parameters.throttle_percent;

  //changes motor_on_portion to fixed value if we are throttling via PWM if DYNAMIC_PWM_MOTOR_ON_PORTION is defined
  #ifdef DYNAMIC_PWM_MOTOR_ON_PORTION
    if (THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
      motor_on_portion = DYNAMIC_PWM_MOTOR_ON_PORTION;
    }
  #endif 

  float led_on_portion = melty_parameters.throttle_percent;  //LED width changed with throttle percent
  if (led_on_portion < 0.10f) led_on_portion = 0.10f;
  if (led_on_portion > 0.90f) led_on_portion = 0.90f;

  melty_parameters.translate_forback = rc_get_forback();

  //if we are in config mode - handle it (and disable steering if needed)
  if (get_config_mode() == true) {
    melty_parameters = handle_config_mode(melty_parameters);
  }

  melty_parameters.rotation_interval_us = get_rotation_interval_ms() * 1000;
  
  //if under defined RPM - just try to spin up (motors on for full rotation)
  if (melty_parameters.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US) motor_on_portion = 1;

  //if we are too slow - don't even try to track heading
  if (melty_parameters.rotation_interval_us > MAX_TRACKING_ROTATION_INTERVAL_US) {
    melty_parameters.rotation_interval_us = MAX_TRACKING_ROTATION_INTERVAL_US;
  }

  unsigned long motor_on_us = motor_on_portion * melty_parameters.rotation_interval_us;
  unsigned long led_on_us = led_on_portion * melty_parameters.rotation_interval_us;
  unsigned long led_offset_us = led_offset_portion * melty_parameters.rotation_interval_us;

  //starts LED on time at point in rotation so it's "centered" on led offset
  if (led_on_us / 2 <= led_offset_us) {
    melty_parameters.led_start = led_offset_us - (led_on_us / 2);
  } else {
    melty_parameters.led_start = (melty_parameters.rotation_interval_us + led_offset_us) - (led_on_us / 2);
  }
  
  melty_parameters.led_stop = melty_parameters.led_start + led_on_us;

  //"wraps" led off time if it exceeds rotation length
  if (melty_parameters.led_stop > melty_parameters.rotation_interval_us){
    melty_parameters.led_stop = melty_parameters.led_stop - melty_parameters.rotation_interval_us;
  }

  uint16_t target_heading = getTargetHeading();
  uint16_t current_heading = getHeadingDeg();
  
  //Calculate error between target and current headings
  int16_t heading_error = (target_heading - current_heading + 360) % 360;
  //Convert heading error into a time offset for the motor window for phase 1
  unsigned long target_offset_us = (heading_error / 360.0f) * melty_parameters.rotation_interval_us;
  //Calculate the motor window for phase 2 based on a 180 degree offset from phase 1
  unsigned long opposite_offset_us = (target_offset_us + (melty_parameters.rotation_interval_us / 2)) % melty_parameters.rotation_interval_us;

  //phase 1 timing: for motor_1 in forward translation or motor_2 in reverse
  melty_parameters.motor_start_phase_1 = target_offset_us - (motor_on_us / 2);
  melty_parameters.motor_stop_phase_1  = target_offset_us + (motor_on_us / 2);

  //phase 2 timing: for motor_2 in forward translation or motor_1 in reverse
  melty_parameters.motor_start_phase_2 = opposite_offset_us - (motor_on_us / 2);
  melty_parameters.motor_stop_phase_2  = opposite_offset_us + (motor_on_us / 2);

  //if the battery voltage is low - shimmer the LED to let user know
#ifdef BATTERY_ALERT_ENABLED
  if (battery_voltage_low() == true) melty_parameters.led_shimmer = 1;
#endif

  return melty_parameters;
}

//Helper function to check if time is within calculated motor window
//This function is needed as phase 1 or 2 can wrap at the ends of the rotation interval depending on the target heading
static bool in_motor_window(unsigned long currentTime, unsigned long start, unsigned long stop) {
  if (start > stop) {
    return currentTime >= start || currentTime <= stop;
  } else {
    return currentTime >= start && currentTime <= stop;
  }
}

//handle translating forward
static void translate_forward(struct melty_parameters_t melty_parameters, unsigned long time_spent_this_rotation_us) {
  if (in_motor_window(time_spent_this_rotation_us, melty_parameters.motor_start_phase_1, melty_parameters.motor_stop_phase_1)) {
    motor_1_on(melty_parameters.throttle_percent);
  } else {
    motor_1_coast();
  }
  if (in_motor_window(time_spent_this_rotation_us, melty_parameters.motor_start_phase_2, melty_parameters.motor_stop_phase_2)) {        
    motor_2_on(melty_parameters.throttle_percent);
  } else {
    motor_2_coast();
  }
}

//handle translating backward (motor1 and motor2 timings are swapped - offset by 180 degrees)
static void translate_backward(struct melty_parameters_t melty_parameters, unsigned long time_spent_this_rotation_us) {
  if (in_motor_window(time_spent_this_rotation_us, melty_parameters.motor_start_phase_2, melty_parameters.motor_stop_phase_2)) {
    motor_1_on(melty_parameters.throttle_percent);
  } else {
    motor_1_coast();
  }

  if (in_motor_window(time_spent_this_rotation_us, melty_parameters.motor_start_phase_1, melty_parameters.motor_stop_phase_1)) {
    motor_2_on(melty_parameters.throttle_percent);
  } else {
    motor_2_coast();
  }
}

//turns on heading LED at appropriate timing
static void update_heading_led(struct melty_parameters_t melty_parameters, unsigned long time_spent_this_rotation_us) {
  if (melty_parameters.led_start > melty_parameters.led_stop) {
    if (time_spent_this_rotation_us >= melty_parameters.led_start || time_spent_this_rotation_us <= melty_parameters.led_stop) {
      heading_led_on(melty_parameters.led_shimmer);
    } else {
      heading_led_off();
    }
  } else {
    if (time_spent_this_rotation_us >= melty_parameters.led_start && time_spent_this_rotation_us <= melty_parameters.led_stop) {
      heading_led_on(melty_parameters.led_shimmer);
    } else {
      heading_led_off();
    }
  }
}

//rotates the robot once + handles translational drift
//(repeat as needed)
void spin_one_rotation(void) {

  //-initial- assignment of melty parameters
  static struct melty_parameters_t melty_parameters = get_melty_parameters();

  //capture initial time stamp before rotation start
  unsigned long start_time = micros();
  unsigned long time_spent_this_rotation_us = 0;

  //tracking cycle count is needed to alternate cycles for non-translation (overflow is non-issue)
  static unsigned long cycle_count = 0;
  cycle_count++;

  //the melty parameters are updated either at the beginning of the rotation - or the middle of the rotation (alternating each time)
  //this is done so that any errors due to the heading read/update/math cycle cancel out any effect on tracking / translational drift
  int melty_parameter_update_time_offset_us = 0;
  if (cycle_count % 2 == 1) melty_parameter_update_time_offset_us = melty_parameters.rotation_interval_us / 2;  
  bool melty_parameters_updated_this_rotation = false;

  #ifdef SERIAL_DEBUG_OUTPUT //Increment cycle count for serial debug output if defined
    static unsigned int debug_cycle_count = 0;
    debug_cycle_count++;
  #endif

  //loop for one rotation of robot
  while (time_spent_this_rotation_us < melty_parameters.rotation_interval_us) {

    //Check for new rising edge on IR sensor
    if(hasNewEdge()){
      updateHeading(getEdgeTime()); //Update heading estimator
    }

    //update melty parameters if we haven't / update time has elapsed
    if (melty_parameters_updated_this_rotation == false && time_spent_this_rotation_us > melty_parameter_update_time_offset_us) { 
      melty_parameters = get_melty_parameters();
      melty_parameters_updated_this_rotation = true;
    }

    //Log data to USB Serial if defined in melty_config.h
    #ifdef SERIAL_DEBUG_OUTPUT
      if(debug_cycle_count % SERIAL_DEBUG_INTERVAL == 0){
        Serial.print("RPM: "); Serial.print(getRPM());
        Serial.print("  Est. Heading: "); Serial.print(getHeadingDeg());
        Serial.print("  Target. Heading: "); Serial.print(getTargetHeading());
        Serial.print("  RC Throttle: "); Serial.print(rc_get_throttle_percent());
        Serial.print("  RC L/R: "); Serial.print(rc_get_leftright());
        Serial.print("  RC F/B: "); Serial.print(rc_get_forback());
        Serial.print("  Motor 1 Phase Start: "); Serial.print(melty_parameters.motor_start_phase_1);
        Serial.print("  Motor 1 Phase Stop: "); Serial.print(melty_parameters.motor_stop_phase_1);
        Serial.print("  Motor 2 Phase Start: "); Serial.print(melty_parameters.motor_start_phase_2);
        Serial.print("  Motor 2 Phase Stop: "); Serial.print(melty_parameters.motor_stop_phase_2);
        Serial.print("  Curr. Timestamp: "); Serial.print(getEstimatorEdgeTime(0));
        Serial.print("  Prev. Timestamp "); Serial.print(getEstimatorEdgeTime(1));
        Serial.println("");
      }
    #endif

    //if translation direction is RC_FORBACK_NEUTRAL - robot cycles between forward and reverse translation for net zero translation
    //if motor 2 (or motor 1) is not present - control sequence remains identical (signal still generated for non-connected motor)

    //translate forward
    if (melty_parameters.translate_forback == RC_FORBACK_FORWARD || (melty_parameters.translate_forback == RC_FORBACK_NEUTRAL && cycle_count % 2 == 0)) {
      translate_forward(melty_parameters, time_spent_this_rotation_us);
    }

    //translate backward
    if (melty_parameters.translate_forback == RC_FORBACK_BACKWARD || (melty_parameters.translate_forback == RC_FORBACK_NEUTRAL && cycle_count % 2 == 1)) {
      translate_backward(melty_parameters, time_spent_this_rotation_us);
    }

    //displays heading LED at correct location
    update_heading_led(melty_parameters, time_spent_this_rotation_us);

    time_spent_this_rotation_us = micros() - start_time;

  }

}
