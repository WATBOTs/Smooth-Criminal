//This module handles the RC interface (interrupt driven)

#include "rc_handler.h"
#include "arduino.h"
#include "melty_config.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Lock state constants
// Used to guard RC channel data from being modified mid-read by an ISR.
// ---------------------------------------------------------------------------
#define RC_DATA_UNLOCKED 0
#define RC_DATA_LOCKED 1

static int rc_data_lock_state = RC_DATA_UNLOCKED;

// ---------------------------------------------------------------------------
// RC channel struct
// Each channel tracks its Arduino pin, the most recent measured pulse length
// (in microseconds), the timestamp of the last rising edge (used to calculate
// pulse length), and the timestamp of the last valid pulse (used for health
// checks).
// ---------------------------------------------------------------------------
struct rc_channel_t {
  int pin;                        // Arduino pin this channel is wired to
  unsigned long pulse_length;     // Most recent valid pulse length (us)
  unsigned long pulse_start_time; // micros() timestamp of last rising edge
  unsigned long last_good_signal; // millis() timestamp of last valid pulse
};

// ---------------------------------------------------------------------------
// Channel instances
// One struct per physical RC channel. Initialised with safe defaults so the
// robot stays still if no signal has arrived yet.
// ---------------------------------------------------------------------------
static struct rc_channel_t forback_rc_channel = {
  .pin = FORBACK_RC_CHANNEL_PIN,
  .pulse_length = MIN_RC_PULSE_LENGTH,
  .pulse_start_time = 0,
  .last_good_signal = 0
};

static struct rc_channel_t leftright_rc_channel = {
  .pin = LEFTRIGHT_RC_CHANNEL_PIN,
  .pulse_length = MIN_RC_PULSE_LENGTH,
  .pulse_start_time = 0,
  .last_good_signal = 0
};

static struct rc_channel_t throttle_rc_channel = {
  .pin = THROTTLE_RC_CHANNEL_PIN,
  .pulse_length = MIN_RC_PULSE_LENGTH,
  .pulse_start_time = 0,
  .last_good_signal = 0
};

// ---------------------------------------------------------------------------
// Data locking helpers
// Call lock_rc_data() before reading any channel struct, and unlock_rc_data()
// immediately after. This prevents an ISR from writing a partial update into
// a channel while the main loop is in the middle of reading it.
// ---------------------------------------------------------------------------
static void lock_rc_data() {
  rc_data_lock_state = RC_DATA_LOCKED;
}

static void unlock_rc_data() {
  rc_data_lock_state = RC_DATA_UNLOCKED;
}

// ---------------------------------------------------------------------------
// update_rc_channel()
// Called from each ISR on every CHANGE of the associated pin.
// - Rising edge (pin == 1): records the start timestamp.
// - Falling edge (pin == 0): computes the pulse length and, if it falls
//   within the valid RC range, stores it and refreshes last_good_signal.
// Skips the update entirely if the main loop currently holds the lock.
// ---------------------------------------------------------------------------
static void update_rc_channel(struct rc_channel_t *rc_channel) {
  if (rc_data_lock_state == RC_DATA_LOCKED) return;

  int rc_channel_current_state = digitalRead(rc_channel->pin);

  // Rising edge — record when the pulse started
  if (rc_channel_current_state == 1) {
    rc_channel->pulse_start_time = micros();
  }

  // Falling edge — measure how long the pulse lasted
  if (rc_channel_current_state == 0) {
    // Guard against micros() overflow wrapping to a value smaller than
    // pulse_start_time; if that happened just skip this sample.
    if (micros() > rc_channel->pulse_start_time) {
      unsigned long new_pulse_length = micros() - rc_channel->pulse_start_time;
      // Only accept pulses within the valid RC PWM window to reject glitches
      if (new_pulse_length <= MAX_RC_PULSE_LENGTH && new_pulse_length >= MIN_RC_PULSE_LENGTH) {
        rc_channel->pulse_length = new_pulse_length;
        rc_channel->last_good_signal = millis();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// rc_signal_is_healthy()
// Returns true if a valid throttle pulse has been received within the allowed
// gap defined by MAX_MS_BETWEEN_RC_UPDATES. Used by the main loop to decide
// whether it is safe to drive the robot.
// ---------------------------------------------------------------------------
bool rc_signal_is_healthy() {
  lock_rc_data();
  unsigned long last_good_signal = throttle_rc_channel.last_good_signal;
  unlock_rc_data();

  // No pulse has ever arrived
  if (last_good_signal == 0) return false;

  // Signal has gone stale
  if (millis() - last_good_signal > MAX_MS_BETWEEN_RC_UPDATES) return false;

  return true;
}

// ---------------------------------------------------------------------------
// rc_get_throttle_percent()
// Maps the raw throttle pulse length to an integer percentage [0, 100].
// Values at or below IDLE_THROTTLE_PULSE_LENGTH return 0; values at or above
// FULL_THROTTLE_PULSE_LENGTH return 100. This creates deliberate dead-zones
// at both ends of the stick travel so the robot doesn't creep unintentionally.
// ---------------------------------------------------------------------------
int rc_get_throttle_percent() {
  lock_rc_data();
  unsigned long pulse_length = throttle_rc_channel.pulse_length;
  unlock_rc_data();

  if (pulse_length >= FULL_THROTTLE_PULSE_LENGTH) return 100;
  if (pulse_length <= IDLE_THROTTLE_PULSE_LENGTH) return 0;

  long throttle_percent = (pulse_length - IDLE_THROTTLE_PULSE_LENGTH) * 100;
  throttle_percent = throttle_percent / (FULL_THROTTLE_PULSE_LENGTH - IDLE_THROTTLE_PULSE_LENGTH);
  return (int)throttle_percent;
}

// ---------------------------------------------------------------------------
// rc_get_is_lr_in_config_deadzone()
// Returns true when the left/right stick is close enough to centre to be
// considered "untouched" during configuration mode. The config deadzone is
// typically wider than the normal deadzone so accidental nudges are ignored
// while the user is navigating menus.
// ---------------------------------------------------------------------------
bool rc_get_is_lr_in_config_deadzone() {
  if (abs(rc_get_leftright()) < LR_CONFIG_MODE_DEADZONE_WIDTH) return true;
  return false;
}

// ---------------------------------------------------------------------------
// rc_get_is_lr_in_normal_deadzone()
// Returns true when the left/right stick offset is small enough to be treated
// as centred during normal driving. Prevents the robot from drifting due to
// stick trim imperfection.
// ---------------------------------------------------------------------------
bool rc_get_is_lr_in_normal_deadzone() {
  if (abs(rc_get_leftright()) < LR_NORMAL_DEADZONE_WIDTH) return true;
  return false;
}

// ---------------------------------------------------------------------------
// rc_get_forback()
// Classifies the forward/back stick position as FORWARD, BACKWARD, or NEUTRAL.
// Uses a threshold offset from the centre pulse so small deviations don't
// trigger unwanted translation.
// ---------------------------------------------------------------------------
rc_forback rc_get_forback() {
  lock_rc_data();
  unsigned long pulse_length = forback_rc_channel.pulse_length;
  unlock_rc_data();

  int rc_forback_offset = pulse_length - CENTER_FORBACK_PULSE_LENGTH;
  if (rc_forback_offset > FORBACK_MIN_THRESH_PULSE_LENGTH) return RC_FORBACK_FORWARD;
  if (rc_forback_offset < (FORBACK_MIN_THRESH_PULSE_LENGTH * -1)) return RC_FORBACK_BACKWARD;
  return RC_FORBACK_NEUTRAL;
}

// ---------------------------------------------------------------------------
// rc_get_leftright()
// Returns the raw signed offset of the left/right stick from its centre pulse
// length, in microseconds. Negative = left, positive = right.
// A perfect centre stick would return 0; real hardware usually returns ±50 us.
// ---------------------------------------------------------------------------
int rc_get_leftright() {
  lock_rc_data();
  unsigned long pulse_length = leftright_rc_channel.pulse_length;
  unlock_rc_data();

  return pulse_length - CENTER_LEFTRIGHT_PULSE_LENGTH;
}

// ---------------------------------------------------------------------------
// World-space command API
// The two functions below convert the raw stick axes into meaningful
// world-space commands: a target heading angle and a scalar translation speed.
// These are the values the motion planner should consume rather than the raw
// channel values above.
// ---------------------------------------------------------------------------

// getTargetHeading()
// Converts the combined left/right and forward/back stick deflection into a
// compass-style heading in degrees [0, 360):
//   0°  = forward, 90° = right, 180° = backward, 270° = left.
// Returns -1.0 when the stick is inside the deadzone, signalling that the
// caller should hold the current heading rather than command a new one.
float getTargetHeading() {
  lock_rc_data();
  unsigned long fb_pulse = forback_rc_channel.pulse_length;
  unsigned long lr_pulse  = leftright_rc_channel.pulse_length;
  unlock_rc_data();

  // Convert raw pulses to signed, zero-centred offsets.
  // lr > 0 means right, fb > 0 means forward.
  float lr = (float)((int)lr_pulse - CENTER_LEFTRIGHT_PULSE_LENGTH);
  float fb = (float)((int)fb_pulse - CENTER_FORBACK_PULSE_LENGTH);

  // If the stick magnitude is within the deadzone, report no heading intent
  float magnitude = sqrtf(lr * lr + fb * fb);
  if (magnitude < (float)LR_NORMAL_DEADZONE_WIDTH) return -1.0f;

  // atan2(x, y) gives angle from the +Y (forward) axis, rotating CW toward +X
  // (right), which matches standard compass / robot heading convention.
  float heading_rad = atan2f(lr, fb);
  float heading_deg = heading_rad * (180.0f / (float)M_PI);

  // Normalise to [0, 360)
  if (heading_deg < 0.0f) heading_deg += 360.0f;

  return heading_deg;
}

// getTranslationSpeed()
// Returns how hard the stick is being pushed, independent of direction, as a
// normalised float [0.0, 1.0]. This is used to scale motor output — 0.0 means
// stopped, 1.0 means full speed.
//
// The deadzone is subtracted before normalisation so speed ramps smoothly from
// zero at the deadzone boundary up to 1.0 at maximum stick deflection, with
// no discontinuity.
float getTranslationSpeed() {
  lock_rc_data();
  unsigned long fb_pulse = forback_rc_channel.pulse_length;
  unsigned long lr_pulse  = leftright_rc_channel.pulse_length;
  unlock_rc_data();

  // Compute stick vector magnitude in pulse-offset space
  float lr = (float)((int)lr_pulse - CENTER_LEFTRIGHT_PULSE_LENGTH);
  float fb = (float)((int)fb_pulse - CENTER_FORBACK_PULSE_LENGTH);
  float magnitude = sqrtf(lr * lr + fb * fb);

  // Maximum deflection a stick can physically reach from centre
  float max_deflection = (float)(MAX_RC_PULSE_LENGTH - MIN_RC_PULSE_LENGTH) / 2.0f;

  // Return 0 for any input inside the deadzone
  float deadzone = (float)LR_NORMAL_DEADZONE_WIDTH;
  if (magnitude <= deadzone) return 0.0f;

  // Remap the remaining range to [0.0, 1.0] and clamp for safety
  float speed = (magnitude - deadzone) / (max_deflection - deadzone);
  if (speed > 1.0f) speed = 1.0f;

  return speed;
}

// ---------------------------------------------------------------------------
// Interrupt Service Routines (ISRs)
// One ISR per channel, each firing on every logic-level CHANGE of its pin.
// They simply delegate to update_rc_channel() which does the real work.
// Kept minimal — ISRs should execute as fast as possible.
// ---------------------------------------------------------------------------
void forback_rc_change() {
  update_rc_channel(&forback_rc_channel);
}
void leftright_rc_change() {
  update_rc_channel(&leftright_rc_channel);
}
void throttle_rc_change() {
  update_rc_channel(&throttle_rc_channel);
}

// ---------------------------------------------------------------------------
// init_rc()
// Registers the three ISRs with the Arduino interrupt system. Must be called
// once during setup() before RC input is needed. CHANGE mode fires on both
// rising and falling edges, which is required to measure pulse width.
// ---------------------------------------------------------------------------
void init_rc(void) {
  attachInterrupt(digitalPinToInterrupt(forback_rc_channel.pin), forback_rc_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftright_rc_channel.pin), leftright_rc_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(throttle_rc_channel.pin), throttle_rc_change, CHANGE);
}