#include "input.h"

#include <math.h>
#include <string.h>

#include "control.h"
#include "profile.h"
#include "util.h"

extern profile_t profile;

// cache the last result so it does not get calculated everytime
static float last_rx[2] = {13.13f, 12.12f};
static float stickvector[3] = {0, 0, 1};

void input_stick_vector(float rx_input[], float maxangle) {
  // only compute stick rotation if values changed
  if (last_rx[0] != rx_input[0] || last_rx[1] != rx_input[1]) {
    last_rx[0] = rx_input[0];
    last_rx[1] = rx_input[1];

    float pitch, roll;

    // rotate down vector to match stick position
    pitch = rx_input[1] * profile.rate.level_max_angle * DEGTORAD + (float)TRIM_PITCH * DEGTORAD;
    roll = rx_input[0] * profile.rate.level_max_angle * DEGTORAD + (float)TRIM_ROLL * DEGTORAD;

    stickvector[0] = fastsin(roll);
    stickvector[1] = fastsin(pitch);
    stickvector[2] = fastcos(roll) * fastcos(pitch);

    float mag2 = (stickvector[0] * stickvector[0] + stickvector[1] * stickvector[1]);

    if (mag2 > 0.001f) {
      mag2 = Q_rsqrt(mag2 / (1 - stickvector[2] * stickvector[2]));
    } else
      mag2 = 0.707f;

    stickvector[0] *= mag2;
    stickvector[1] *= mag2;

#ifdef INVERTED_ENABLE
    extern int pwmdir;

    if (pwmdir == REVERSE) {
      stickvector[0] = -stickvector[0];
      stickvector[1] = -stickvector[1];
      stickvector[2] = -stickvector[2];
    }
#endif
  }

  // find error between stick vector and quad orientation
  // vector cross product
  state.errorvect.axis[1] = -((state.GEstG.axis[1] * stickvector[2]) - (state.GEstG.axis[2] * stickvector[1]));
  state.errorvect.axis[0] = (state.GEstG.axis[2] * stickvector[0]) - (state.GEstG.axis[0] * stickvector[2]);

  // some limits just in case

  limitf(&state.errorvect.axis[0], 1.0);
  limitf(&state.errorvect.axis[1], 1.0);

  // fix to recover if triggered inverted
  // the vector cross product results in zero for opposite vectors, so it's bad at 180 error
  // without this the quad will not invert if angle difference = 180
}

#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

static float calc_bf_rates(int axis) {
  float rcRate, superExpo;
  switch (axis) {
  case ROLL:
    rcRate = profile.rate.betaflight.rc_rate.roll;
    superExpo = profile.rate.betaflight.super_rate.roll;
    break;
  case PITCH:
    rcRate = profile.rate.betaflight.rc_rate.pitch;
    superExpo = profile.rate.betaflight.super_rate.pitch;
    break;
  case YAW:
    rcRate = profile.rate.betaflight.rc_rate.yaw;
    superExpo = profile.rate.betaflight.super_rate.yaw;
    break;
  }
  if (rcRate > 2.0f) {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
  }
  const float rcCommandfAbs = state.rx_filtered.axis[axis] > 0 ? state.rx_filtered.axis[axis] : -state.rx_filtered.axis[axis];
  float angleRate = 200.0f * rcRate * state.rx_filtered.axis[axis];
  if (superExpo) {
    const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superExpo), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
  }
  return constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT) * (float)DEGTORAD;
}

static float calc_actual_rates(int axis) {
  float centerSensitivity, maxRate;
  switch (axis) {
  case ROLL:
    centerSensitivity = profile.rate.acutal.center_sensitivity.roll;
    maxRate = profile.rate.acutal.max_rate.roll;
    break;
  case PITCH:
    centerSensitivity = profile.rate.actual.center_sensitivity.pitch;
    maxRate = profile.rate.acutal.max_rate.pitch;
    break;
  case YAW:
    centerSensitivity = profile.rate.actual.center_sensitivity.yaw;
    maxRate = profile.rate.actual.max_rate.yaw;
    break;
  }
    float stickMovement = maxRate - centerSensitivity;
    if (stickMovement < 0) {
      stickMovement = 0;
    }
    return state.rx_filtered_no_expo.axis[axis] * centerSensitivity + stickMovement * state.rx_filtered.axis[axis];
}

static float calc_kiss_rates(int axis) {
  float kissRates, rcRates;
  switch (axis) {
  case ROLL:
    rcRates = profile.rate.kiss.rc_rate.roll;
    kissRates = profile.rate.kiss.rate.roll;
    break;
  case PITCH:
    rcRates = profile.rate.kiss.rc_rate.pitch;
    kissRates = profile.rate.kiss.rate.pitch;
    break;
  case YAW:
    rcRates = profile.rate.kiss.rc_rate.yaw;
    kissRates = profile.rate.kiss.rate.yaw;
    break;
  }

  kissRates = 1 / (constrainf(1 - (state.rx_filtered_no_expo.axis[axis] * kissRates), 0.01, 1.0));
  rcRates = state.rx_filtered.axis[axis] * rcRates;
  return constrainf(2000.0 * kissRates * rcRates, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT) * (float)DEGTORAD;
}

static float calc_raceflight_rates(int axis) {
  float rate, acroPlus;
  switch (axis) {
  case ROLL:
    rate = profile.rate.raceflight.rate.roll;
    acroPlus = profile.rate.raceflight.acro_plus.roll;
    break;
  case PITCH:
    rate = profile.rate.raceflight.rate.pitch;
    acroPlus = profile.rate.raceflight.acro_plus.pitch;
    break;
  case YAW:
    rate = profile.rate.raceflight.rate.yaw;
    acroPlus = profile.rate.raceflight.acro_plus.yaw;
  break;
  }

  float angleRate = state.rx_filtered.axis[axis] * rate;
  return angleRate = angleRate * (1 + state.rx_filtered_no_expo.axis[axis] * acroPlus);
}

void input_rates_calc(float rates[]) {
  // high-low rates switch
  float rate_multiplier = 1.0;
  if (rx_aux_on(AUX_HIGH_RATES) <= 0) {
    rate_multiplier = profile.rate.low_rate_mulitplier;
  }

  switch (profile.rate.mode) {
  case RATE_MODE_BETAFLIGHT:
    rates[0] = rate_multiplier * calc_bf_rates(0);
    rates[1] = rate_multiplier * calc_bf_rates(1);
    rates[2] = rate_multiplier * calc_bf_rates(2);
    break;
  case RATE_MODE_ACTUAL:
    rates[0] = rate_multiplier * calc_actual_rates(0);
    rates[1] = rate_multiplier * calc_actual_rates(1);
    rates[2] = rate_multiplier * calc_actual_rates(2);
    break;
  case RATE_MODE_KISS:
    rates[0] = rate_multiplier * calc_kiss_rates(0);
    rates[1] = rate_multiplier * calc_kiss_rates(1);
    rates[2] = rate_multiplier * calc_kiss_rates(2);
    break;
  case RATE_MODE_RACEFLIGHT:
    rates[0] = rate_multiplier * calc_raceflight_rates(0);
    rates[1] = rate_multiplier * calc_raceflight_rates(1);
    rates[2] = rate_multiplier * calc_raceflight_rates(2);
    break;
  case RATE_MODE_SILVERWARE:
    rates[0] = rate_multiplier * state.rx_filtered.axis[0] * profile.rate.silverware.max_rate.roll * DEGTORAD;
    rates[1] = rate_multiplier * state.rx_filtered.axis[1] * profile.rate.silverware.max_rate.pitch * DEGTORAD;
    rates[2] = rate_multiplier * state.rx_filtered.axis[2] * profile.rate.silverware.max_rate.yaw * DEGTORAD;
    break;
  }
}
