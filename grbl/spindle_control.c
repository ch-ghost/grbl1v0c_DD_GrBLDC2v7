/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


//LUT to find corresponding PWM->RPM register value
//Substitutes requested 8bit PWM counter value with empirical RPM data
	static const __flash uint8_t lookup[256] = {
		0,	0,	59,	67,	73,	78,	82,	86,
		89,	93,	96,	98,	101,	103,	106,	108,
		110,	112,	114,	116,	118,	119,	121,	123,
		124,	126,	127,	129,	130,	132,	133,	134,
		136,	137,	138,	139,	141,	142,	143,	144,
		145,	146,	147,	148,	149,	150,	151,	152,
		153,	154,	155,	156,	157,	158,	159,	160,
		161,	162,	162,	163,	164,	165,	166,	166,
		167,	168,	169,	170,	170,	171,	172,	173,
		173,	174,	175,	176,	176,	177,	178,	178,
		179,	180,	180,	181,	182,	182,	183,	184,
		184,	185,	186,	186,	187,	187,	188,	189,
		189,	190,	190,	191,	192,	192,	193,	193,
		194,	194,	195,	196,	196,	197,	197,	198,
		198,	199,	199,	200,	200,	201,	201,	202,
		203,	203,	204,	204,	205,	205,	206,	206,
		207,	207,	208,	208,	208,	209,	209,	210,
		210,	211,	211,	212,	212,	213,	213,	214,
		214,	215,	215,	215,	216,	216,	217,	217,
		218,	218,	219,	219,	219,	220,	220,	221,
		221,	222,	222,	222,	223,	223,	224,	224,
		224,	225,	225,	226,	226,	226,	227,	227,
		228,	228,	228,	229,	229,	230,	230,	230,
		231,	231,	231,	232,	232,	233,	233,	233,
		234,	234,	234,	235,	235,	236,	236,	236,
		237,	237,	237,	238,	238,	238,	239,	239,
		240,	240,	240,	241,	241,	241,	242,	242,
		242,	243,	243,	243,	244,	244,	244,	245,
		245,	245,	246,	246,	246,	247,	247,	247,
		248,	248,	248,	249,	249,	249,	250,	250,
		250,	251,	251,	251,	252,	252,	252,	252,
		253,	253,	253,	254,	254,	254,	255,	255
		};

#ifdef VARIABLE_SPINDLE
  static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
#endif


void spindle_init()
{
  #ifdef VARIABLE_SPINDLE
    // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
    // combined unless configured otherwise.
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
    SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #else
      #ifndef ENABLE_DUAL_AXIS
        SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
      #endif
    #endif
    pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #ifndef ENABLE_DUAL_AXIS
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
    #endif
  #endif

  spindle_stop();
}


uint8_t spindle_get_state()
{
  #ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      // No spindle direction output pin. 
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #else
        if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #endif
    #else
      if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
        #ifdef ENABLE_DUAL_AXIS
          return(SPINDLE_STATE_CW);
        #else
          if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
          else { return(SPINDLE_STATE_CW); }
        #endif
      }
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { 
    #else
      if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
    #endif
      #ifdef ENABLE_DUAL_AXIS    
        return(SPINDLE_STATE_CW);
      #else
        if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      #endif
    }
  #endif
  return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
  #ifdef VARIABLE_SPINDLE
    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
    #else
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
    #endif
  #endif
}


#ifdef VARIABLE_SPINDLE
  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  void spindle_set_speed(uint8_t pwm_value)
  {
//    SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        spindle_stop();
      } else {
	//JTS changes
	SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
	
	pwm_value = lookup[pwm_value]; //LUT to find corresponding PWM->RPM register value
	
	uint8_t temp_pwm = SPINDLE_OCR_REGISTER; //get previous PWM register value
	while(temp_pwm != pwm_value) {
		if(temp_pwm > pwm_value) {
			temp_pwm -= 1;
		} else {
			temp_pwm += 1;
		}
		SPINDLE_OCR_REGISTER = temp_pwm; // Set PWM output level.
		delay_ms(2);
	}
	//end JTS changes
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      }
    #else
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
      } else {
	//JTS changes
	SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
	
	pwm_value = lookup[pwm_value]; //LUT to find corresponding PWM->RPM register value
	
	uint8_t temp_pwm = SPINDLE_OCR_REGISTER; //get previous PWM register value
	while(temp_pwm != pwm_value) {
		if(temp_pwm > pwm_value) {
			temp_pwm -= 1;
		} else {
			temp_pwm += 1;
		}
		SPINDLE_OCR_REGISTER = temp_pwm; // Set PWM output level.
		delay_ms(2);
	}
	//end JTS changes
      }
    #endif
  }


  #ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
  
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
        rpm = RPM_MAX;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= RPM_MIN) {
        if (rpm < 0.0) { // S0 disables spindle
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else {
          rpm = RPM_MIN;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else {
        // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
        #if (N_PIECES > 3)
          if (rpm > RPM_POINT34) {
            pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
          } else 
        #endif
        #if (N_PIECES > 2)
          if (rpm > RPM_POINT23) {
            pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
          } else 
        #endif
        #if (N_PIECES > 1)
          if (rpm > RPM_POINT12) {
            pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
          } else 
        #endif
        {
          pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
        }
      }
      sys.spindle_speed = rpm;
      return(pwm_value);
    }
    
  #else 
  
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_speed = settings.rpm_max;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= settings.rpm_min) {
        if (rpm < 0.0) { // S0 disables spindle
          sys.spindle_speed = 0.0;
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else { // Set minimum PWM output
          sys.spindle_speed = settings.rpm_min;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else { 
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_speed = rpm;
        pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
      }
      return(pwm_value);
    }
    
  #endif
#endif


// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(uint8_t state, float rpm)
#else
  void _spindle_set_state(uint8_t state)
#endif
{
  if (sys.abort) { return; } // Block during abort.

  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0;
    #endif
    spindle_stop();
  
  } else {
    
    #if !defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(ENABLE_DUAL_AXIS)
      if (state == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif
  
    #ifdef VARIABLE_SPINDLE
      // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
      if (settings.flags & BITFLAG_LASER_MODE) { 
        if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
      }
      spindle_set_speed(spindle_compute_pwm_value(rpm));
    #endif
    #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && \
        !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
      // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
      // if the spindle speed value is zero, as its ignored anyhow.
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif    
    #endif
  
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  void spindle_sync(uint8_t state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }
#endif
