#include <Adafruit_NeoPixel.h>

// reset pin is pin 1
#define STROBE_IN   3  // physical pin 2
#define ANAMIC_PIN  4  // physical pin 3; also arduino analog input 2
// ground, pin 4
#define PWM_PIN     0  // physical pin 5; pwm, aref, mosi
#define WS2812_OUT  1  // physical pin 6; pwm, miso
#define FRIEND_PIN  2  // physical pin 7; analog input 1, SCK
// power, pin 8

// start with just one pixel in the strip, we assume we can auto-detect length
Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, WS2812_OUT, NEO_GRB + NEO_KHZ800);

uint8_t friends = 1;

////////////////////
/// MATH ROUTINES
/// help with manipulating pixels
////////////////////

// saturating subtract. returns a-b, stopping at 0. assumes 8-bit types
uint8_t satsub_8( uint8_t a, uint8_t b ) {
  if( a >= b )
    return (a - b);
  else
    return 0;
}

// saturating add, returns a+b, stopping at 255. assumes 8-bit types
uint8_t satadd_8( uint8_t a, uint8_t b ) {
  uint16_t c = (uint16_t) a + (uint16_t) b;

  if( c > 255 )
    return (uint8_t) 255;
  else
    return (uint8_t) (c & 0xFF);
}

// saturating subtract, acting on a whole RGB pixel
uint32_t satsub_8p( uint32_t c, uint8_t val ) {
  uint8_t r, g, b;
  r = (c >> 16) & 0xFF;
  g = (c >> 8) & 0xFF;
  b = (c & 0xFF);
  r = satsub_8( r, val );
  g = satsub_8( g, val );
  b = satsub_8( b, val );

  return( ((uint32_t) r) << 16 | ((uint32_t) g) << 8 | b );
}

// saturating add, acting on a whole RGB pixel
uint32_t satadd_8p( uint32_t c, uint8_t val ) {
  uint8_t r, g, b;
  r = (uint8_t) ((c >> 16) & 0xFF);
  g = (uint8_t) ((c >> 8) & 0xFF);
  b = (uint8_t) ((c & 0xFF));
  r = satadd_8( r, val );
  g = satadd_8( g, val );
  b = satadd_8( b, val );

  return( ((uint32_t) r) << 16 | ((uint32_t) g) << 8 | (uint32_t) b );
}

// alpha blend, scale the input color based on a value from 0-255. 255 is full-scale, 0 is black-out.
// uses fixed-point math.
uint32_t alphaPix( uint32_t c, uint8_t alpha ) {
  uint32_t r, g, b;
  r = ((c >> 16) & 0xFF) * alpha;
  g = ((c >> 8) & 0xFF) * alpha;
  b = (c & 0xFF) * alpha;

  r = r / 255;
  g = g / 255;
  b = b / 255;

  return( ((uint32_t) r) << 16 | ((uint32_t) g) << 8 | b );  
}


//////////////////
/// button loop. This should be polled once per main loop, and also polled
/// after every pixel show() command. Returns 1 if the button has just been pressed,
/// 0 if there is no status change to report. Typically, a statement like
///     if( buttonLoop() ) return; 
/// is sufficient to insert after show() to implement the polling inside your animation routine.
//////////////////
#define STROBE_LEN 3000  // in milliseconds

#define MODE_FRIEND 0
#define MODE_STROBE 3

#define FRIEND_PERIOD 1200 // period to integrate friend count over

unsigned long strobe_time = 0;

uint8_t last_fp_st = 1;

uint8_t last_friends = 1;  // i am my own friend.
uint8_t current_friends = 1;
unsigned long friendtimer;

uint8_t mode = MODE_FRIEND;
uint16_t avg_value = 0;

uint8_t buttonLoop() {

  sample_average();

  if( (millis() - friendtimer) > FRIEND_PERIOD ) {
    friendtimer = millis();
    last_friends = current_friends;
    current_friends = 1;
    
//    if( avg_value > 80 )  // we have an extra 4 friends if the music is loud! medium-loud music averages around 64
//      last_friends = last_friends + 4;
    int excitement = map( avg_value, 0, 32, 0, 4 );
    last_friends = last_friends + excitement;
    
    if( last_friends > 10 )
      last_friends = 10;  // just range check this
  }

  do_legs(); // manage leg animation
  if( digitalRead(STROBE_IN) == LOW ) {
    strobe_time = millis();
    mode = MODE_STROBE;
    return 1;
  }

  if( (millis() - strobe_time) > STROBE_LEN ) {
    mode = MODE_FRIEND;
  }

  if( digitalRead(FRIEND_PIN) == LOW ) {
    // should we trigger retval = 1 here??
    if( last_fp_st == 1 ) {
      current_friends++;
    }
    last_fp_st = 0;    
  } 
  else {
    last_fp_st = 1;
  }

  return 0;
}

/////////////////////
/// setup routine, standard arduino libcall
/////////////////////

void setup() {
  pinMode( STROBE_IN, INPUT ); // go into strobe mode
  pinMode( FRIEND_PIN, INPUT ); // friend pinger pin
  pinMode( PWM_PIN, OUTPUT );

  analogReference(DEFAULT);
  pinMode(A2, INPUT);

  strip.begin();  // just initializes pins & stuff

  // strip.resize(6); // fixed size for locator badge 

  friendtimer = millis();
}

/////////////////////
/// main loop - remember that some of the animation routines are blocking and we're relying on
/// the implementor to play fair by calling the buttonLoop() and returning on status change to the main loop.
///
/////////////////////

void loop() {
  buttonLoop();

  switch( mode ) {
  case MODE_FRIEND: // 0
    rainbowCycle(30, 66);  // 66 + depth 64 for individuals, 128 + depth 128 for strangelove
    break;
  case MODE_STROBE:   // 3
    sparkle(0xFFFFFF, 25, 150); // for strobe effect
    break;
  default:
    rainbowCycle(30, 66);  // 66 + depth 64 for individuals, 128 + depth 128 for strangelove
  }     
}

uint16_t leg_brightness = 0;
uint8_t leg_state = 0;

uint8_t leg_gamma(uint8_t b) {
  uint16_t x;
  // we want it to spend more time at the dim state
  x = b * b;
  x = x >> 8; // try square-law scaling?

  x = x + 32;

  if( x >= 254 )
    x = 254;

  return (uint8_t) (x & 0xFF);
}


#define AGC_INTERVAL 1000
#define SAMPLES 128
#define DC_VAL 511

////////
// sample_adc() should be called as often as possible
////////
#define SAMPLE_INTERVAL 2000  // average interval in microseconds
#define SAMPLE_DEPTH 32
#define MID_THRESH_H 665  // 30% over 512
#define MID_THRESH_L 358  // 30% below 512
unsigned long sampletime = 0;
uint16_t sampletime_target = SAMPLE_INTERVAL;
uint16_t samples[SAMPLE_DEPTH];
uint8_t sample_index = 0;
uint16_t last_sample_max = 512;
#define AVG_MAX 1  // 0 = don't average the max value over an interval
void sample_adc() {
  if( (micros() - sampletime) > sampletime_target ) {
    uint16_t temp = analogRead(A2) & 0x3FF;
    if( temp < MID_THRESH_L ) // "rectify" the signal, since 512 is the midpoint
      temp = 1023 - temp;
    if( temp < MID_THRESH_H ) // toss out the lower 10%, it's going to be noise/irrelevant
      temp = MID_THRESH_H;
    temp = temp - MID_THRESH_H; // now we have a rectified, low-noise filtered signal
    samples[sample_index] = temp;
    sample_index = (sample_index + 1) % SAMPLE_DEPTH;
    sampletime_target = micros() + random(SAMPLE_INTERVAL * 2); // randomize to dither aliasing artifacts
  }

  if( sample_index == 0 ) {
    bubblesort(samples);
#if AVG_MAX
    uint32_t avg = 0;
    for( int i = 0; i < SAMPLE_DEPTH / 2; i++ ) {
      avg += samples[i];
    }
    avg = avg / (SAMPLE_DEPTH / 2);
    last_sample_max = (uint16_t) avg;
#else
    last_sample_max = samples[0]; // just take the max from the sort
#endif
    interval_max();
  }
}

// this function attempts to determine the average noise level

#define AVG_SAMPLE_INTERVAL 20000  // average interval in microseconds
#define SAMP_AVG_DEPTH 64  // this is eating 128 bytes out of 512!
uint16_t avg_samples[SAMP_AVG_DEPTH];
uint8_t avg_sample_index = 0;
unsigned long avg_sampletime = 0;
unsigned long avg_sampletime_target = AVG_SAMPLE_INTERVAL;

void sample_average() {
  if( (micros() - avg_sampletime) > avg_sampletime_target ) {
    uint16_t temp = analogRead(A2) & 0x3FF;
    if( temp < MID_THRESH_L ) // "rectify" the signal, since 512 is the midpoint
      temp = 1023 - temp;
    if( temp < MID_THRESH_H ) // toss out the lower 10%, it's going to be noise/irrelevant
      temp = MID_THRESH_H;
    temp = temp - MID_THRESH_H; // now we have a rectified, low-noise filtered signal
    avg_samples[avg_sample_index] = temp;
    avg_sample_index = (avg_sample_index + 1) % SAMP_AVG_DEPTH;
    avg_sampletime_target = micros() + random(AVG_SAMPLE_INTERVAL * 2); // randomize to dither aliasing artifacts
    
    uint32_t local_avg = 0;
    for( int i = 0; i < SAMP_AVG_DEPTH; i++ ) {
      local_avg += avg_samples[i];
    }
    avg_value = (uint16_t) (local_avg / SAMP_AVG_DEPTH);
  }
}

/// this is called by the sampling program every time a new sample set is updated
#define MAX_INTERVAL  2000   // in ms
uint16_t max_interval_value = 512;  // this is the output of the computation
unsigned long maxtime = 0;
uint16_t max_running_value = 0;  // this holds the intermediate value between calls
void interval_max() {
  if( millis() - maxtime > MAX_INTERVAL ) {
    maxtime = millis();
    // update the interval (long-term) value from the computed max value over the past interval
    max_interval_value = max_running_value; // this is the output of the function
    max_running_value = 0; // reset the search intermediate
  }
  // check if our running value needs an updated
  if( last_sample_max > max_running_value ) {
    max_running_value = last_sample_max; 
  }
}

void bubblesort(uint16_t *array) {
  uint8_t n = SAMPLE_DEPTH;
  uint16_t swap;
  for (uint8_t c = 0 ; c < ( n - 1 ); c++) {
    for (uint8_t d = 0 ; d < n - c - 1; d++)  {
      if (array[d] < array[d+1]) /* For increasing order use > */ {
        swap       = array[d];
        array[d]   = array[d+1];
        array[d+1] = swap;
      }
    }
  }
}


/////////////////////////////
//// LEGS -- manage the leg brightness
/////////////////////////////

void do_legs() {  
#if 0  // quick and dirty test code to see if avg_value is working right
  uint8_t temp;
  
  if( avg_value > 255 )
     temp = 255;
  else
     temp = avg_value;
     
  analogWrite(PWM_PIN, temp );
  return;
#endif 

  if( mode == MODE_FRIEND ) {
    analogWrite(PWM_PIN, leg_gamma((uint8_t) leg_brightness));
  } 
  else {
    analogWrite(PWM_PIN, 255); // just bring it all the way bright when we're strobing
  }

  if( leg_state == 0 ) {
    leg_brightness = leg_brightness + 3;
    if( leg_brightness >= 255 ) {
      leg_state = 1;
      leg_brightness = 255;
    }
  } 
  else {
    leg_brightness = leg_brightness - 3;
    if( leg_brightness <= 3 ) {
      leg_state = 0;
      leg_brightness = 0;
    }
  }  

}


/////////////////////////////
//// SPARKLE - stroboscopic effect
/////////////////////////////
void sparkle(uint32_t c, uint16_t time_on, uint16_t time_off) {
  uint16_t target;

  strip.setBrightness(255);  // restore brightness setting

  target = (uint16_t) random(strip.numPixels());
  for( uint16_t i = 0; i < strip.numPixels(); i++ ) {
    if( random(strip.numPixels()) < (strip.numPixels() / 2) )
      strip.setPixelColor(i, c);
    else
      strip.setPixelColor(i, 0);
  }
  strip.show();
  if( buttonLoop() )  return;
  delay( random(time_on) + 5 ); 
  for( uint16_t i = 0; i < strip.numPixels(); i++ ) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  if( buttonLoop() )  return;
  delay( random(time_off) + time_off / 5 );
}

/////////////////////////////
//// makes a rainbow pattern that fades in and out based on a modulation depth, and whose rate
//// depends on the number of "friends" present, as rated on a scale of 1-8
//// "primary" mode of operation for locator badges
/////////////////////////////
#define MOD_DEPTH 64  // 128 for strangelove

void rainbowCycle(uint8_t wait, uint8_t brightness) {
  uint16_t i, j;
  uint8_t bright = brightness;
  uint8_t dir = 1;
  uint8_t rate = friends;

  uint8_t eff_wait;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    
    strip.setBrightness(bright);
    //strip.setBrightness(1);
    if( dir ) {
      bright = satadd_8(bright, rate);
      rate = friends;
      if( (bright == 255) || (bright >= (brightness + MOD_DEPTH))  ) {
        dir = 0;
        if( bright > brightness ) {
          // every time through the loop update our friends status
          if( last_friends > friends ) {
            friends = friends + 1;
          } 
          else if( last_friends < friends ) {
            friends = friends - 1;
          }

          if( friends > 10 )  
            friends = 10; // limit our excitedness so we still have an effect
          if( friends == 0 )
            friends = 1;  // we at least have ourself
        }
      }
    } 
    else {
      bright = satsub_8(bright, rate);
      if( bright < brightness ) {
        rate = 1;
      }
      if( (bright == 0) || (bright <= (brightness - MOD_DEPTH)) ) {
        dir = 1;
      }
    }  

    strip.show();
    if( buttonLoop() )  return;

    if( friends == 1 ) {
      eff_wait = 30;
    } 
    else if( friends == 2 ) {
      eff_wait = 20;
    } 
    else if( friends == 3 ) {
      eff_wait = 18;
    } 
    else if( friends == 4 ) {
      eff_wait = 15;
    } 
    else if( friends == 5 ) {
      eff_wait = 10;
    } 
    else {
      eff_wait = wait / friends;
      if( eff_wait < 3 )
        eff_wait = 3;
    }
    delay(eff_wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


