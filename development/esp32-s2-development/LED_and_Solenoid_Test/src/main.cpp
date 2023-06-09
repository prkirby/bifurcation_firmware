#include <Arduino.h>

#define DRIVER_PIN 21
#define SN1_CTL 36
#define SN2_CTL 37

/**
 * Different modes available:
 *  1 - "Steady"                  {Constant on}
 *  2 - "Blink"                   {Annoying blinking}
 *  3 - "Full Fade - Triangle"    {Fade both LED banks together in pulse - Triangle waveform}
 *  4 - "Full Fade - Saw"         {Fade both LED banks together in pulse - Saw waveform}
 *  5 - "Full Fade - Sine"        {Fade both LED banks together in pulse - Sine waveform}
 *  6 - "Banked Fade - Triangle"  {Fade each LED Banks in and out in pulse - Triangle waveform} NOTE: Doesnt get close to minDIM for some reason
 *  7 - "Banked Fade - Saw"       {Fade each LED Banks in and out in pulse - Saw waveform}
 *  8 - "Banked Fade - Sine"      {Fade each LED Banks in and out in pulse - Sine waveform}
 */
bool status = true;       // on = true | off = false
int mode = 1;             // the current animation mode
int animationTime = 8000; // Milliseconds for a full "loop" of animation
int minDim = 5;           // In 0-255 scale (PWM)
int curMaxDim = 255;      // Current maximum dim, set by potentiometer (in 0-255 PWM scale)

// 50 hz should be considered as minimum dimming.

// Frequency of the LED lights is 150HZ between two polarities by default
// Frequency of phase shift for each LED "Bank" (Lower freq makes things a bit tripper for steady modes)
// IMPORTANT: frequency seems to be needed to be dialed on a string by string basis
int halfPeriod = 500000 / 120; // (1000000us / 2 / frequency) (240hz seems to be sweet spot for low fades?)
unsigned long curMicros = 0;
unsigned long prevMicros = 0;
unsigned long prevDimMicros = 0;
unsigned long curMillis = 0;
unsigned long prevMillis = 0;
int curAnimTime = 0;
bool phase = true;           // True Bank_A is active, False Bank_B is active
int curDim = 255;            // Dim between 0 and 255, adjusted by animation functions (in 0-255 PWM scale)
int curDimB = 255;           // Same as above, but for use in 2 bank patterns only (symmetrical patterns)
bool dir = true;             // true for up, false for down, used in wavetype animations
bool twoBankPattern = false; // Whether or not this is a pattern that switches between each bank
bool bankPhase = false;      // Phase boolean for if we have twoBankPatter (both banks running patterns in symmetry)

/**
 * @brief Get the Cur Rads based on animation percent
 *
 * @return double
 */
double getCurRads()
{
  double animPercent = double(curAnimTime) / double(animationTime);
  return animPercent * PI;
}

/**
 * @brief Map the diming value based on a sin wav function
 * @return int
 */
int sinDimMap(double rads)
{
  double sinVal = sin(rads);
  if (sinVal < 0)
    sinVal *= -1;
  return map(double(255.0 * sinVal), 0.0, 255.0, minDim, curMaxDim);
}

/**
 * @brief Map the diming based on animation time linrarly
 * @return int
 */
int linearDimMap(bool dir = true)
{
  int from = dir ? minDim : curMaxDim;
  int to = dir ? curMaxDim : minDim;

  return map(curAnimTime, 0.0, animationTime, from, to);
}

bool s1open = false;

void swapSolenoids()
{
  if (!s1open)
  {
    digitalWrite(SN1_CTL, HIGH);
    digitalWrite(SN2_CTL, LOW);
    s1open = true;
    Serial.println("SN1 Open | SN2 Closed");
  }
  else
  {
    digitalWrite(SN1_CTL, LOW);
    digitalWrite(SN2_CTL, HIGH);
    s1open = false;
    Serial.println("SN1 Closed | SN2 Open");
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(DRIVER_PIN, OUTPUT);
  digitalWrite(DRIVER_PIN, LOW);

  pinMode(SN1_CTL, OUTPUT);
  pinMode(SN2_CTL, OUTPUT);
  digitalWrite(SN1_CTL, LOW);
  digitalWrite(SN2_CTL, HIGH);
}

void loop()
{
  // Sample time, and set phase for which LED Bank we are currently interested in
  // Check for new dimming value
  curMicros = micros();
  curMillis = millis();

  // Set current anim time based millis
  if (curMillis - prevMillis >= animationTime)
  {
    prevMillis = curMillis;
    dir = !dir;
    swapSolenoids();
  }

  curAnimTime = curMillis - prevMillis;

  // Sin Wave
  curDim = sinDimMap(getCurRads());

  // Triangle Wave
  // curDim = linearDimMap(dir);

  analogWrite(DRIVER_PIN, curDim);
}