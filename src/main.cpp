/******************************************************************************
 * Copyright (C) 2022 dyifm.com
 * This file is part of FM3-public <https://github.com/dyifm/fm3-public>.
 *
 * fm3-public is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * fm3-public is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fm3-public.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
#include "global.h"
#include <ESP_FlexyStepper.h>
#include <driver/adc.h>
#include "WiFi.h"
#include "esp_task_wdt.h"
#include "util/MovingAvg.h"

// IO pin assignments
#define DRIVER_STEP_PIN 19
#define DRIVER_DIR_PIN 18
// #define EMERGENCY_PIN     13
#define LIMIT_PIN 16 // define the IO pin where the limit switches are connected to (switches in series in normally closed setup against ground)

#define OFFSET_PIN 34 // 26
#define THRUST_PIN 35 // 25
#define SPEED_PIN 27
#define DRIVER_EN_PIN 5

// ADC Sample
#define ADC_COUNT 3
#define ADC_OFFSET_IDX 0
#define ADC_THRUST_IDX 1
#define ADC_SPEED_IDX 2
#define MIN_POS_DELTA 0.05 // 5%

struct TSample
{
    int adcPIN[ADC_COUNT] =
        {OFFSET_PIN, THRUST_PIN, SPEED_PIN};

    float adcOffset;
    float adcThrust;
    float adcSpeed;

    float adcOffsetMM;
    float adcThrustMM;
    float adcSpeedMMpS;

    MovingAvg adcRAWOffset;
    MovingAvg adcRAWThrust;
    MovingAvg adcRAWSpeed;

    uint16_t adcOffsetMax = 1;
    uint16_t adcThrustMax = 1;
    uint16_t adcSpeedMax = 1;
};

enum EState
{
  SNONE,
  SINITIALIZING,
  SHOMING_APPROACH,
  SHOMING_OFFSET,
  SCALIBRATE,
  SRUNNING,
  SERROR
};

enum ETransition
{
  TNONE,
  TNONE__HOMING_APPROACH,
  THOMING_APPROACH__HOMING_OFFSET,
  THOMING_OFFSET__CALIBRATE,
  TCALIBRATE__RUNNING,
  TX_ERROR
};

// Driver settings
#define STEPS_PER_MM 12.74
#define MAX_DISTANCE_TO_TRAVEL_IN_MM 120
#define DISTANCE_TO_TRAVEL_IN_MM 120//140
#define DISTANCE_TO_TRAVEL_IN_STEPS DISTANCE_TO_TRAVEL_IN_MM *STEPS_PER_MM // 157mm == 2000steps //0.0785mm/step
#define SPEED_IN_STEPS_PER_SECOND 7000                                     // 7000 //15000 //300
#define ACCELERATION_IN_STEPS_PER_SECOND 50000
#define DECELERATION_IN_STEPS_PER_SECOND 50000

#define INITIAL_POSITION_OFFSET_MM -50//-50 // -100 / STEPS_PER_MM
#define MIN_TRAVEL_MM 0.05 * DISTANCE_TO_TRAVEL_IN_MM
// ----------------------------------------------------------------------------

ESP_FlexyStepper mStepper;
TSample mSample;
bool moveOut = false;

EState mState = SNONE;
ETransition mCurrentTransition = TNONE__HOMING_APPROACH;
volatile byte limitSwitchState = LOW;
byte oldConfirmedLimitSwitchState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100; // the minimum delay in milliseconds to check for bouncing of the switch. Increase this slighlty if you switches tend to bounce a lot
// bool mTargetReached = false;
bool homing = false;

void motorControlTask(void* parameter);
void hmiTask(void* parameter);

// ----------------------------------------------------------------------------

ICACHE_RAM_ATTR void almISR()
{

  Log.infoln(F("almISR"));
}

// ----------------------------------------------------------------------------

double scaleADC(uint16_t reading, uint16_t maxValue)
{
  double retVal = (double) map(reading, 0, maxValue, 0, 100) / (double) 100;
  // Log.infoln("scaleADC - reading=" + String(reading) +", retVal=" + String(retVal));

  if(retVal > 1)
    retVal = 1;
  if(retVal < 0)
    retVal = 0;

  return retVal;
}

// ----------------------------------------------------------------------------

bool approx_equal(double x, double y, double delta)
{
  if(x == 0)
    return fabs(y) <= delta;
  if(y == 0)
    return fabs(x) <= delta;

  return fabs(x - y) / max(fabs(x), fabs(y)) <= delta;
}

// ----------------------------------------------------------------------------

void readADC()
{
  uint16_t sample = analogRead(mSample.adcPIN[ADC_SPEED_IDX]);
  mSample.adcRAWSpeed.add(sample);
  mSample.adcSpeed = (scaleADC(mSample.adcRAWSpeed.get(), mSample.adcSpeedMax));
  mSample.adcSpeedMMpS = mSample.adcSpeed * SPEED_IN_STEPS_PER_SECOND;

  mSample.adcRAWOffset.add(analogRead(OFFSET_PIN));
  mSample.adcOffset = (scaleADC(mSample.adcRAWOffset.get(), mSample.adcOffsetMax));
  mSample.adcOffsetMM = mSample.adcOffset * DISTANCE_TO_TRAVEL_IN_MM;

  mSample.adcRAWThrust.add(analogRead(mSample.adcPIN[ADC_THRUST_IDX]));
  mSample.adcThrust = (scaleADC(mSample.adcRAWThrust.get(), mSample.adcThrustMax));

  float remainingMM = DISTANCE_TO_TRAVEL_IN_MM - mSample.adcOffsetMM;
  mSample.adcThrustMM = remainingMM * mSample.adcThrust + mSample.adcOffsetMM;
}

// ----------------------------------------------------------------------------

void initADC()
{

  Log.infoln(F("initADC"));

  adcAttachPin(OFFSET_PIN);
  adcAttachPin(THRUST_PIN);
  adcAttachPin(SPEED_PIN);
}

// ----------------------------------------------------------------------------

void limitSwitchHandler()
{
  limitSwitchState = digitalRead(LIMIT_PIN);
  lastDebounceTime = millis();
}

void initDriver()
{

  Log.infoln(F("initDriver"));

  // Interrupts
  // pinMode(DRIVER_ALM_PIN, INPUT_PULLUP);
  // attachInterrupt(DRIVER_ALM_PIN, almISR, CHANGE);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), limitSwitchHandler, CHANGE);

  // connect and configure the stepper motor to its IO pins
  mStepper.connectToPins(DRIVER_STEP_PIN, DRIVER_DIR_PIN);

  // set the speed and acceleration rates for the stepper motor
  mStepper.setSpeedInStepsPerSecond(0.1 * SPEED_IN_STEPS_PER_SECOND);
  mStepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  mStepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  mStepper.setStepsPerMillimeter(STEPS_PER_MM);

  // mStepper.registerTargetPositionReachedCallback(targetPositionReachedCallback);

  pinMode(DRIVER_EN_PIN, OUTPUT);
  digitalWrite(DRIVER_EN_PIN, LOW);
}

void processHomingApproach()
{
  Log.infoln(F("processHomingApproach"));

  mStepper.moveToHomeInMillimeters(-1, 0.1 * SPEED_IN_STEPS_PER_SECOND, MAX_DISTANCE_TO_TRAVEL_IN_MM, LIMIT_PIN); // goToLimitAndSetAsHome(0, DISTANCE_TO_TRAVEL_IN_STEPS);
  Log.infoln(F("processHomingApproach - Moving towards limit switch"));

  mStepper.setCurrentPositionAsHomeAndStop();

  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), limitSwitchHandler, CHANGE);

  mCurrentTransition = THOMING_APPROACH__HOMING_OFFSET;
  delay(1000);
}

void processHomingOffset()
{
  Log.infoln(F("processHomingOffset"));

  mStepper.clearLimitSwitchActive();
  mStepper.moveToPositionInMillimeters(INITIAL_POSITION_OFFSET_MM);
  Log.infoln(F("processHomingOffset - Moving away from limit switch"));

  mStepper.setCurrentPositionAsHomeAndStop();
  Log.infoln(F("processHomingOffset - New home set"));

  if(limitSwitchState == LOW)
  {
    mCurrentTransition = THOMING_OFFSET__CALIBRATE;
  }
  else
  {
    mCurrentTransition = TX_ERROR;
    Log.errorln(F("processHomingOffset - Failed to move to offset"));
  }

  delay(1000);
}

void processCalibrate()
{
  int size = mSample.adcRAWOffset.getSize();

  Log.infoln(F("wait till all sliders are zero"));
  do
  {
    for(int i = 0; i < size; i++)
      readADC();
  } while((10 < mSample.adcRAWOffset.get()) && (10 < mSample.adcRAWSpeed.get()) && (10 < mSample.adcRAWThrust.get()));

  Log.infoln(F("wait till all sliders are not zero"));
  do
  {
    for(int i = 0; i < size; i++)
      readADC();
  } while((10 > mSample.adcRAWOffset.get()) || (10 > mSample.adcRAWSpeed.get()) || (10 > mSample.adcRAWThrust.get()));

  Log.infoln(F("wait till all sliders are back to zero while recording max values"));
  do
  {
    for(int i = 0; i < size; i++)
      readADC();
    uint16_t sample = mSample.adcRAWOffset.get();
    mSample.adcOffsetMax = (sample > mSample.adcOffsetMax) ? sample : mSample.adcOffsetMax;

    sample = mSample.adcRAWSpeed.get();
    mSample.adcSpeedMax = (sample > mSample.adcSpeedMax) ? sample : mSample.adcSpeedMax;

    sample = mSample.adcRAWThrust.get();
    mSample.adcThrustMax = (sample > mSample.adcThrustMax) ? sample : mSample.adcThrustMax;
  } while((10 < mSample.adcRAWOffset.get()) && (10 < mSample.adcRAWSpeed.get()) && (10 < mSample.adcRAWThrust.get()));

  Log.infoln(F("RAW Calibration finished. Max Offset=%d, Max Speed=%d, Max Thrust=%d"), mSample.adcOffsetMax, mSample.adcSpeedMax, mSample.adcThrustMax);
  mCurrentTransition = TCALIBRATE__RUNNING;
}

void processRunning()
{
  mStepper.processMovement();

  // just move the stepper back and forth in an endless loop
  if(mStepper.getDistanceToTargetSigned() == 0)
    moveOut = !moveOut;

  if(oldConfirmedLimitSwitchState == 0)
  {
    // if (thrust > MIN_TRAVEL_MM)
    {

      float targetPosition = (moveOut) ? -mSample.adcThrustMM : -mSample.adcOffsetMM;

      mStepper.setSpeedInStepsPerSecond(mSample.adcSpeedMMpS);
      mStepper.setTargetPositionInMillimeters(targetPosition);
    }
  }
}

void processLimitSwitch()
{
  if(limitSwitchState != oldConfirmedLimitSwitchState && (millis() - lastDebounceTime) > debounceDelay)
  {
    oldConfirmedLimitSwitchState = limitSwitchState;

    // active high switch configuration (NC connection with internal pull up)
    mStepper.setLimitSwitchActive(mStepper.LIMIT_SWITCH_COMBINED_BEGIN_AND_END); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is agtive
    if(limitSwitchState == HIGH)
    {
      Log.infoln(F("setLimitSwitchActive"));

      mStepper.setLimitSwitchActive(mStepper.LIMIT_SWITCH_COMBINED_BEGIN_AND_END); // this will cause to stop any motion that is currently going on and block further movement in the same direction as long as the switch is agtive
    }
    else
    {
      Log.infoln(F("clearLimitSwitchActive"));
      mStepper.clearLimitSwitchActive(); // clear the limit switch flag to allow movement in both directions again
    }
  }
}

void processState()
{

  switch(mState)
  {
    case (SHOMING_APPROACH):
      processHomingApproach();
      break;
    case (SHOMING_OFFSET):
      processHomingOffset();
      break;
    case (SCALIBRATE):
      processCalibrate();
      break;
    case (SRUNNING):
      processRunning();
      break;
    case (SERROR):
      break;
  }
}

void processTransition()
{

  EState previousState = mState;

  switch(mCurrentTransition)
  {
    case (TNONE):
      break;
    case (TNONE__HOMING_APPROACH):
      mState = SHOMING_APPROACH;
      break;
    case (THOMING_APPROACH__HOMING_OFFSET):
      mState = SHOMING_OFFSET;
      break;
    case (THOMING_OFFSET__CALIBRATE):
      mState = SCALIBRATE;
      break;
    case (TCALIBRATE__RUNNING):
      mState = SRUNNING;
      break;
    case (TX_ERROR):
      mState = SERROR;
      break;
  }

  if(previousState != mState)
    Log.infoln(F("INFO: processTransition %i > %i"), previousState, mState);
}

// // ----------------------------------------------------------------------------

void loop()
{
  Log.infoln(F("AVG Speed=%Fmm/s [%F], Thrust=%Fmm [%F], Depth=%Fmm [%F], limitSW=%i shallSW=%i"),
      mSample.adcSpeedMMpS, mSample.adcRAWSpeed.get(), mSample.adcThrustMM, mSample.adcRAWThrust.get(), mSample.adcOffsetMM, mSample.adcRAWOffset.get(), limitSwitchState, digitalRead(LIMIT_PIN));

  delay(1000);
}

// ----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Log.infoln(F("\n\n********************************************"));
  Log.infoln(F("MAC: %x"), WiFi.macAddress());
  Log.infoln(F("SDK: %s"), ESP.getSdkVersion());
  Log.infoln(F("FW: %s"), APPVERSION.stringVersionDateTime.c_str());
  Log.infoln(F("********************************************"));

  WiFi.mode(WIFI_OFF);
  btStop();

  initADC();
  initDriver();

  disableCore0WDT(); // we have to disable the Watchdog timer to prevent it from rebooting the ESP all the time another option would be to add a vTaskDelay but it would slow down the stepper

  xTaskCreatePinnedToCore(
      motorControlTask, /* Task function. */
      "motorControlTask", /* name of task. */
      10000, /* Stack size of task */
      NULL, /* parameter of the task */
      5, /* priority of the task */
      NULL, /* Task handle to keep track of created task */
      0);

  xTaskCreatePinnedToCore(
      hmiTask, /* Task function. */
      "hmiTask", /* name of task. */
      10000, /* Stack size of task */
      NULL, /* parameter of the task */
      2, /* priority of the task */
      NULL, /* Task handle to keep track of created task */
      1);
}

// ----------------------------------------------------------------------------

void motorControlTask(void* parameter)
{
  for(;;)
  {
    processLimitSwitch();
    processTransition();
    processState();
  }
}

// ----------------------------------------------------------------------------

void hmiTask(void* parameter)
{
  for(;;)
  {
    readADC();

    delay(1);
  }
}
