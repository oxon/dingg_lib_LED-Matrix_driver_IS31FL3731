/**-----------------------------------------------------------------------------
 * \file    IS31FL3731.cpp
 * \author  jh
 * \date    xx.01.2017
 * @{
 -----------------------------------------------------------------------------*/

/*
 * Note:
 * Frame 1... 8 Registers can only be read in software shutdown (SDB HIGH).
 * The Function Register can be read in both modes.
 *
 * Recommended workflow:
 * Power on -> enableHW() -> configure register -> enableSW() -> display something ->
 * disableSW() (matrix blanked) -> read and/or clear register -> disableHW()
 */

 /* Includes --------------------------------------------------- */
#include "IS31FL3731.h"

/* Public ----------------------------------------------------- */
void IS31FL3731::begin(uint8_t i2cAddr)
{
  i2cAddr_ = i2cAddr;

  ISSIWire_.begin();

  enableHW();

  /* init all LEDs of all frames */
  for (uint8_t f = FRAME1; f <= FRAME8; f++)
  {
    selectFrame(frame(f));

    // //TODO: allow Variable LED-Matrix sizes
    // // LEDs of Matrix A
    // for (uint8_t i = REG_CONTROL_LED_A_START; i <= REG_CONTROL_LED_A_END; i += 0x02)
    // {
    //   if (i < y_*2)
    //   {
    //     if (x_ >= 8) writeRegister(i, 0xFF);
    //     else ...
    //   }
    // }

    //blup: only 8x8 for now
    // LEDs of Matrix A
    for (uint8_t i = REG_CONTROL_LED_A_START; i <= REG_CONTROL_LED_A_END; i += 0x02)
    {
      if (i == 0x10) writeRegister(i, 0x00);  // Column      9: off
      else writeRegister(i, 0xFF);            // Column 1... 8: on
    }
    // LEDs of Matrix B
    for (uint8_t i = REG_CONTROL_LED_B_START; i <= REG_CONTROL_LED_B_END; i += 0x02) writeRegister(i, 0x00); // Column 1... 9: off

    // All Blink off
    for (uint8_t i = REG_CONTROL_BLINK_START; i <= REG_CONTROL_BLINK_END; i += 0x01) writeRegister(i, 0x00);

    // All PWM dc = 0
    for (uint8_t i = REG_PWM_START; i <= REG_PWM_END; i += 0x01) writeRegister(i, 0x00);
  }

  /* init */
  setMode(PICTURE_MODE);
  setAudioSync(false);
  setFrame(FRAME1);

  enableSW();
}

void IS31FL3731::end()
{
  disableHW();
  ISSIWire_.end();
}

void IS31FL3731::enableHW()
{
  *enPort_ |= (1 << enPin_);          // HW enable (enable r/w)
  delay(100);
  disableSW();
}

void IS31FL3731::enableSW()
{
  selectFrame(FUNCTION_REGISTERS);
  writeRegister(REG_SHUTDOWN, 0x01);  // SW enable (normal mode)
}

void IS31FL3731::disableHW()
{
  clear();
  delay(10);
  disableSW();
  *enPort_ &= ~(1 << enPin_);         // HW shutdown (disable r/w)
}

void IS31FL3731::disableSW()
{
  selectFrame(FUNCTION_REGISTERS);
  writeRegister(REG_SHUTDOWN, 0x00);  // SW shutdown (shutdown mode)
}

void IS31FL3731::setFrame(frame frameNr)
{
  if (frameNr > FRAME8) frameNr = FRAME1;
  frame_ = frameNr;
}

void IS31FL3731::setMode(mode modeNr)
{
  selectFrame(FUNCTION_REGISTERS);
  uint8_t conf = readRegister(REG_CONF);
  conf = (conf & 0b11100111) | (modeNr << 3);
  writeRegister(REG_CONF, conf);
}

void IS31FL3731::setDisplayOptions(bool intensityControl, bool blinkEnable, uint8_t blinkPeriodTime)  // in ms
{
  uint8_t displayOption = (intensityControl << 5) | (blinkEnable << 3) | blinkPeriodTime;
  selectFrame(FUNCTION_REGISTERS);
  writeRegister(REG_DISPLAY_OPTION, displayOption);
}

void IS31FL3731::drawPixel(uint8_t x, uint8_t y, uint8_t brigtness)
{
  if ((x >= x_) || (y >= y_)) return;

  selectFrame(frame_);
  writeRegister(REG_PWM_START + x + y*MAX_NUMBER_OF_COLUMNS, brigtness);
}

uint8_t IS31FL3731::getPixel(uint8_t x, uint8_t y)
{
  disableSW();
  selectFrame(frame_);
  uint8_t brigtness = readRegister(REG_PWM_START + x + y*MAX_NUMBER_OF_COLUMNS);
  enableSW();
  return brigtness;
}

void IS31FL3731::clear()
{
  fillScreen(0);
}


void IS31FL3731::fillScreen(uint8_t brightness)  {
  selectFrame(frame_);

  for (uint8_t y = 0; y < MAX_NUMBER_OF_ROWS; y++)
  {
    ISSIWire_.beginTransmission(i2cAddr_);
    ISSIWire_.write(REG_PWM_START + y*MAX_NUMBER_OF_COLUMNS);
    for (uint8_t x = 0; x < MAX_NUMBER_OF_COLUMNS; x++) ISSIWire_.write(brightness);  // write 0 for each LED
    ISSIWire_.endTransmission();
  }
}

/* Picture Mode */
void IS31FL3731::displayFrame(frame frameNr)
{
  if (frameNr > FRAME8) frameNr = FRAME1;
  selectFrame(FUNCTION_REGISTERS);
  writeRegister(REG_PICTURE_DISPLAY, frameNr);
}

// void IS31FL3731::setFadeOutTime(uint16_t time)  // in ms
// {
//   //TODO: FOT = 26ms * 2^time
//   //time /= TYP_FADE_OUT_TIME;
//   //selectFrame(FUNCTION_REGISTERS);
//   //writeRegister(REG_CONTROL_BREATH1, );
// }

// void IS31FL3731::setFadeInTime(uint16_t time)  // in ms
// {
//   //TODO: FIT = 26ms * 2^time
//   //time /= TYP_FADE_IN_TIME;
//   //selectFrame(FUNCTION_REGISTERS);
//   //writeRegister(REG_CONTROL_BREATH1, );
// }

void IS31FL3731::setBreathControl(bool enable, uint8_t extinguishTime)
{
  selectFrame(FUNCTION_REGISTERS);
  writeRegister(REG_CONTROL_BREATH2, ((enable << 4) | extinguishTime));
}

void IS31FL3731::setAudioSync(bool enable)  // enable to modulate the intensity of the matrix by the audio input signal
{
  selectFrame(FUNCTION_REGISTERS);
  writeRegister(REG_AUDIO_SYNC, enable);
}

/* Auto Frame Play Mode */
void IS31FL3731::setStartFrame(frame frameNr)
{
  selectFrame(FUNCTION_REGISTERS);
  uint8_t conf = readRegister(REG_CONF);
  conf = (conf & 0b11111000) | frameNr;
  writeRegister(REG_CONF, conf);
}

void IS31FL3731::setNumberOfLoops(numberOfLoops loop)
{
  uint8_t controlAutoPlay1 = readRegister(REG_CONTROL_AUTO_PLAY1);
  controlAutoPlay1 = (controlAutoPlay1 & 0b10001111) | (loop << 4);
  writeRegister(REG_CONTROL_AUTO_PLAY1, controlAutoPlay1);
}

void IS31FL3731::setNumberOfFrames(numberOfFrames frame)
{
  uint8_t controlAutoPlay1 = readRegister(REG_CONTROL_AUTO_PLAY1);
  controlAutoPlay1 = (controlAutoPlay1 & 0b11111000) | frame;
  writeRegister(REG_CONTROL_AUTO_PLAY1, controlAutoPlay1);
}

void IS31FL3731::setDelayBetweenFrames(uint16_t delay)  // in ms
{
  delay /= TYP_FRAME_DELAY_TIME;
  writeRegister(REG_CONTROL_AUTO_PLAY2, (delay & 0x3F));
}

bool IS31FL3731::getMovieFinished()
{
  return (readRegister(REG_FRAME_STATE) & 0b00010000);
}

uint8_t IS31FL3731::getCurrentFrameDisplay()
{
  return frame(readRegister(REG_FRAME_STATE) & 0b00000111);
}

/* Audio Frame Play Mode */
void IS31FL3731::setAudioADCRate(uint8_t rate)
{
  writeRegister(REG_AUDIO_ADC_RATE, rate);
}

void IS31FL3731::setAGCControl(bool mode, bool enable, uint8_t gain)
{
  writeRegister(REG_CONTROL_AGC, ((mode << 4) | (enable << 3) | gain));
}

/* Private ---------------------------------------------------- */
void IS31FL3731::selectFrame(frame frameNr)
{
  ISSIWire_.beginTransmission(i2cAddr_);
  ISSIWire_.write(REG_CMD);
  ISSIWire_.write(frameNr);
  ISSIWire_.endTransmission();

  if (frameNr <= FRAME8) frame_ = frameNr;  // set current frame
}

uint8_t IS31FL3731::readRegister(uint8_t address)
{
  ISSIWire_.beginTransmission(i2cAddr_);
  ISSIWire_.write(address);
  ISSIWire_.endTransmission();

  ISSIWire_.requestFrom(i2cAddr_, uint8_t(1));
  return ISSIWire_.read();
}

void IS31FL3731::writeRegister(uint8_t address, uint8_t value)
{
  ISSIWire_.beginTransmission(i2cAddr_);
  ISSIWire_.write(address);
  ISSIWire_.write(value);
  ISSIWire_.endTransmission();
}

/**
 * @}
 */
