/*******************************************************************************
* \file    IS31FL3731.cpp
********************************************************************************
* \author  Jascha Haldemann jh@oxon.ch
* \date    01.01.2017
* \version 1.0
*
* \note    Frame 1... 8 registers can only be read in sw-shutdown (SDB HIGH)
*          The Function Register can be read in both modes.
*
* Recommended workflow:
* Power on -> enableHW() -> configure register -> enableSW() -> display something ->
* disableSW() (matrix blanked) -> read and/or clear register -> disableHW()
*
*******************************************************************************/

/* ================================= Imports ================================ */
#include "IS31FL3731.h"

/* ======================= Module constant declaration ====================== */

/* ======================== Module macro declaration ======================== */

/* ====================== Module class instantiations ======================= */

/* ======================== Public member Functions ========================= */
/** -------------------------------------------------------------------------
  * \fn     begin
  * \brief  inits the LED-Matrix driver
  *
  * \param  clockSpeed  clock speed in Hz
  * \param  i2cAddr     I2C (TWI) address
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::begin(const uint32_t clockSpeed, uint8_t i2cAddr)
  {
    i2cAddr_ = i2cAddr;

    ISSIWire_.begin();
    ISSIWire_.setClock(clockSpeed);

    enableHW();

    /* init all LEDs of all frames */
    for (uint8_t f = FRAME1; f <= FRAME8; f++)
    {
      selectFrame(frame_t(f));

      // //TODO: allow variable LED-Matrix sizes
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

      // // All Blink off   //blup: commented to decrease startup time
      // for (uint8_t i = REG_CONTROL_BLINK_START; i <= REG_CONTROL_BLINK_END; i += 0x01) writeRegister(i, 0x00);

      // // All PWM dc = 0  //blup: commented to decrease startup time
      // for (uint8_t i = REG_PWM_START; i <= REG_PWM_END; i += 0x01) writeRegister(i, 0x00);
    }

    /* init */
    setMode(PICTURE_MODE);
    setAudioSync(false);
    setFrame(FRAME1);

    enableSW();
  }

/** -------------------------------------------------------------------------
  * \fn     end
  * \brief  deinits the LED-Matrix driver
  --------------------------------------------------------------------------- */
  void IS31FL3731::end()
  {
    disableHW();
    ISSIWire_.end();
  }

/** -------------------------------------------------------------------------
  * \fn     enableHW
  * \brief  enables the LED-Matrix driver by HW
  --------------------------------------------------------------------------- */
  void IS31FL3731::enableHW()
  {
    *enPort_ |= (1 << enPin_);          // HW enable (enable r/w)
    //delay(10); // not necessary
    disableSW();
  }

/** -------------------------------------------------------------------------
  * \fn     enableSW
  * \brief  enables the LED-Matrix driver by SW
  --------------------------------------------------------------------------- */
  void IS31FL3731::enableSW()
  {
    selectFrame(FUNCTION_REGISTERS);
    writeRegister(REG_SHUTDOWN, 0x01);  // SW enable (normal mode)
  }

/** -------------------------------------------------------------------------
  * \fn     disableHW
  * \brief  disables the LED-Matrix driver by HW
  --------------------------------------------------------------------------- */
  void IS31FL3731::disableHW()
  {
    clear();
    delay(10);
    disableSW();
    *enPort_ &= ~(1 << enPin_);         // HW shutdown (disable r/w)
  }

/** -------------------------------------------------------------------------
  * \fn     disableSW
  * \brief  disables the LED-Matrix driver by SW
  --------------------------------------------------------------------------- */
  void IS31FL3731::disableSW()
  {
    selectFrame(FUNCTION_REGISTERS);
    writeRegister(REG_SHUTDOWN, 0x00);  // SW shutdown (shutdown mode)
  }

/** -------------------------------------------------------------------------
  * \fn     setFrame
  * \brief  sets the current frame
  *
  * \param  frameNr  frame number (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setFrame(frame_t frameNr)
  {
    if (frameNr > FRAME8) frameNr = FRAME1;
    frame_ = frameNr;
  }

/** -------------------------------------------------------------------------
  * \fn     setMode
  * \brief  sets the current mode
  *
  * \param  modeNr  mode number (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setMode(mode_t modeNr)
  {
    selectFrame(FUNCTION_REGISTERS);
    uint8_t conf = readRegister(REG_CONF);
    conf = (conf & 0b11100111) | (modeNr << 3);
    writeRegister(REG_CONF, conf);
  }

/** -------------------------------------------------------------------------
  * \fn     setDisplayOptions
  * \brief  sets the different display options
  *
  * \param  intensityControl  enable (true) or disable the intensity control
  * \param  blinkEnable       enable (true) or disable the blink function
  * \param  blinkPeriodTime   blink period time in ms
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setDisplayOptions(bool intensityControl, bool blinkEnable, uint8_t blinkPeriodTime)
  {
    uint8_t displayOption = (intensityControl << 5) | (blinkEnable << 3) | blinkPeriodTime;
    selectFrame(FUNCTION_REGISTERS);
    writeRegister(REG_DISPLAY_OPTION, displayOption);
  }

/** -------------------------------------------------------------------------
  * \fn     drawPixel
  * \brief  draws a pixel on the display
  *
  * \param  x           coordinate of the pixel
  * \param  y           coordinate of the pixel
  * \param  brightness  brightness value
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::drawPixel(uint8_t x, uint8_t y, uint8_t brigtness)
  {
    if ((x >= x_) || (y >= y_)) return;

    selectFrame(frame_);
    writeRegister(REG_PWM_START + x + y*MAX_NUMBER_OF_COLUMNS, brigtness);
  }

/** -------------------------------------------------------------------------
  * \fn     getPixel
  * \brief  reads the state of a pixel on the display
  *
  * \param  x   coordinate of the pixel
  * \param  y   coordinate of the pixel
  * \return brightness value
  --------------------------------------------------------------------------- */
  uint8_t IS31FL3731::getPixel(uint8_t x, uint8_t y)
  {
    disableSW();
    selectFrame(frame_);
    uint8_t brigtness = readRegister(REG_PWM_START + x + y*MAX_NUMBER_OF_COLUMNS);
    enableSW();
    return brigtness;
}

/** -------------------------------------------------------------------------
  * \fn     clear
  * \brief  clears the screen
  --------------------------------------------------------------------------- */
  void IS31FL3731::clear()
  {
    fillScreen(0);
  }

/** -------------------------------------------------------------------------
  * \fn     fillScreen
  * \brief  fills the screen with given brightness
  *
  * \param  brightness  brightness value
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::fillScreen(uint8_t brightness)
  {
    selectFrame(frame_);

    for (uint8_t y = 0; y < MAX_NUMBER_OF_ROWS; y++)
    {
      ISSIWire_.beginTransmission(i2cAddr_);
      ISSIWire_.write(REG_PWM_START + y*MAX_NUMBER_OF_COLUMNS);
      for (uint8_t x = 0; x < MAX_NUMBER_OF_COLUMNS; x++) ISSIWire_.write(brightness);
      ISSIWire_.endTransmission();
    }
  }

/* Picture Mode */
/** -------------------------------------------------------------------------
  * \fn     displayFrame
  * \brief  display given frame
  *
  * \param  frameNr   frame number (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::displayFrame(frame_t frameNr)
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

/** -------------------------------------------------------------------------
  * \fn     setBreathControl
  * \brief  configurates the breath control function
  *
  * \param  enable          enables (true) or disables the breath control
  * \param  extinguishTime  extinguish time value
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setBreathControl(bool enable, uint8_t extinguishTime)
  {
    selectFrame(FUNCTION_REGISTERS);
    writeRegister(REG_CONTROL_BREATH2, ((enable << 4) | extinguishTime));
  }

/** -------------------------------------------------------------------------
  * \fn     setAudioSync
  * \brief  configurates the audio sync function
  *         (enables to modulate the intensity of the matrix by the audio
  *         input signal)
  *
  * \param  enable  enables (true) or disables the audio sync function
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setAudioSync(bool enable)
  {
    selectFrame(FUNCTION_REGISTERS);
    writeRegister(REG_AUDIO_SYNC, enable);
  }

/* Auto Frame Play Mode */
/** -------------------------------------------------------------------------
  * \fn     setStartFrame
  * \brief  sets the start frame
  *
  * \param  frameNr  frame number (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setStartFrame(frame_t frameNr)
  {
    selectFrame(FUNCTION_REGISTERS);
    uint8_t conf = readRegister(REG_CONF);
    conf = (conf & 0b11111000) | uint8_t(frameNr);
    writeRegister(REG_CONF, conf);
  }

/** -------------------------------------------------------------------------
  * \fn     setNumberOfLoops
  * \brief  sets the number of loops
  *
  * \param  loop  number of loops (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setNumberOfLoops(numberOfLoops_t loop)
  {
    uint8_t controlAutoPlay1 = readRegister(REG_CONTROL_AUTO_PLAY1);
    controlAutoPlay1 = (controlAutoPlay1 & 0b10001111) | (loop << 4);
    writeRegister(REG_CONTROL_AUTO_PLAY1, controlAutoPlay1);
  }

/** -------------------------------------------------------------------------
  * \fn     setNumberOfFrames
  * \brief  sets the number of frames
  *
  * \param  frame  number of frames (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setNumberOfFrames(numberOfFrames_t frame)
  {
    uint8_t controlAutoPlay1 = readRegister(REG_CONTROL_AUTO_PLAY1);
    controlAutoPlay1 = (controlAutoPlay1 & 0b11111000) | uint8_t(frame);
    writeRegister(REG_CONTROL_AUTO_PLAY1, controlAutoPlay1);
  }

/** -------------------------------------------------------------------------
  * \fn     setDelayBetweenFrames
  * \brief  sets the delay between the frames
  *
  * \param  delay  delay in ms
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setDelayBetweenFrames(uint16_t delay)
  {
    delay /= TYP_FRAME_DELAY_TIME;
    writeRegister(REG_CONTROL_AUTO_PLAY2, (delay & 0x3F));
  }

/** -------------------------------------------------------------------------
  * \fn     getMovieFinished
  * \brief  returns the state of the movie
  *
  * \return true if movie has finished
  --------------------------------------------------------------------------- */
  bool IS31FL3731::getMovieFinished()
  {
    return (readRegister(REG_FRAME_STATE) & 0b00010000);
  }

/** -------------------------------------------------------------------------
  * \fn     getCurrentFrameDisplay
  * \brief  returns the current frame number
  *
  * \return current frame number (see enumerator in the header file)
  --------------------------------------------------------------------------- */
  IS31FL3731::frame_t IS31FL3731::getCurrentFrameDisplay()
  {
    return frame_t(readRegister(REG_FRAME_STATE) & 0b00000111);
  }

/* Audio Frame Play Mode */
/** -------------------------------------------------------------------------
  * \fn     setAudioADCRate
  * \brief  sets the audio adc rate
  *
  * \param  rate  adc rate
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setAudioADCRate(uint8_t rate)
  {
    writeRegister(REG_AUDIO_ADC_RATE, rate);
  }

/** -------------------------------------------------------------------------
  * \fn     setAGCControl
  * \brief  configurates the AGC control
  *
  * \param  mode    AGC control mode
  * \param  enable  enable (true) or diable the AGC control
  * \param  gain    AGC control gain
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::setAGCControl(bool mode, bool enable, uint8_t gain)
  {
    writeRegister(REG_CONTROL_AGC, ((mode << 4) | (enable << 3) | gain));
  }

/* ======================= Private member Functions ========================= */
/** -------------------------------------------------------------------------
  * \fn     selectFrame
  * \brief  selects the current frame
  *
  * \param  frameNr   frame number (see enumerator in the header file)
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::selectFrame(frame_t frameNr)
  {
    ISSIWire_.beginTransmission(i2cAddr_);
    ISSIWire_.write(REG_CMD);
    ISSIWire_.write(frameNr);
    ISSIWire_.endTransmission();

    if (frameNr <= FRAME8) frame_ = frameNr;  // set current frame
  }

/** -------------------------------------------------------------------------
  * \fn     readRegister
  * \brief  reads the register of given address
  *
  * \param  address   address of the LED-Matrix driver
  * \return content of the given address
  --------------------------------------------------------------------------- */
  uint8_t IS31FL3731::readRegister(uint8_t address)
  {
    ISSIWire_.beginTransmission(i2cAddr_);
    ISSIWire_.write(address);
    ISSIWire_.endTransmission();

    ISSIWire_.requestFrom(i2cAddr_, uint8_t(1));
    return ISSIWire_.read();
  }

/** -------------------------------------------------------------------------
  * \fn     writeRegister
  * \brief  writes a value to the given register address
  *
  * \param  address   address of the LED-Matrix driver
  * \param  value     value to write
  * \return None
  --------------------------------------------------------------------------- */
  void IS31FL3731::writeRegister(uint8_t address, uint8_t value)
  {
    ISSIWire_.beginTransmission(i2cAddr_);
    ISSIWire_.write(address);
    ISSIWire_.write(value);
    ISSIWire_.endTransmission();
  }
