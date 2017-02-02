/**-----------------------------------------------------------------------------
 * \file    IS31FL3731.h
 * \author  jh
 * \date    xx.01.2017
 *
 * \version 1.0
 *
 * \brief   The IS31FL3731 can drive a 16x9 (144) LED-Matrix.
 *          It is able to control the brightness of every LED individually by
 *          setting a 8-bit PWM duty cycle.
 *          It consists of 8 frames that you can prepare beforehand and display
 *          with either the displayFrame()-function (Picture Mode) or in a loop
 *          (Auto Frame Play Mode).
 * 
 * \note:   The IS31FL3731 is very critical when it comes to high temperatures.
 *          Do not exceed 260°C (for > 30s) in the production process.
 *
 * @{
 -----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -----------------------*/
#ifndef IS31FL3731_H_
#define IS31Fl3731_H_

/* Includes --------------------------------------------------- */
#include <Arduino.h>
#include <Wire.h>
#include <LEDMatrix.h>
//#include <Adafruit_GFX.h>

/* Typedefs ----------------------------------------------------*/

/* Macros ----------------------------------------------------- */
#ifndef _swap_int16_t
  #define _swap_int16_t(a, b) {int16_t temp = a; a = b; b = temp;}
#endif

/* Defines -----------------------------------------------------*/

/* Class ------------------------------------------------------ */
class IS31FL3731 : public LEDMatrix
//class IS31FL3731 : public Adafruit_GFX
{
public:
  /* constructor(s) & deconstructor */
  IS31FL3731(TwoWire& ISSIWire, volatile uint8_t *enPort, uint8_t enPin, uint8_t x = 9, uint8_t y = 16) : 
    ISSIWire_(ISSIWire), enPort_(enPort), enPin_(enPin), LEDMatrix(x, y) {};
  ~IS31FL3731() {};

  /* public constants (static) */
  // ...

  /* public enumerations */
  enum mode : uint8_t {PICTURE_MODE = 0, AUTO_FRAME_PLAY_MODE = 1, AUDIO_FRAME_PLAY_MODE = 2};
  enum frame : uint8_t {FRAME1 = 0, FRAME2 = 1, FRAME3 = 2, FRAME4 = 3, FRAME5 = 4, FRAME6 = 5, FRAME7 = 6, FRAME8 = 7, FUNCTION_REGISTERS = 8};
  enum numberOfLoops : uint8_t {ENDLESS = 0, LOOP1 = 1, LOOP2 = 2, LOOP3 = 3, LOOP4 = 4, LOOP5 = 5, LOOP6 = 6, LOOP7 = 7};
  enum numberOfFrames : uint8_t {ALL_FRAMES = 0, ONE_FRAME = 1, TWO_FRAMES = 2, THREE_FRAMES = 3, FOUR_FRAMES = 4, FIVE_FRAMES = 5, SIX_FRAMES = 6, SEVEN_FRAMES = 7};

  /* public methods */
  void begin(uint8_t i2cAddr = DEFAULT_I2C_ADDR);
  void end();
  void enableHW();
  void enableSW();
  void disableHW();
  void disableSW();
  void setFrame(frame frameNr);
  void setMode(mode modeNr);
  void setDisplayOptions(bool intensityControl, bool blinkEnable, uint8_t blinkPeriodTime);  // in ms
  void drawPixel(uint8_t x, uint8_t y, uint8_t brigtness);
  //void drawPixel(int16_t x, int16_t y, uint16_t color);
  uint8_t getPixel(uint8_t x, uint8_t y);
  void clear();
  
  // Picture Mode methods:
  void displayFrame(frame frameNr);
  void setFadeOutTime(uint16_t time);  // in ms
  void setFadeInTime(uint16_t time);   // in ms
  void setBreathControl(bool enable, uint8_t extinguishTime);
  void setAudioSync(bool enable);
  
  // Auto Frame Play Mode methods:
  void setStartFrame(frame frameNr);
  void setNumberOfLoops(numberOfLoops loop);
  void setNumberOfFrames(numberOfFrames frame);
  void setDelayBetweenFrames(uint16_t delay);  // in ms
  bool getMovieFinished();
  uint8_t getCurrentFrameDisplay();
  
  // Audio Frame Play Mode methods:
  void setAudioADCRate(uint8_t rate);
  void setAGCControl(bool mode, bool enable, uint8_t gain);

private:
  /* attributes */
  TwoWire& ISSIWire_;
  volatile uint8_t *enPort_;
  uint8_t enPin_;

  /* private constants (static) */
  static const uint8_t DEFAULT_I2C_ADDR         = 0x74;  // AD connected to GND, 0x75 (VCC), 0x76 (SCL), 0x77 (SDA)

  // Command Register:
  static const uint8_t REG_CMD                  = 0xFD;  // Command Register to point at the current frame

  // Frame Registers: (select FRAME1... FRAME8)
  static const uint8_t REG_CONTROL_LED_A_START  = 0x00;  // 0x00, 0x02, 0x04... 0x10
  static const uint8_t REG_CONTROL_LED_A_END    = 0x10;
  static const uint8_t REG_CONTROL_LED_B_START  = 0x01;  // 0x01, 0x03, 0x05... 0x11
  static const uint8_t REG_CONTROL_LED_B_END    = 0x11;
  static const uint8_t REG_CONTROL_BLINK_START  = 0x12;  // 0x12... 0x23
  static const uint8_t REG_CONTROL_BLINK_END    = 0x23;
  static const uint8_t REG_PWM_START            = 0x24;  // 0x24... 0xB3
  static const uint8_t REG_PWM_END              = 0xB3;

  // Function Registers: (before writing or to a Function Register, you have to select "frame nine" -> FUNCTION_REGISTERS)
  static const uint8_t REG_CONF                 = 0x00;
  static const uint8_t REG_PICTURE_DISPLAY      = 0x01;
  static const uint8_t REG_CONTROL_AUTO_PLAY1   = 0x02;
  static const uint8_t REG_CONTROL_AUTO_PLAY2   = 0x03;
  static const uint8_t REG_DISPLAY_OPTION       = 0x05;
  static const uint8_t REG_AUDIO_SYNC           = 0x06;
  static const uint8_t REG_FRAME_STATE          = 0x07;
  static const uint8_t REG_CONTROL_BREATH1      = 0x08;
  static const uint8_t REG_CONTROL_BREATH2      = 0x09;
  static const uint8_t REG_SHUTDOWN             = 0x0A;
  static const uint8_t REG_CONTROL_AGC          = 0x0B;
  static const uint8_t REG_AUDIO_ADC_RATE       = 0x0C;

  // constants
  static const uint8_t TYP_FRAME_DELAY_TIME     = 11;     // in ms
  static const uint8_t TYP_FADE_OUT_TIME        = 26;     // in ms
  static const uint8_t TYP_FADE_IN_TIME         = 26;     // in ms
  static const uint8_t MAX_NUMBER_OF_COLUMNS    = 16;
  static const uint8_t MAX_NUMBER_OF_ROWS       = 9;

  /* private variables */
  uint8_t i2cAddr_;
  frame frame_;

  /* private methods */
  void selectFrame(frame frameNr);
  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
};

#endif

/**
 * @}
 */
