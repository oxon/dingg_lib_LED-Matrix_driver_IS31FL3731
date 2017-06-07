#ifndef _IS31FL3731_H_
#define _IS31Fl3731_H_
/*******************************************************************************
* \file    IS31FL3731.h
********************************************************************************
* \author  Jascha Haldemann jh@oxon.ch
* \date    01.01.2017
* \version 1.0
*
* \brief   The IS31FL3731 can drive a 16x9 (144) LED-Matrix.
*
* \section DESCRIPTION
* The IS31FL3731 is able to control the brightness of every LED individually by
* setting a 8-bit PWM duty cycle.
* It consists of 8 frames that you can prepare beforehand and display
* with either the displayFrame()-function (Picture Mode) or in a loop
* (Auto Frame Play Mode).
*
* \note:   The IS31FL3731 is very critical when it comes to high temperatures.
*          Do not exceed 260°C (for > 30s) in the production process.
*
********************************************************************************
* LED-Matrix driver Library
*******************************************************************************/

/* ============================== Global imports ============================ */
#include <Arduino.h>
#include <Wire.h>
#include <LEDMatrix.h>

/* ==================== Global module constant declaration ================== */

/* ========================= Global macro declaration ======================= */
#ifndef _swap_int16_t
  #define _swap_int16_t(a, b) {int16_t temp = a; a = b; b = temp;}
#endif

/* ============================ Class declaration =========================== */
class IS31FL3731 : public LEDMatrix
{
public:
  /* Public member typedefs */
  typedef enum : uint8_t
  {
    PICTURE_MODE = 0,
    AUTO_FRAME_PLAY_MODE = 1,
    AUDIO_FRAME_PLAY_MODE = 2
  } mode_t;

  typedef enum : uint8_t
  {
    FRAME1 = 0x00,
    FRAME2 = 0x01,
    FRAME3 = 0x02,
    FRAME4 = 0x03,
    FRAME5 = 0x04,
    FRAME6 = 0x05,
    FRAME7 = 0x06,
    FRAME8 = 0x07,
    FUNCTION_REGISTERS = 0x0B
  } frame_t;

  typedef enum : uint8_t
  {
    ENDLESS = 0,
    LOOP1 = 1,
    LOOP2 = 2,
    LOOP3 = 3,
    LOOP4 = 4,
    LOOP5 = 5,
    LOOP6 = 6,
    LOOP7 = 7
  } numberOfLoops_t;

  typedef enum : uint8_t
  {
    ALL_FRAMES = 0,
    ONE_FRAME = 1,
    TWO_FRAMES = 2,
    THREE_FRAMES = 3,
    FOUR_FRAMES = 4,
    FIVE_FRAMES = 5,
    SIX_FRAMES = 6,
    SEVEN_FRAMES = 7
  } numberOfFrames_t;

  /* Constructor(s) and  Destructor */
  IS31FL3731(TwoWire& ISSIWire, volatile uint8_t *enPort, uint8_t enPin, uint8_t x = 9, uint8_t y = 16) :
    LEDMatrix(x, y), ISSIWire_(ISSIWire), enPort_(enPort), enPin_(enPin) {};
  ~IS31FL3731() {};

  /* Public member functions */
  void begin(const uint32_t clockSpeed = 200000L, uint8_t i2cAddr = DEFAULT_I2C_ADDR);
  void end();
  void enableHW();
  void enableSW();
  void disableHW();
  void disableSW();
  void setFrame(frame_t frameNr);
  void setMode(mode_t modeNr);
  void setDisplayOptions(bool intensityControl, bool blinkEnable, uint8_t blinkPeriodTime);  // in ms
  void drawPixel(uint8_t x, uint8_t y, uint8_t brigtness);
  uint8_t getPixel(uint8_t x, uint8_t y);
  void clear();
  void fillScreen(uint8_t brightness);

  // Picture Mode functions:
  void displayFrame(frame_t frameNr);
  void setFadeOutTime(uint16_t time);  // in ms
  void setFadeInTime(uint16_t time);   // in ms
  void setBreathControl(bool enable, uint8_t extinguishTime);
  void setAudioSync(bool enable);

  // Auto Frame Play Mode functions:
  void setStartFrame(frame_t frameNr);
  void setNumberOfLoops(numberOfLoops_t loop);
  void setNumberOfFrames(numberOfFrames_t frame);
  void setDelayBetweenFrames(uint16_t delay);  // in ms
  bool getMovieFinished();
  frame_t getCurrentFrameDisplay();

  // Audio Frame Play Mode functions:
  void setAudioADCRate(uint8_t rate);
  void setAGCControl(bool mode, bool enable, uint8_t gain);

private:
  /* Private constant declerations (static) */
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

  // Function Registers: (before writing to a Function Register, you have to select "frame nine" -> FUNCTION_REGISTERS: 0x0B = 11)
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

  /* Private member data */
  TwoWire& ISSIWire_;
  volatile uint8_t *enPort_;
  uint8_t enPin_;
  uint8_t i2cAddr_;
  frame_t frame_;

  /* Private member functions */
  void selectFrame(frame_t frameNr);
  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
};

#endif
