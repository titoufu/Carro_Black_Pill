#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
}
#endif

class Motor {
public:
  enum class Direction { Forward, Backward };

  // 8 parâmetros: timer, canal, IN1, IN2, PWM_MAX, invert
  Motor(TIM_HandleTypeDef* tim,
        uint32_t channel,
        GPIO_TypeDef* in1_port, uint16_t in1_pin,
        GPIO_TypeDef* in2_port, uint16_t in2_pin,
        uint32_t pwm_max,
        bool invert = false);

  void begin();                                      // inicia PWM (CCR=0)
  void start(Direction dir, uint8_t duty_percent);   // define dir + duty
  void setDirection(Direction dir);
  void setSpeed(uint8_t duty_percent);               // 0..100
  void brake();                                      // freio (IN1=1, IN2=1)
  void coast();                                      // livre (IN1=0, IN2=0)

private:
  TIM_HandleTypeDef* _tim;
  uint32_t _channel;
  GPIO_TypeDef* _in1_port; uint16_t _in1_pin;
  GPIO_TypeDef* _in2_port; uint16_t _in2_pin;
  uint32_t _pwm_max;
  bool _invert;
  uint8_t _last_duty;
  Direction _dir;

  void applyPins();
  void applyDuty();
};
