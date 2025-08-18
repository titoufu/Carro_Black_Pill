#include "motor.hpp"

Motor::Motor(TIM_HandleTypeDef* tim, uint32_t channel,
             GPIO_TypeDef* in1_port, uint16_t in1_pin,
             GPIO_TypeDef* in2_port, uint16_t in2_pin,
             uint32_t pwm_max, bool invert)
: _tim(tim), _channel(channel),
  _in1_port(in1_port), _in1_pin(in1_pin),
  _in2_port(in2_port), _in2_pin(in2_pin),
  _pwm_max(pwm_max), _invert(invert),
  _last_duty(0), _dir(Direction::Forward) {}

void Motor::begin() {
  _last_duty = 0; _dir = Direction::Forward;
  applyPins();
  __HAL_TIM_SET_COMPARE(_tim, _channel, 0);
  HAL_TIM_PWM_Start(_tim, _channel);
}

void Motor::start(Direction dir, uint8_t duty_percent) {
  setDirection(dir);
  setSpeed(duty_percent);
}

void Motor::setDirection(Direction dir) {
  _dir = dir; applyPins();
}

void Motor::setSpeed(uint8_t duty_percent) {
  if (duty_percent > 100) duty_percent = 100;
  _last_duty = duty_percent; applyDuty();
}

void Motor::brake() {
  HAL_GPIO_WritePin(_in1_port, _in1_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(_in2_port, _in2_pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(_tim, _channel, 0);
}

void Motor::coast() {
  HAL_GPIO_WritePin(_in1_port, _in1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(_in2_port, _in2_pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(_tim, _channel, 0);
}

void Motor::applyPins() {
  bool fwd = (_dir == Direction::Forward);
  if (_invert) fwd = !fwd;
  if (fwd) {
    HAL_GPIO_WritePin(_in1_port, _in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_in2_port, _in2_pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(_in1_port, _in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_in2_port, _in2_pin, GPIO_PIN_SET);
  }
}

void Motor::applyDuty() {
  uint32_t ccr = (uint32_t)((_pwm_max + 1U) * _last_duty) / 100U;
  __HAL_TIM_SET_COMPARE(_tim, _channel, ccr);
}
