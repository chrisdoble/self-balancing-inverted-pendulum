/**
 * The A channel pin from the motor's rotary encoder.
 *
 * Needs to be on pin 2 so we can attach an interrupt.
 */
const uint8_t motorAngleAPin = 2;

/**
 * The A channel pin from the pendulum's rotary encoder.
 *
 * Needs to be on pin 3 so we can attach an interrupt.
 */
const uint8_t pendulumAngleAPin = 3;

/**
 * The B channel pin from the motor's rotary encoder.
 *
 * It's a quarter period shifted from the A channel pin from which we can determine the direction of rotation.
 */
const uint8_t motorAngleBPin = 4;

/** The B channel pin from the pendulum's rotary encoder. */
const uint8_t pendulumAngleBPin = 5;

/**
 * The output pin from the limit switch.
 *
 * If this is high something has pressed the limit switch.
 */
const uint8_t limitSwitchPin = 6;

/**
 * The motor's direction pin.
 *
 * LOW is towards the motor, HIGH is away from it.
 */
const uint8_t motorSpeedDirPin = 8;

/**
 * The motor's speed pin.
 *
 * Accepts a PWM signal that controls the motor's speed.
 */
const uint8_t motorSpeedPwmPin = 9;