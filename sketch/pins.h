/**
 * The A_LSB_U pin from the pendulum's rotary encoder.
 *
 * Needs to be on pin 2 so we can attach an interrupt.
 */
const uint8_t pendulumAngleAPin = 2;

/**
 * The A_LSB_U pin from the motor's rotary encoder.
 *
 * Needs to be on pin 3 so we can attach an interrupt.
 */
const uint8_t motorAngleAPin = 3;

/**
 * The B_Dir_V pin from the pendulum's rotary encoder.
 *
 * It's a quarter period shifted from the A_LSB_U pin from which we can
 * determine the direction of rotation.
 */
const uint8_t pendulumAngleBPin = 4;

/**
 * The MagINCn pin from the pendulum's rotary encoder.
 *
 * If this is low the magnet has moved closer to the rotary encoder. If both
 * this and the MagDECn pin are low the magnet isn't in range.
 */
const uint8_t pendulumAngleMagIncPin = 5;

/**
 * The MagDECn pin from the pendulum's rotary encoder.
 *
 * If this is low the magnet has moved further from the rotary encoder. If both
 * this and the MagINCn pin are low the magnet isn't in range.
 */
const uint8_t pendulumAngleMagDecPin = 6;

/** The B_Dir_V pin from the motor's rotary encoder. */
const uint8_t motorAngleBPin = 7;

/** The MagINCn pin from the motor's rotary encoder. */
const uint8_t motorAngleMagIncPin = 8;

/** The MagDECn pin from the motor's rotary encoder. */
const uint8_t motorAngleMagDecPin = 9;

/**
 * The output pin from the limit switch.
 *
 * If this is high something has pressed the limit switch.
 */
const uint8_t limitSwitchPin = 10;

/**
 * The motor's speed pin.
 *
 * Accepts a PWM signal that controls the motor's speed.
 */
const uint8_t motorSpeedPwmPin = 11;

/**
 * The motor's direction pin.
 *
 * LOW is towards the motor, HIGH is away from it.
 */
const uint8_t motorSpeedDirPin = 12;