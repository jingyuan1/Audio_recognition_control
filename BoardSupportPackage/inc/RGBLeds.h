#ifndef RGBLEDS_H_
#define RGBLEDS_H_

//Include files
#include <stdint.h>

//Public enums
/**
 * @brief @VALUE_MODE Simply reflects the bit pattern to the leds where 1 means on and 0 means off
 */
typedef enum
{
    OFF_MODE = 0b00,
    ON_MODE = 0b01,
    PWM0_MODE = 0b10,
    PWM1_MODE = 0b11,
    VALUE_MODE
}RGB_OUTPUT_MODE;

//Public defines
#define RED_FLAG 0b1
#define GREEN_FLAG 0b10
#define BLUE_FLAG 0b100
/**
 * @brief These are flags for the pwm functions
 */
#define PWM0_FLAG 0b1
#define PWM1_FLAG 0b10

//Public Function Prototypes
/**
 * @brief This initializes the rgb leds to the default off state as well as the i2c to control the leds
 * @important This must be called before any other of the below functions
 */
void init_RGBLeds(void);

/**
 * @brief This function buffers a new output to the leds which can be outputted with the function @output_RGBLeds
 * @param rgb_flags - specifies a possible bitwise ORing of RGB_FLAGs for which to change
 * @param mode - Specifies which output mode to change the leds specified by @led_bit_mask to
 * @param led_bit_mask - There are 16 leds, one for each bit, any leds with bit set to 1 will have @mode applied to them
 */
void edit_RGBLeds(uint8_t rgb_flags, RGB_OUTPUT_MODE mode, uint16_t led_bit_mask);

/**
 * @brief This function outputs the buffered data to the RGB leds
 */
void output_RGBLeds(void);

/**
 * @brief This edits the pwm frequency of pwm0 and/or pwm1
 * @param rgb_flags - specifies a possible bitwise ORing of RGB_FLAGs for which to change
 * @param pwm_flags - This is where pwm flags can be bitwise OR'd to specify editing more than one pwm channel
 * @param frequency - The desired frequency in Hz. Ranges from 160 Hz to 0.625 Hz
 */
void pwmFreq_RGBLeds(uint8_t rgb_flags, uint8_t pwm_flags, float frequency);

/**
 * @brief This edits the pwm duty cycle of pwm0 and/or pwm1
 * @param rgb_flags - specifies a possible bitwise ORing of RGB_FLAGs for which to change
 * @param pwm_flags - This is where pwm flags can be bitwise OR'd to specify editing more than one pwm channel
 * @param duty_cycle - The desired duty cycle from 0 to 255
 */
void pwmDuty_RGBLeds(uint8_t rgb_flags, uint8_t pwm_flags, uint8_t duty_cycle);

#endif /* RGBLEDS_H_ */
