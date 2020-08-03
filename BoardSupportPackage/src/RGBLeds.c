//Include files
#include "RGBLeds.h"
#include "msp.h"

//Private Defines
#define AUTO_INCRMENT_REG_ADDR 0x10

//Private Enums
typedef enum
{
    INPUT1_ADDR = 0,
    INPUT2_ADDR,
    PSC0_ADDR,
    PWM0_ADDR,
    PSC1_ADDR,
    PWM1_ADDR,
    LS0_ADDR,
    LS1_ADDR,
    LS2_ADDR,
    LS3_ADDR
}LP3943_REG_ADDR;

//Private Structs
typedef struct
{
    /**
     * @brief This contains the array of LS0-3 for the lp3943.
     */
    uint32_t selector_data;
    /**
     * @brief This acts as a bool if any of the selector data was modified
     */
    uint8_t modified;
}UNIT_SELECTOR;

//Private Global Variables
/**
 * @brief 0 initialized unit selectors. Indices go 0,1,2 : r,g,b respectively.
 */
static UNIT_SELECTOR RGB_SELECTORS[3] = {0};
/**
 * @brief These are the slave addressses of the lp3943's. Indices go 0,1,2 : r,g,b respectively.
 */
static uint8_t RGB_SLAVE_ADDR[3] = {0x62, 0x61, 0x60};

//Private Function Prototypes
/**
 * @brief This swaps LED0 with LED15, 1 with 14, ...
 * @param selector_data - This is the selector data for the current rgb unit
 */
static uint32_t swapLedPositions(const uint32_t selector_data);
/**
 * @brief This sends an array of bytes through I2C with a 7-bit slave address
 * @param address -  7-bit address of slave to send data to
 * @param reg_addr - Optional register address to send before data. Send nullptr to not send @reg_addr
 * @param data - array of size @size of data to send
 * @param size - size of @data
 */
static void I2C_sendData(uint8_t address, uint8_t* reg_addr, uint8_t* data, uint8_t size);
/**
 * @brief This breaks a uint32 and converts it into an array little endian style.
 * @param input_data - Input data
 * @param output_data - A byte array initialized to have 4 elements.
 */
static void uint32ToArr(uint32_t input_data, uint8_t output_data[4]);
/**
 * @brief This sends @data to rgb units and pwm_flags.
 * @param rgb_flags - The bitwise ORing of RGB colors
 * @param pwm_flags - The bitwise ORing of PWM units
 * @param pwm_reg_addresses - This is an array of size 2 where the first element is the pwm0 reg and second element is pwm1 reg.
 * @param data - The data to be sent
 */
static void outputPwmData(uint8_t rgb_flags, uint8_t pwm_flags, uint8_t pwm_reg_addresses[2], uint8_t data);
//Private Function Definitions
static uint32_t swapLedPositions(const uint32_t selector_data)
{
    uint32_t outputSelectorData = 0;
    //Left half
    for(uint8_t i = 0; i < 8; i++)
    {
        outputSelectorData |= (selector_data & (0x3 << (i * 2))) << (30 - (4 * i));
    }
    //Right half
    for(uint8_t i = 8; i > 0; i--)
    {
        outputSelectorData |= (selector_data & (0x3 << (((i - 1) + 8) * 2))) >> (30 - (4 * ( 7 - (i - 1))));
    }
    return outputSelectorData;
}

static void I2C_sendData(uint8_t address, uint8_t* reg_addr, uint8_t* data, uint8_t size)
{
    //Reset interrupt flags
    UCB2IFG = 0;
    //Set slave address
    UCB2I2CSA = address;
    //Send start bit
    UCB2CTLW0 |= UCTXSTT;
    //Send optional reg_addr
    if(reg_addr)
    {
        while(!(UCB2IFG & UCTXIFG0));
        UCB2TXBUF = *reg_addr;
    }
    //Send data
    for(uint8_t i = 0; i < size; i++)
    {
        //Wait til tx buffer is empty
        while(!(UCB2IFG & UCTXIFG0));
        UCB2TXBUF = *(data++);
    }

    //Wait til tx buffer is empty
    while(!(UCB2IFG & UCTXIFG0));
    //Send stop bit
    UCB2CTLW0 |= UCTXSTP;
}

static void uint32ToArr(uint32_t input_data, uint8_t output_data[4])
{
    for(uint8_t i = 0; i < 4; i++)
    {
        output_data[i] = (uint8_t)(input_data & 0xFF);
        input_data >>= 8;
    }
}

static void outputPwmData(uint8_t rgb_flags, uint8_t pwm_flags, uint8_t pwm_reg_addresses[2], uint8_t data)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        if(rgb_flags & 1)
        {
            uint8_t tmpPwmFlags = pwm_flags;
            for(uint8_t j = 0; j < 2; j++)
            {
                if(tmpPwmFlags & 1)
                {
                    I2C_sendData(RGB_SLAVE_ADDR[i], &pwm_reg_addresses[j], &data, 1);
                }
                tmpPwmFlags >>= 1;
            }
        }
        rgb_flags >>= 1;
    }
}
//Public Function Definitions
void init_RGBLeds(void)
{
    //Software reset enable
    UCB2CTLW0 = UCSWRST;

    //INitialize I2C master
    // Set as master, I2C mode, Clock sync, SMCLK source, Transmitter, and 7-bit addr
    UCB2CTLW0 |= (UCMST | UCSYNC | UCMODE_3 | UCSSEL_3 | UCTR);

    //Set the Fclk as 400khz.
    //Make sure that Fsmclk is 12Mhz.
    UCB2BRW = 30;

    //This sets the gpio pins into I2C mode sda and scl
    P3SEL0 |= BIT6 | BIT7;
    P3SEL1 &= ~(BIT6|BIT7);

    //Take I2C out of reset
    UCB2CTLW0 &= ~UCSWRST;

    //Turn off all leds
    edit_RGBLeds(RED_FLAG|BLUE_FLAG|GREEN_FLAG, OFF_MODE, 0xFFFF);
    output_RGBLeds();
}

void edit_RGBLeds(uint8_t rgb_flags, RGB_OUTPUT_MODE mode, uint16_t led_bit_mask)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        if(rgb_flags & 1)
        {
            RGB_SELECTORS[i].modified = 1;
            uint8_t bitNum = 0;
            uint16_t tempBitMask = led_bit_mask;
            if(mode == VALUE_MODE)
                RGB_SELECTORS[i].selector_data = 0;
            while(tempBitMask != 0)
            {
                if(mode == VALUE_MODE)
                {
                    RGB_SELECTORS[i].selector_data |= ((uint32_t)(tempBitMask & 1)) << (bitNum * 2);
                }else{
                    if(tempBitMask & 1)
                    {
                        RGB_SELECTORS[i].selector_data = (RGB_SELECTORS[i].selector_data & ~(0b11 << (bitNum * 2))) | (mode << (bitNum * 2));
                    }
                }
                bitNum++;
                tempBitMask >>= 1;
            }
        }
        rgb_flags >>= 1;
    }
}

void output_RGBLeds(void)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        if(RGB_SELECTORS[i].modified)
        {
            uint32_t swappedSelectorData = swapLedPositions(RGB_SELECTORS[i].selector_data);
            uint8_t selectorArr[4];
            uint8_t regAddr = (uint8_t)LS0_ADDR | AUTO_INCRMENT_REG_ADDR;
            uint32ToArr(swappedSelectorData, selectorArr);
            I2C_sendData(RGB_SLAVE_ADDR[i], &regAddr, selectorArr, 4);
            RGB_SELECTORS[i].modified = 0;
        }
    }
}

void pwmFreq_RGBLeds(uint8_t rgb_flags, uint8_t pwm_flags, float frequency)
{
    //Calculate prescaler from frequency
    uint8_t prescaler;
    float prescaler_f = (160.0f/frequency) - 1;
    if((prescaler_f + 0.5) > 255)
    {
        prescaler = 255;
    }else if((prescaler_f + 0.5) < 0)
    {
        prescaler = 0;
    }else{
        prescaler = (uint8_t)(prescaler_f + 0.5);
    }
    //Output Data
    uint8_t pwmRegAddresses[2] = {(uint8_t)PSC0_ADDR, (uint8_t)PSC1_ADDR};
    outputPwmData(rgb_flags, pwm_flags, pwmRegAddresses, prescaler);
}

void pwmDuty_RGBLeds(uint8_t rgb_flags, uint8_t pwm_flags, uint8_t duty_cycle)
{
    //Output Data
    uint8_t pwmRegAddresses[2] = {(uint8_t)PWM0_ADDR, (uint8_t)PWM1_ADDR};
    outputPwmData(rgb_flags, pwm_flags, pwmRegAddresses, duty_cycle);
}
