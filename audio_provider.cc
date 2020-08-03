/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include "msp.h"

#include "audio_provider.h"
#include "driverlib.h"
#include <cstdlib>
#include <string.h>
// clang-format on"
#include "micro_features/micro_model_settings.h"

//Private Constants
static const char *TAG = "TF_LITE_AUDIO_PROVIDER";
/* model requires 20ms new data from g_audio_capture_buffer and 10ms old data
 * each time , storing old data in the history buffer , {
 * history_samples_to_keep = 10 * 16 } */
static constexpr int32_t history_samples_to_keep =
    ((kFeatureSliceDurationMs - kFeatureSliceStrideMs) *
     (kAudioSampleFrequency / 1000));
/* new samples to get each time from ringbuffer, { new_samples_to_get =  20 * 16
 * } */
static constexpr int32_t new_samples_to_get =
    (kFeatureSliceStrideMs * (kAudioSampleFrequency / 1000));

//Private Global Variables
namespace {
/**
 * @brief Circular buffer to hold incoming audio data
 */
/* Removed:
 * constexpr int audioCapBuffSliceCount = kFeatureSliceCount;
 * because we need more heap,
 * audioCapBuffSliceCount = 1 is equivalent to 640 Bytes (SRAM)
 */
constexpr int audioCapBuffSliceCount = 33;
int16_t g_audio_capture_buffer[audioCapBuffSliceCount][new_samples_to_get];
volatile int16_t g_audio_capture_circ_write_index = 0;
volatile int16_t g_audio_capture_circ_read_index = 0;
volatile bool g_audio_capture_first_time = true;
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
bool g_is_audio_initialized = false;
int16_t g_history_buffer[history_samples_to_keep];

volatile int16_t g_slices_overridden = 0;
}  // namespace
volatile int32_t g_latest_audio_timestamp = 0;

//Private Function Prototypes
static void incCircBufferIndex(int16_t* index, int16_t max_index);
static void DMA_init(void);
static void ADC_init(void);

//Public Function Definitions
/* Completion interrupt for ADC14 MEM0 */
extern "C" void DMA_INT1_IRQHandler(void)
{
    if(g_audio_capture_first_time)
    {
        g_audio_capture_first_time = false;
    }else{
        incCircBufferIndex((int16_t *)&g_audio_capture_circ_write_index, audioCapBuffSliceCount);
        g_latest_audio_timestamp += ((1000 * new_samples_to_get) /kAudioSampleFrequency);
    }
    DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                               UDMA_MODE_BASIC, (void*) &ADC14->MEM[0],
                               g_audio_capture_buffer[g_audio_capture_circ_write_index], new_samples_to_get);
    DMA_enableChannel(7);
    DMA_Control->ENASET = BIT7;
}

TfLiteStatus InitAudioRecording(tflite::ErrorReporter *error_reporter)
{
    //Initialize DMA first then ADC/Timer
    DMA_init();
    ADC_init();
    /* Wait for DMA to interrupt, signaling a complete conversion and transfer. */
    PCM_gotoLPM0();
    return kTfLiteOk;
}

TfLiteStatus GetAudioSamples(tflite::ErrorReporter *error_reporter,
                             int start_ms, int duration_ms,
                             int *audio_samples_size, int16_t **audio_samples)
{
    // if this is the first time running initialize everything
    if (!g_is_audio_initialized)
    {
        TfLiteStatus init_status = InitAudioRecording(error_reporter);
        if (init_status != kTfLiteOk)
        {
            return init_status;
        }
        g_is_audio_initialized = true;
    }
    /* copy 160 samples (320 bytes) into output_buff from history */
    memcpy((void *)(g_audio_output_buffer), (void *)(g_history_buffer),
         history_samples_to_keep * sizeof(int16_t));

    /* copy 320 samples (640 bytes) from rb at ( int16_t*(g_audio_output_buffer) +
    * 160 ), first 160 samples (320 bytes) will be from history */
    // TODO: Change to DMA?
    memcpy((void *)(g_audio_output_buffer + history_samples_to_keep),(void *)(g_audio_capture_buffer[g_audio_capture_circ_read_index]),
         new_samples_to_get * sizeof(int16_t));
    //Increment read index circularly
    incCircBufferIndex((int16_t *)&g_audio_capture_circ_read_index, audioCapBuffSliceCount);

    /* Zero pad the remaining bytes */
    memset((void *)(g_audio_output_buffer + history_samples_to_keep + new_samples_to_get), 0, kMaxAudioSampleSize - history_samples_to_keep - new_samples_to_get);

    /* copy 160 samples from output_buff into history */
    memcpy((void *)(g_history_buffer),
         (void *)(g_audio_output_buffer + new_samples_to_get),
         history_samples_to_keep * sizeof(int16_t));

    *audio_samples_size = kMaxAudioSampleSize;
    *audio_samples = g_audio_output_buffer;
    return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }

//Private Function Definitions
static void incCircBufferIndex(int16_t* index, int16_t max_index)
{
    if(*index == max_index - 1)
    {
        *index = 0;
    }else{
        (*index)++;
    }
}

static void DMA_init(void)
{
    //DMA Control table to store DMA config
#pragma DATA_ALIGN(1024)
    static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[16];
    /* Configuring DMA module */
    DMA_enableModule();
    DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);
    DMA_assignChannel(DMA_CH7_ADC14);

    DMA_disableChannelAttribute(DMA_CH7_ADC14,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    /*
     * Setting Control Indexes.
     * DMA transfer to ADC14 Memory 0
     *  and the destination to the
     * destination data array.
     */
    DMA_setChannelControl(
        UDMA_PRI_SELECT | DMA_CH7_ADC14,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
        UDMA_DST_INC_16 | UDMA_ARB_1);
    //First time fill history buffer
    DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                           UDMA_MODE_BASIC, (void*) &ADC14->MEM[0],
                           g_history_buffer, history_samples_to_keep);

    /* Assigning/Enabling Interrupts */
    DMA_assignInterrupt(DMA_INT1, 7);
    Interrupt_enableInterrupt(INT_DMA_INT1);
    DMA_clearInterruptFlag(7);
    Interrupt_enableMaster();

    /* Now that the DMA is primed and setup, enabling the channels. The ADC14
     * hardware should take over and transfer/receive all bytes */
    DMA_enableChannel(7);
    // TODO: Add error checking to make sure DMA was properly initialized.
}
static void ADC_init(void)
{
    /* Initializing timer that will control the ADC */
    const Timer_A_PWMConfig pwm =
    {
         TIMER_A_CLOCKSOURCE_SMCLK,
         TIMER_A_CLOCKSOURCE_DIVIDER_1,
         750,
         TIMER_A_CAPTURECOMPARE_REGISTER_1,
         TIMER_A_OUTPUTMODE_SET_RESET,
         750
    };
    Timer_A_generatePWM(TIMER_A0_BASE, &pwm);
    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    bool ADCConfig = ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    /* Setting trigger source to be the TCCA0 */
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Configuring GPIOs (4.3 A10) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3,
                                               GPIO_TERTIARY_MODULE_FUNCTION);
    /* Turning on the microphone (4.1) */
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);

    /* Configuring ADC Memory */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                    ADC_INPUT_A10, false);

    /* Set ADC result format to signed binary */
    ADC14_setResultFormat(ADC_SIGNED_BINARY);
    ADC14_enableConversion();
    /* TODO: Add error logs here */
    // ADC fail
    // Timer fail
    //Start timer and in turn start the ADC
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}
