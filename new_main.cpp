
#include "msp.h"
#include "driverlib.h"
extern "C"
{
#include "BSP.h"
}

#include "main_functions.h"
#include "audio_provider.h"
#include "command_responder.h"
#include "feature_provider.h"
#include "micro_features/micro_model_settings.h"
#include "micro_features/tiny_conv_micro_features_model_data.h"
#include "recognize_commands.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
/************************************************************
 * Global Variables
 ************************************************************/
// pointer to error reporter
tflite::ErrorReporter* error_reporter = nullptr;

void main(void)
{
    // initialize board
    /*
     * CLOCK STATES
     * Sets MCLK to 48 MHz
     * Sets HSMCLK to 24 MHz
     * Sets SMCLK to 12 MHz
     */
    BSP_InitBoard();
    setup();
    while (true)
    {
        loop();
    }
}
