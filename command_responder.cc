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

#include "command_responder.h"
#include "RGBLeds.c"
// The default implementation writes out the name of the recognized command
// to the error console. Real applications will want to take some custom
// action instead, and should implement their own versions of this function.
void RespondToCommand(tflite::ErrorReporter* error_reporter,
                      int32_t current_time, const char* found_command,
                      uint8_t score, volatile bool is_new_command, uint8_t* scores, int top_index) {
        if (is_new_command){
            edit_RGBLeds(RED_FLAG, OFF_MODE, 0xFFFF);
            edit_RGBLeds(BLUE_FLAG, OFF_MODE, 0xFFFF);
            edit_RGBLeds(GREEN_FLAG, OFF_MODE, 0xFFFF);
            output_RGBLeds();
        if ( top_index == 2 ) {
            edit_RGBLeds(RED_FLAG, ON_MODE, 0xFFFF);
            edit_RGBLeds(BLUE_FLAG, OFF_MODE, 0xFFFF);
            edit_RGBLeds(GREEN_FLAG, OFF_MODE, 0xFFFF);
            output_RGBLeds();

        }
        else if ( top_index == 3 ) {
                    edit_RGBLeds(RED_FLAG, ON_MODE, 0xFFFF);
                    edit_RGBLeds(BLUE_FLAG, ON_MODE, 0xFFFF);
                    edit_RGBLeds(GREEN_FLAG, OFF_MODE, 0xFFFF);
                    output_RGBLeds();

                }
        else if (top_index == 4){
            edit_RGBLeds(BLUE_FLAG, ON_MODE, 0xFFFF);
            edit_RGBLeds(RED_FLAG, OFF_MODE, 0xFFFF);
            edit_RGBLeds(GREEN_FLAG, OFF_MODE, 0xFFFF);
            output_RGBLeds();

      }
        else if ( top_index == 5 ) {
                            edit_RGBLeds(RED_FLAG, OFF_MODE, 0xFFFF);
                            edit_RGBLeds(BLUE_FLAG, OFF_MODE, 0xFFFF);
                            edit_RGBLeds(GREEN_FLAG, ON_MODE, 0xFFFF);
                            output_RGBLeds();

                        }
        }
        TF_LITE_REPORT_ERROR(error_reporter, "Heard %d (%d) @%dms", top_index,
                                     score, current_time);
//}
}
