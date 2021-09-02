/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <MUSCLE_AI_inferencing.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
BLEService inferenceService("2d0a0000-e0f7-5fc8-b50f-05e267afeb67");  
BLEStringCharacteristic inferenceCharacteristic("2d0a0001-e0f7-5fc8-b50f-05e267afeb67", BLERead | BLENotify, 56);
/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("BLE interference demo");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }


    if (!BLE.begin()) {   // initialize BLE
      Serial.println("starting BLE failed!");
      while (1);
    }
     BLE.setLocalName("EI-FITNESS_BAND"); 
     BLE.setConnectionInterval(160, 200);
    BLE.setAdvertisedService(inferenceService); // Advertise service
    inferenceService.addCharacteristic(inferenceCharacteristic); // Add characteristic to service
    BLE.addService(inferenceService); // Add service
    inferenceCharacteristic.writeValue("inference"); // Set first value string
    //Set the advertising interval in units of 0.625 ms
    //Bluetooth LE advertising interval around 50ms
    BLE.setAdvertisingInterval(80);
    BLE.advertise();  // Start advertising


    
}

    


/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}





void sendInferenceOverBLE(String inferenceResult) {
  
inferenceCharacteristic.writeValue(inferenceResult); // Write value
        
}






void loop()
{
    BLEDevice central = BLE.central();
    String inferenceResult = "";
    if (central.discoverAttributes()) {
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
  

    // while the central is connected:
    while (central.connected()) {
        ei_printf("\n ---------------- \n Starting inferencing...\n");   
        ei_printf("Allocating buffer size...");
    
        // Allocate a buffer here for the values we'll read from the IMU
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
    
        ei_printf("Done\n");
    
        ei_printf("Sampling...");
    
        static unsigned long acquisition_start = millis();
        
        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }
    
        static unsigned long acquisition_end = millis();
        static unsigned long acquisition_time = acquisition_end - acquisition_start;
    
        ei_printf("Done in %d ms\n", acquisition_time);
    
        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }
        ei_printf("Done...\n");
    
        ei_printf("Running the classifier... \n");
        ei_impulse_result_t result = { 0 };
    
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }
    
        // print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
            if(result.classification[ix].value >0.7 && inferenceResult != result.classification[ix].label){
              inferenceResult = result.classification[ix].label;
              sendInferenceOverBLE(inferenceResult);
            } 
        }
        
    }
    }
    
    
    } 





    
    
