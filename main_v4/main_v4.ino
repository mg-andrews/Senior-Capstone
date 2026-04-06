// Main
// Author: Miles Wilson
// Published: 03/04/2026

// Uses FreeRTOS to schedule serial input, heart rate, pressure simulation, and stepper motor update tasks.
// Added motor pulse on EVENT_HEARTBEAT, limited pulse to pressure range between systole and diastole
// Added serial input for systole/diastole

// Arduino IDE
// Windows 11
// board: esp32s3

// Fixed pressure mapping to dial face

// BUG: currently beat audio may play twice when cuffPressure enters/exits the systole/diastole range due to queued EVENT_HEARTBEAT in event_queue
// FIX(?): create separate heartbeat_queue with length of 1 to ensure ulTaskNotifyTake can only take one event per beat
// FIXED: task notification sent to audio task from heartbeat callback

// BUG: currently motor defaults to 16 microsteps on powerup of system. 
// This is most likely because the motor and MCU are powered on at different times, and the driver could miss the UART init commands.
// TODO: issue might resolve when single power source is used, but adding driver UART handshake before sending motor init will resolve issue

// TODO: Create UART send queue for spo2, stepper driver handshake(?)

//TODO: add bpm recieve from uart

#include "spo2sim.h"
#include "KorotkoffAudio.h"
#include "pindef.h"
#include "MotorSetup.h"
#include "spo2sim.h"
#include <stdlib.h>
#include <math.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>
#include <stdio.h>

// Cuff simulation param init
float cuffPressure = 0.0f;
float pulseOffset = 0.0f;
float outputPressure = 0.0f;
float tau = 5.0f;     // seconds (default)
float inflationRate = 50.0f;

volatile float pulseAmplitude = 3.0f;   // small bump
const float pulseLength = 0.05f; // 50 ms timer
volatile float pulseTimer = 0.0f;
volatile bool firstCycle = true;

float systole = 120.0f;
float diastole = 80.0f;
float MAP = diastole + (systole - diastole)/3; // Mean arterial pressure

bool au_gap = false;
float au_gap_start = 0.18 * (systole - diastole) + ((esp_random() % 50) / 1000.0); // AU gap start after diastole [mmHg]
float au_gap_length; // length of au gap [mmHg]

// Map [0,360]degrees of motor to [0,320]mmHg of dial face
float max_dial_pressure = 320;

portMUX_TYPE cuffMux = portMUX_INITIALIZER_UNLOCKED;

// Heart rate queue and timer init
TimerHandle_t heartbeatTimer;
TimerHandle_t LEDOffTimer;

int HR_MIN = 40;
int HR_MAX = 200;
int currentHR = 60;

// Motor init
portMUX_TYPE motorMUX = portMUX_INITIALIZER_UNLOCKED;

// Define queues
QueueHandle_t motor_queue;
static const uint8_t motor_queue_len = 1; // Keep 1 so motor always follows latest position update
QueueHandle_t event_queue;
static const uint8_t event_queue_len = 20;

// Task handles
TaskHandle_t StateManagerHandle = NULL;
TaskHandle_t MotorHandle = NULL;
TaskHandle_t HRHandle = NULL;
TaskHandle_t CuffHandle = NULL;
TaskHandle_t SerialHandle = NULL;
TaskHandle_t AudioHandle = NULL;
TaskHandle_t uartHandle = NULL;

// Define system states
typedef enum {
  STATE_SETUP,
  STATE_RUNNING
} SystemState_t;

// Define events
typedef enum {
  EVENT_STARTSIM,
  EVENT_STOPSIM,
  EVENT_HR_UPDATE,
  EVENT_SYSTOLE_UPDATE,
  EVENT_DIASTOLE_UPDATE,
  EVENT_HEARTBEAT,
  EVENT_OS_UPDATE,
  EVENT_KOROTKOFF_UPDATE
} SystemEventType_t;

// Define event object structure
typedef struct {
  SystemEventType_t type;
  float value;
} SystemEvent_t;

volatile SystemState_t currentState = STATE_SETUP;

//--------------------------------------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------------------------------------
// Motor angle to dial pressure
float pressure_to_angle(float pressure) {

  return(pressure*(360/max_dial_pressure));
}

// Heartbeat timer param update
void updateHeartRate() {

  int hrPeriod = (60000 / currentHR);

  xTimerChangePeriod(
    heartbeatTimer,
    pdMS_TO_TICKS(hrPeriod),
    portMAX_DELAY
  );

  Serial.print("HR updated to ");
  Serial.print(currentHR);
  Serial.println(" BPM");
}

// Gaussian Scaling Function, returns scaling factor
float gaussian(float P, float mean_pressure)
{
  float sigma = 15.0f;   // tune this

  float exponent = -((P - mean_pressure) * (P - mean_pressure)) / (2.0f * sigma * sigma);

  return expf(exponent);   // returns 0–1 smooth curve
}

// Linear interpolation, t is varied
float lerp(float start, float end, float t) {
    // Constrain t to be between 0 and 1 to prevent "overshooting"
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return start + t * (end - start);
}

//--------------------------------------------------------------------------------------------------
// Timer callbacks
//--------------------------------------------------------------------------------------------------

// Heartbeat timer callback
void heartbeatCallback(TimerHandle_t xTimer) {
  digitalWrite(HRLED, HIGH);
  xTimerStart(LEDOffTimer, 0);

  SystemEvent_t event = {};
  event.type = EVENT_HEARTBEAT;
  xTaskNotifyGive(AudioHandle);
  xQueueSend(event_queue, &event, portMAX_DELAY);

}

// Turns led off at end of LEDOffTimer
void LEDTimerCallback(TimerHandle_t xTimer) {
  digitalWrite(HRLED, LOW);
}

// -------------------------------------------------------------------
// 2. THE ISR (Interrupt Service Routine)
// This runs in the hardware context the moment a byte hits the RX pin.
// -------------------------------------------------------------------
void IRAM_ATTR onUartReceive() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  // Notify the task to wake up immediately
  vTaskNotifyGiveFromISR(uartHandle, &xHigherPriorityTaskWoken);
  
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

//--------------------------------------------------------------------------------------------------
// Tasks
//--------------------------------------------------------------------------------------------------

// State manager task
void StateManagerTask(void *parameter) {

  SystemEvent_t event;

  for(;;) {

    if(xQueueReceive(event_queue, &event, portMAX_DELAY)) {

      switch(currentState) {
        case STATE_SETUP:

          if(event.type == EVENT_STARTSIM) {

            Serial.println("State swapped: (SETUP --> RUNNING)");

            updateHeartRate();
            xTimerStart(heartbeatTimer, 0);

            firstCycle = true;
            vTaskResume(CuffHandle);
            vTaskResume(MotorHandle);
            vTaskResume(AudioHandle);

            currentState = STATE_RUNNING;

          } else if(event.type == EVENT_HR_UPDATE) {
            
            currentHR = (int)event.value;
            Serial.print("HR set to ");
            Serial.print(currentHR);
            Serial.println(" BPM");

          } else if(event.type == EVENT_SYSTOLE_UPDATE) {

            systole = event.value;
            Serial.print("Systole updated to: ");
            Serial.println(systole);

          } else if(event.type == EVENT_DIASTOLE_UPDATE) {

            diastole = event.value;
            Serial.print("Diastole updated to: ");
            Serial.println(diastole);

          } else if(event.type == EVENT_KOROTKOFF_UPDATE) {

            au_gap_length = event.value;
            Serial.print("Auscultatory gap set to: ");
            Serial.println(au_gap_length);
          }

          break;

        case STATE_RUNNING:

          if(event.type == EVENT_STOPSIM) {

            Serial.println("State swapped: (RUNNING --> SETUP)");

            xTimerStop(heartbeatTimer, 0);
            vTaskSuspend(CuffHandle);
            vTaskSuspend(AudioHandle);

            // Sets sim pressures to 0 while blocking any outside updates
            portENTER_CRITICAL(&cuffMux);
            cuffPressure = 0;
            pulseOffset = 0;
            outputPressure = 0;
            portEXIT_CRITICAL(&cuffMux);

            // Zero motor on simulation stop
            float zero = 0.0f;
            xQueueOverwrite(motor_queue, &zero);
            xTaskNotify(MotorHandle, 0, eNoAction);

            // 3-second timeout before motor zeroing is cancelled
            TickType_t startTime = xTaskGetTickCount();
            while (motor_get_position() != 0) {
              if (xTaskGetTickCount() - startTime > pdMS_TO_TICKS(3000)) {
                Serial.println("Warning: Motor zero timeout");
                break;
              }
              vTaskDelay(pdMS_TO_TICKS(10));
            }

            Serial.println("Motor zeroed");
            vTaskSuspend(MotorHandle);

            currentState = STATE_SETUP;

          } else if(event.type == EVENT_HEARTBEAT) {

              // Logic for korotkoff sounds and motor pulse on heartbeat
              xTaskNotifyGive(CuffHandle);
              //xTaskNotifyGive(AudioHandle);
              //Serial.println("beat send");

          } else if(event.type == EVENT_HR_UPDATE) {

              currentHR = (int)event.value;
              updateHeartRate();
          }

          break;

      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Wait for serial parameter input and put in queue
void SerialTask(void *parameter) {

  for(;;) {

    if(Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      SystemEvent_t event ={};

      if(input == "start") {

        event.type = EVENT_STARTSIM;
        xQueueSend(event_queue, &event, portMAX_DELAY);

      } else if(input == "stop") {

        event.type = EVENT_STOPSIM;
        xQueueSend(event_queue, &event, portMAX_DELAY);

      } else if(input.startsWith("hr ")) {

        float hrInput = input.substring(3).toFloat();
        if(hrInput >= HR_MIN && hrInput <= HR_MAX) {
          event.type = EVENT_HR_UPDATE;
          event.value = hrInput;
          xQueueSend(event_queue, &event, portMAX_DELAY);

        } else {

          Serial.print("HR input out of range. HR must be between ");
          Serial.print(HR_MIN);
          Serial.print(" and ");
          Serial.print(HR_MAX);
          Serial.println(" BPM");
        }
        
      } else if(input.startsWith("sys ")) {   // Added systole input command

        systole = input.substring(4).toFloat();
        /*
        event.type = EVENT_SYSTOLE_UPDATE;
        event.value = systole;
        xQueueSend(event_queue, &event, portMAX_DELAY);
        */

        Serial.print("Systole set to ");
        Serial.print(systole);
        Serial.println(" mmHg");

      } else if(input.startsWith("dia ")) {   // Added diastole input command

        diastole = input.substring(4).toFloat();
        /*
        event.type = EVENT_DIASTOLE_UPDATE;
        event.value = diastole;
        xQueueSend(event_queue, &event, portMAX_DELAY);
        */

        Serial.print("Diastole set to ");
        Serial.print(diastole);
        Serial.println(" mmHg");

      }
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Check for input every 50 ms
  }
}

//--------------------------------------------------------------------------------------------------
// Task: receive scenario info through UART
void uartTask(void *pvParameters) {

  static const int param_num = 10;
  
  String rxBuffer = "";
  SystemEvent_t event ={};

  SystemEvent_t start_event = {};
  start_event.type = EVENT_STARTSIM;

  for (;;) {
    // Sleep until the hardware interrupts and notifies this task
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    static float newHR;

    while (Serial1.available() > 0) {
      char c = Serial1.read();

      if (c == '\n') {
        rxBuffer.trim();

        // Stop sim and skip rest of parsing logic if serial signal is "STOP"
        // --- 1. HANDLE THE "STOP" CASE ---
        if (rxBuffer.equalsIgnoreCase("STOP")) {
          SystemEvent_t stop_event = {};
          stop_event.type = EVENT_STOPSIM; // Ensure this is defined in your enum
          xQueueSend(event_queue, &stop_event, portMAX_DELAY);
          
          Serial.println("Stopping simulation...");
          rxBuffer = ""; // Reset buffer
          continue;      // Skip the CSV parsing logic below
        }
        
        // 1. Define your 7 local variables
        String localData[param_num]; 
        
        int startIdx = 0;
        int commaIdx = 0;

        // 2. Parse the CSV into the local array
        for (int i = 0; i < param_num; i++) {
          commaIdx = rxBuffer.indexOf(',', startIdx);
          
          if (commaIdx != -1) {
            localData[i] = rxBuffer.substring(startIdx, commaIdx);
            startIdx = commaIdx + 1;
          } else {
            // The last piece of data after the final comma
            localData[i] = rxBuffer.substring(startIdx);
          }
        }

        // Scenario update
        Serial.println("\nUART recieved (Scenario, Oxygen, Systole, Diastole, BP str, Fitz, Temp, Korotkoff, AU Gap):");
        Serial.print("              (");
        /*
        event.type = EVENT_SCENARIO_UPDATE;
        event.value = localData[0];
        xQueueSend(event_queue, &event, portMAX_DELAY);
        */
        Serial.print(localData[0]);
        Serial.print(", ");

        // Oxygen update
        //event.type = EVENT_OXYGEN_UPDATE;
        //event.value = localData[1];
        //xQueueSend(event_queue, &event, portMAX_DELAY);
        Serial.print(localData[1]);
        Serial.print(", ");

        // Systole update
        event.type = EVENT_SYSTOLE_UPDATE;
        event.value = localData[2].toFloat();
        xQueueSend(event_queue, &event, portMAX_DELAY);
        Serial.print(localData[2]);
        Serial.print(", ");

        // Diastole update
        event.type = EVENT_DIASTOLE_UPDATE;
        event.value = localData[3].toFloat();
        xQueueSend(event_queue, &event, portMAX_DELAY);
        Serial.print(localData[3]);
        Serial.print(", ");

        // HR update
        /*
        newHR = localData[4].toFloat();
        if (newHR <= HR_MIN && newHR >= HR_MAX) {
          event.type = EVENT_HR_UPDATE;
          xQueueSend(event_queue, &event, portMAX_DELAY);
        }
        */
        Serial.print(localData[4]);
        Serial.print(", ");

        // Fitz update
        //event.type = EVENT_FITZ_UPDATE;
        //event.value = localData[5];
        //xQueueSend(event_queue, &event, portMAX_DELAY);
        Serial.print(localData[5]);
        Serial.print(", ");

        // Temperature update
        //event.type = EVENT_TEMPERATURE_UPDATE;
        //event.value = localData[6];
        //xQueueSend(event_queue, &event, portMAX_DELAY); 
        Serial.print(localData[6]);
        Serial.print(", ");

        // Korotkoff
        event.type = EVENT_KOROTKOFF_UPDATE;
        if (localData[7] == "Auscultatory Gap") {
          au_gap = true;
          event.value = localData[8].toFloat();
          xQueueSend(event_queue, &event, portMAX_DELAY);
        } else {
          au_gap = false;
        }
        Serial.print(localData[7]);
        Serial.print(", ");
        Serial.print(localData[8]);
        Serial.println(", ");

        // Temperature update
        event.type = EVENT_HR_UPDATE;
        event.value = localData[9].toFloat();
        xQueueSend(event_queue, &event, portMAX_DELAY); 
        Serial.print(localData[9]);
        Serial.println(")\n");

        rxBuffer = ""; // Clear for the next incoming line

        xQueueSend(event_queue, &start_event, portMAX_DELAY);

      } else if (c != '\r') {
        rxBuffer += c;
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Play Korotkoff audio

// gap start/end locations are only calculated on first task call and task is constantly running, does NOT update when systole/diastole does
// Define in for(;;)
void AudioTask(void *parameter) {
  const float base_gain = 1.4f;
  const float fade_length = 1.5f; 

  for(;;) {
    // 1. Recalculate these EVERY loop to track real-time changes to systole/au_gap variables
    float gap_start_pressure = systole - au_gap_start;
    float gap_end_pressure = gap_start_pressure - au_gap_length;

    // 2. Only "take" the notification if one is waiting
    if(ulTaskNotifyTake(pdTRUE, 0)) {
      if(cuffPressure < systole && cuffPressure > diastole) {
        
        float target_gain = base_gain; 

        if(au_gap) {
          // Fade Out
          if(cuffPressure <= (gap_start_pressure + fade_length) && cuffPressure > gap_start_pressure) {
            float t = (cuffPressure - gap_start_pressure) / fade_length; 
            target_gain = lerp(0.0f, base_gain, t);
          } 
          // Silence in Gap
          else if(cuffPressure <= gap_start_pressure && cuffPressure >= gap_end_pressure) {
            target_gain = 0.2f; 
          } 
          // Fade In
          else if(cuffPressure < gap_end_pressure && cuffPressure >= (gap_end_pressure - fade_length)) {
            float t = (gap_end_pressure - cuffPressure) / fade_length;
            target_gain = lerp(0.0f, base_gain, t);
          }
        }

        set_gain_to(target_gain);

        // 3. Only trigger the beat if we are outside the silent part of the gap
        if(target_gain > 0.05f) {
            start_beat();
        }
      }
    }

    // 4. This continues to call every loop iteration as requested
    audio_process();
  }
}
/*
void AudioTask(void *parameter) {

  const float base_gain = 1.8f;
  const float fade_length = 1.5f; // Fade over fade_length mmHg
  float gap_start_pressure = systole - au_gap_start;
  Serial.println(gap_start_pressure);
  float gap_end_pressure = systole - au_gap_start - au_gap_length;
  Serial.println(gap_end_pressure);

  for(;;) {
    
    if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      if(cuffPressure < systole && cuffPressure > diastole) {

        // Check for au gap and decrease gain if in gap range
        if(au_gap) {
          if(cuffPressure < (gap_start_pressure + fade_length) && cuffPressure > (gap_start_pressure)) {
            float t = (cuffPressure - gap_start_pressure) / fade_length; 
            // Note: Since current_p is decreasing, lerp from 0 volume up to base_gain
            set_gain_to(lerp(0.0f, base_gain, t));
          } else if(cuffPressure < gap_start_pressure && cuffPressure > gap_end_pressure) {
            set_gain_to(0.1);
          } else if(cuffPressure < gap_end_pressure && cuffPressure > (gap_end_pressure - fade_length)) {
            float t = (gap_end_pressure - cuffPressure) / fade_length;
            set_gain_to(lerp(0.0f, base_gain, t));
          } else {
            set_gain_to(base_gain);
          }
        }
        start_beat();
      }
    }
    audio_process();
  }
}
*/

//--------------------------------------------------------------------------------------------------
// Task: Update cuff pressure model
void CuffTask(void *parameter) {
  const float dt = 0.005f;   // 5 ms --> 200 Hz update
  TickType_t lastWakeTime = xTaskGetTickCount();
  firstCycle = true;
  float pressure;

  for(;;) {
    if(firstCycle) {
      lastWakeTime = xTaskGetTickCount();
      firstCycle = false;
    }

    // Read potentiometer
    int potValue = analogRead(POT_PIN);

    // Map to tau range (1s to 40s)
    tau = 1.0f + (potValue / 4095.0f) * 39.0f;

    /* FOR BUTTON INPUT
    // Read button to input pressure in system
    if(!digitalRead(BUTTON_PIN)) {
      cuffPressure += inflationRate * dt;   // Cuff inflation rate
      if (cuffPressure > 360.0f)
        cuffPressure = 360.0f;
    }
    */

    // Read flex sensor bend to input pressure in system
    if(digitalRead(BUTTON_PIN)) {
      cuffPressure += inflationRate * dt;   // Cuff inflation rate
      if (cuffPressure > max_dial_pressure)
        cuffPressure = max_dial_pressure;
    }


    // Exponential decay model
    cuffPressure *= expf(-dt / tau);    

    // Start pulse timer if EVENT_HEARTBEAT occurs
    if(ulTaskNotifyTake(pdTRUE, 0) > 0) {
      if((cuffPressure < systole + 5) && (cuffPressure > diastole - 5)) {

        pulseTimer = pulseLength;
      }
    }

    // Calculate pulseOffset if pulse timer is active and decrement pulseTimer until 0
    if(pulseTimer > 0.0f) {

      pulseOffset = gaussian(cuffPressure, MAP);
      pulseTimer -= dt;

    } else if(pulseTimer <= 0.0f) {

      pulseTimer = 0.0f;
      pulseOffset = 0.0f;

    }

    outputPressure = cuffPressure + (pulseAmplitude * pulseOffset);

    pressure = pressure_to_angle(outputPressure);

    // Clamp small values
    if(pressure < 0.1f)
      pressure = 0.0f;

    xQueueOverwrite(motor_queue, &pressure);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(dt * 1000));
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Update stepper position based on pressure in motor_queue
void MotorTask(void *parameter) {
  float pressure;
  int lastPosition = 0;

  for(;;) {

    if(xTaskNotifyWait(0, 0, NULL, 0) == pdTRUE) {
      
      motor_move_to(0);
      lastPosition = 0;
    }

    if (xQueueReceive(motor_queue, &pressure, portMAX_DELAY) == pdTRUE) {
      // Convert pressure to motor position
      int targetPosition = (int)((pressure / 360.0f) * motor_steps_per_rev());

      if (abs(targetPosition - lastPosition) > motor_deadband()) {
        motor_move_to(targetPosition);
        lastPosition = targetPosition;
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Calibrate motor

void calibrateMotorTask(void *parameter) {
   // init stallguard
   // Set motor speed to lower constant speed

  for(;;) {

    // Set direction CCW
    // Call motor_move()
    // Check current draw
    // IF current spikes:
    //  Stop moving motor
    //  Set position to 0
    // Set direction CW
    // Call motor_move()
    // IF current spikes:
    //  Check current step
    //  Set position to max motor angle
    // Return to zero position
  }
}

//--------------------------------------------------------------------------------------------------
// Task: calculate and pass oxygen saturation to UART task
/*
void oxsatTask(void *parameter) {

  SpO2Result reading;
  SystemEventType_t event = {};

  for(;;) {

    spo2_reading = calculate_spo2();

    event.type = EVENT_OS_UPDATE;
    event.value = spo2_reading;
    xQueueSent(event_queue, &event, portMAX_DELAY);
  }
}
*/

//--------------------------------------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------------------------------------
void setup() {
  
  Serial.begin(115200);
  delay(500 / portTICK_PERIOD_MS);

  // Cuff pressure setup
  pinMode(V_PLUS, OUTPUT);
  pinMode(VIO_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(HRLED, OUTPUT);

  digitalWrite(V_PLUS, HIGH); // Sets input of pot to 3.3V
  digitalWrite(VIO_PIN, HIGH); // Sets input to VIO on driver to 3.3V

  // Stepper driver setup
  setup_motor();

  // Audio setup
  setup_i2s();

  // Init instruction messages
  Serial.println("       ...");
  Serial.println("Setup initialized");
  Serial.println("       ...");
  Serial.println("Input heart rate by typing 'hr [BPM]'");
  Serial.println("Input systole by typing 'sys [mmHg]'");
  Serial.println("Input diastole by typing 'dia [mmHg]'\n");
  Serial.println("Start simulation by typing 'start'");
  Serial.println("Suspend simulation by typing 'stop'\n");

  Serial1.begin(115200, SERIAL_8N1, UART_RXD, UART_TXD);
  delay(500 / portTICK_PERIOD_MS);
  Serial1.onReceive(onUartReceive);

  /*
  Serial.print("Initial heart rate set to ");
  Serial.print(currentHR);
  Serial.println(" BPM");
  Serial.print("Initial systole set to ");
  Serial.print(systole);
  Serial.println(" mmHg");
  Serial.print("Initial diastole set to ");
  Serial.print(diastole);
  Serial.println(" mmHg");
  */

  // Create queues
  motor_queue = xQueueCreate(motor_queue_len, sizeof(float));
  event_queue = xQueueCreate(event_queue_len, sizeof(SystemEvent_t));

  // Create software timers
  heartbeatTimer = xTimerCreate(
    "HeartbeatTimer",
    pdMS_TO_TICKS(500),
    pdTRUE, // Retriggers on end
    NULL,
    heartbeatCallback
  );

  LEDOffTimer = xTimerCreate(
    "LEDOffTimer",
    pdMS_TO_TICKS(100),
    pdFALSE, // One-shot
    NULL,
    LEDTimerCallback
  );

  // Create task instances
  xTaskCreatePinnedToCore(
    StateManagerTask,     // Task function
    "StateManagerTask",   // Task name
    4096,           // Stack size in bytes
    NULL,           // Parameters
    2,              // Priority
    &StateManagerHandle,  // Task handle
    1   
  );

  xTaskCreatePinnedToCore(
    SerialTask,     // Task function
    "SerialTask",   // Task name
    4096,           // Stack size in bytes
    NULL,           // Parameters
    1,              // Priority
    &SerialHandle,  // Task handle
    1               // Run on core 1
  );

  xTaskCreatePinnedToCore(
    uartTask,     // Task function
    "uartTask",   // Task name
    4096,           // Stack size in bytes
    NULL,           // Parameters
    3,              // Priority
    &uartHandle,  // Task handle
    1               // Run on core 1
  );

  xTaskCreatePinnedToCore(
    AudioTask,     // Task function
    "AudioTask",   // Task name
    8192,         // Stack size
    NULL,         // Parameters
    1,            // Priority
    &AudioHandle,  // Task handle
    0             // Run on core 1
  );

  vTaskSuspend(AudioHandle);

  xTaskCreatePinnedToCore(
    CuffTask,     // Task function
    "CuffTask",   // Task name
    2048,         // Stack size
    NULL,         // Parameters
    4,            // Priority
    &CuffHandle,  // Task handle
    1             // Run on core 1
  );

  vTaskSuspend(CuffHandle);

  xTaskCreatePinnedToCore(
    MotorTask,        // Task function
    "MotorTask",      // Task name
    4096,             // Stack size in bytes
    NULL,             // Parameters
    4,                // Priority
    &MotorHandle,     // Task handle
    1                 // Run on core 1
  );

  vTaskSuspend(MotorHandle);
}

void loop() {
}