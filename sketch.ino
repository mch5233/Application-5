/* --------------------------------------------------------------
   Application 5
   Release Type: Use of Memory Based Task Communication
   Class: Real Time Systems - Su 2025
   Theme: Healthcare Systems - Heart Rate Monitoring with Emergency Response
   Author: Mya Camacho-Hill
   Email: my062925@ucf.edu
   Company: [University of Central Florida]
   Website: theDRACOlab.com
   AI Use: Structure and synchronization patterns adapted from provided example
   and troubleshooting purposes
---------------------------------------------------------------*/

#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// ----- Pin Definitions (Healthcare System) -----
const int HEARTBEAT_LED   = 5;   // Green LED - system status
const int ALERT_LED       = 4;   // Red LED - heart rate alerts
const int EMERGENCY_BTN   = 18;  // Emergency button for nurses
const int HR_SENSOR_PIN   = 34;  // Analog heart rate sensor (ADC)

// ----- Healthcare Parameters -----
const int MAX_HR_EVENTS      = 30;   // Max queued heart rate alerts
const int HR_LOW_THRESHOLD   = 40;   // Low heart rate threshold (bradycardia) - 40 BPM
const int HR_HIGH_THRESHOLD  = 100;  // High heart rate threshold (tachycardia) - 100 BPM
const int DEBOUNCE_MS        = 50;   // Button debounce time
const int HR_SAMPLE_RATE_MS  = 17;   // Heart rate sampling every 17ms

// ----- WiFi Configuration -----
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PWD  = "";

// ----- Web Server -----
WebServer medicalConsole(80);

// ----- FreeRTOS Synchronization Primitives -----
SemaphoreHandle_t hr_alert_semaphore;     // Counting semaphore for HR alerts
SemaphoreHandle_t emergency_semaphore;    // Binary semaphore for emergency button
SemaphoreHandle_t serial_mutex;           // Mutex for serial output protection
QueueHandle_t     hr_data_queue;          // Queue for heart rate data logging

// ----- Global State Variables -----
volatile bool monitoring_mode     = true;   // true = NORMAL, false = EMERGENCY
volatile bool alert_active        = false;  // Current alert status
volatile int  current_hr_reading  = 0;      // Latest heart rate value
volatile int  alert_count         = 0;      // Number of alerts triggered
volatile bool system_running      = true;   // System status indicator

// ----- Task Function Declarations -----
void wifiServerTask(void* parameters);
void heartRateMonitorTask(void* parameters);
void emergencyButtonTask(void* parameters);
void medicalEventResponseTask(void* parameters);
void systemHeartbeatTask(void* parameters);
void dataLoggingTask(void* parameters);

// ----- Web Interface Functions -----
void sendMedicalDashboard() {
  String html = "<html><body><h1>Heart Rate Monitor</h1>";
  
  html += "<p><b>System Mode:</b> " + String(monitoring_mode ? "NORMAL" : "EMERGENCY") + "</p>";
  html += "<p><b>Alert Status:</b> " + String(alert_active ? "ACTIVE" : "CLEAR") + "</p>";
  html += "<p><b>Heart Rate:</b> " + String(current_hr_reading) + " BPM</p>";
  html += "<p><b>Total Alerts:</b> " + String(alert_count) + "</p>";
  
  html += "<p><a href=\"/emergency\"><button>Emergency Alert</button></a></p>";
  html += "<p><a href=\"/toggle-mode\"><button>Toggle Mode</button></a></p>";
  html += "<p><a href=\"/\"><button>Refresh</button></a></p>";
  
  html += "</body></html>";
  
  medicalConsole.send(200, "text/html", html);
}

void handleRoot() {
  sendMedicalDashboard();
}

void handleEmergency() {
  // Trigger emergency button semaphore
  xSemaphoreGive(emergency_semaphore);
  sendMedicalDashboard();
}

void handleToggleMode() {
  // Trigger emergency button to toggle mode
  xSemaphoreGive(emergency_semaphore);
  sendMedicalDashboard();
}

// ----- Arduino Setup -----
void setup() {
  Serial.begin(115200);
  
  // Initialize GPIO pins
  pinMode(HEARTBEAT_LED, OUTPUT);
  pinMode(ALERT_LED, OUTPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);
  
  // Create FreeRTOS synchronization primitives
  hr_alert_semaphore = xSemaphoreCreateCounting(MAX_HR_EVENTS, 0);
  emergency_semaphore = xSemaphoreCreateBinary();
  serial_mutex = xSemaphoreCreateMutex();
  hr_data_queue = xQueueCreate(50, sizeof(int));
  
  // Verify semaphore creation
  if (!hr_alert_semaphore || !emergency_semaphore || !serial_mutex || !hr_data_queue) {
    Serial.println("ERROR: Failed to create synchronization primitives!");
    while(1) { delay(1000); }
  }
  
  // Configure web server routes
  medicalConsole.on("/", handleRoot);
  medicalConsole.on("/emergency", handleEmergency);
  medicalConsole.on("/toggle-mode", handleToggleMode);
  
  // Print system startup message
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("=== Healthcare Heart Rate Monitor Starting ===");
  Serial.println("System: Initializing FreeRTOS tasks...");
  xSemaphoreGive(serial_mutex);
  
  // Create FreeRTOS tasks with appropriate priorities
  xTaskCreate(wifiServerTask,          "WiFi_Server",    4096, NULL, 4, NULL);  // High priority - network
  xTaskCreate(emergencyButtonTask,     "Emergency_Btn",  2048, NULL, 6, NULL);  // Highest - emergency response
  xTaskCreate(heartRateMonitorTask,    "HR_Monitor",     2048, NULL, 5, NULL);  // High - critical monitoring
  xTaskCreate(medicalEventResponseTask,"Event_Response", 2048, NULL, 3, NULL);  // Medium - event handling
  xTaskCreate(dataLoggingTask,         "Data_Logger",    2048, NULL, 2, NULL);  // Low-medium - logging
  xTaskCreate(systemHeartbeatTask,     "Sys_Heartbeat",  1024, NULL, 1, NULL);  // Lowest - status indicator
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("System: All tasks created successfully");
  Serial.println("Healthcare monitoring system is now active!");
  xSemaphoreGive(serial_mutex);
}

void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ----- Task Implementations -----

void wifiServerTask(void* parameters) {
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.print("WiFi: Connecting to network");
  xSemaphoreGive(serial_mutex);
  
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("\nWiFi: Connected successfully!");
  Serial.print("WiFi: Medical dashboard available at: http://");
  Serial.println(WiFi.localIP());
  xSemaphoreGive(serial_mutex);
  
  // Start web server
  medicalConsole.begin();
  
  // Handle web requests continuously
  while (1) {
    medicalConsole.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void heartRateMonitorTask(void* parameters) {
  bool previous_alert_state = false;
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("HR Monitor: Heart rate monitoring started");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    // Read heart rate sensor (simulated with potentiometer)
    int adc_raw = analogRead(HR_SENSOR_PIN);
    int hr_reading = adc_raw / 40;  // Convert to realistic heart rate values (0-102 BPM)
    current_hr_reading = hr_reading;
    
    // Send data to logging queue
    xQueueSend(hr_data_queue, &hr_reading, 0);
    
    // Check for heart rate anomalies
    bool current_alert = (hr_reading < HR_LOW_THRESHOLD) || (hr_reading > HR_HIGH_THRESHOLD);
    
    // Trigger alert on new anomaly detection (edge-triggered)
    if (current_alert && !previous_alert_state) {
      xSemaphoreGive(hr_alert_semaphore);
      previous_alert_state = true;
      
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      if (hr_reading < HR_LOW_THRESHOLD) {
        Serial.println("HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low!");
      } else {
        Serial.println("HR Monitor: ⚠️ TACHYCARDIA DETECTED - Heart rate too high!");
      }
      Serial.print("HR Monitor: Current reading: ");
      Serial.println(hr_reading);
      xSemaphoreGive(serial_mutex);
    } else if (!current_alert) {
      previous_alert_state = false;
    }
    
    // Sample every 17ms as specified
    vTaskDelay(pdMS_TO_TICKS(HR_SAMPLE_RATE_MS));
  }
}

void emergencyButtonTask(void* parameters) {
  int previous_button_state = HIGH;
  TickType_t last_press_time = 0;
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Emergency: Emergency button monitoring active");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    int current_button_state = digitalRead(EMERGENCY_BTN);
    
    // Detect button press (HIGH to LOW transition with debouncing)
    if (current_button_state == LOW && previous_button_state == HIGH) {
      TickType_t current_time = xTaskGetTickCount();
      
      if ((current_time - last_press_time) * portTICK_PERIOD_MS > DEBOUNCE_MS) {
        last_press_time = current_time;
        
        // Signal emergency event
        xSemaphoreGive(emergency_semaphore);
        
        xSemaphoreTake(serial_mutex, portMAX_DELAY);
        Serial.println("Emergency: 🚨 EMERGENCY BUTTON PRESSED!");
        xSemaphoreGive(serial_mutex);
      }
    }
    
    previous_button_state = current_button_state;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void medicalEventResponseTask(void* parameters) {
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Event Response: Medical event handler ready");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    // Handle heart rate alerts (counting semaphore)
    if (xSemaphoreTake(hr_alert_semaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
      alert_count++;
      alert_active = true;
      
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.println("Response: 💓 Processing heart rate alert");
      Serial.print("Response: Total alerts today: ");
      Serial.println(alert_count);
      xSemaphoreGive(serial_mutex);
      
      // Visual alert - flash red LED rapidly
      for (int i = 0; i < 5; i++) {
        digitalWrite(ALERT_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(ALERT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      
      alert_active = false;
    }
    
    // Handle emergency button events (binary semaphore)
    if (xSemaphoreTake(emergency_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
      monitoring_mode = !monitoring_mode;
      
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.print("Response: 🔄 System mode changed to: ");
      Serial.println(monitoring_mode ? "NORMAL MONITORING" : "EMERGENCY MODE");
      xSemaphoreGive(serial_mutex);
      
      // Visual feedback - longer red LED pulse
      digitalWrite(ALERT_LED, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(ALERT_LED, LOW);
    }
    
    // Small delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void systemHeartbeatTask(void* parameters) {
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Heartbeat: System status indicator started");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    // 1 Hz heartbeat (1 second on, 1 second off)
    digitalWrite(HEARTBEAT_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(HEARTBEAT_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void dataLoggingTask(void* parameters) {
  int sensor_value;
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Data Logger: Heart rate data logging started");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    // Process queued sensor data
    if (xQueueReceive(hr_data_queue, &sensor_value, portMAX_DELAY) == pdTRUE) {
      // Update global variable for web interface
      current_hr_reading = sensor_value;
      
      // Log data periodically (every 10th reading to avoid spam)
      static int log_counter = 0;
      if (++log_counter >= 10) {
        log_counter = 0;
        
        xSemaphoreTake(serial_mutex, portMAX_DELAY);
        Serial.print("Data Logger: HR=");
        Serial.print(sensor_value);
        Serial.print(" Mode=");
        Serial.print(monitoring_mode ? "NORMAL" : "EMERGENCY");
        Serial.print(" Alerts=");
        Serial.println(alert_count);
        xSemaphoreGive(serial_mutex);
      }
    }
  }
}
