# Application-5
Wokwi link: https://wokwi.com/projects/437014475323133953
Migration Strategy & Refactor Map

Outline the three largest structural changes you had to make when porting the single-loop Arduino sketch to your chosen RTOS framework.
1. Decomposition into Independent Tasks: Instead of one never ending loop doing all the task we split them into individualized units called FreeRTOS task. With this each task handles a single, specific job. 
2.  Introduction of Inter-Task Communication Primitives: Since each task is seprate it needs a way to communicate with the other tasks. Some specialized FreeRTOS tools such as queues and Semphores allow for this communication to be more efficient. Queues allow tasks to securly communicate data to each think of like a mailbox system. Semaphores keep track of how many times an event has occured to ensure task communicate reliably without error. 
3. Removal of Blocking Delays and Reliance on `vTaskDelay`: In Arduino delay() is used which causes the whole system to wait for the task to be done. In FreeRTOS there is vTaskDelay() which allows other task to continue while that specific task pauses. 

Include one code snippet (before → after) that best illustrates the refactor.
Before:
void loop() {
  // Read HR sensor
  int hr_reading = analogRead(HR_SENSOR_PIN) / 40;
  if (hr_reading < HR_LOW_THRESHOLD || hr_reading > HR_HIGH_THRESHOLD) {
    // Trigger alert
    digitalWrite(ALERT_LED, HIGH);
    delay(100);
    digitalWrite(ALERT_LED, LOW);
    // Update web server variable (global)
    current_hr_reading = hr_reading;
  }
  // Check button
  if (digitalRead(EMERGENCY_BTN) == LOW) {
    monitoring_mode = !monitoring_mode;
    delay(500); // Debounce
  }
  delay(HR_SAMPLE_RATE_MS); // Main loop delay
}

After: 
void heartRateMonitorTask(void* parameters) {
  bool previous_alert_state = false;
  while (1) {
    int adc_raw = analogRead(HR_SENSOR_PIN);
    int hr_reading = adc_raw / 40;
    current_hr_reading = hr_reading; // Update shared variable for web server
    xQueueSend(hr_data_queue, &hr_reading, 0); // Send data to logging task

    bool current_alert = (hr_reading < HR_LOW_THRESHOLD) || (hr_reading > HR_HIGH_THRESHOLD);
    if (current_alert && !previous_alert_state) {
      xSemaphoreGive(hr_alert_semaphore); // Signal medicalEventResponseTask
    }
    previous_alert_state = current_alert;
    vTaskDelay(pdMS_TO_TICKS(HR_SAMPLE_RATE_MS));
  }
}

Framework Trade-off Review

Compare ESP-IDF FreeRTOS vs Arduino + FreeRTOS for this project: list two development advantages and one limitation you encountered with the path you chose.
For this project I chose Arduino + FreeRTOS. I chose this one because for one I am very familiar with. I also think the that extensive amount of libraries allows so much time to be 
saved that would be spent configuring peripherals. The simplified API and pre-configured build system of Arduino allowed for faster iteration and testing of individual components before integrating them into the full RTOS application. While convenient, the Arduino abstraction can sometimes hinder the FreeRTOS mechanisms. Debugging issues related to task scheduling, memory management, or subtle RTOS interactions can be harder when you're not directly interacting with the ESP-IDF's more explicit APIs and build tools.

If you had chosen the other path, which specific API or tooling difference do you think would have helped / hurt?
If I had chosen the ESP-IDF FreeRTOS path, I believe the tooling difference of having the idf.py monitor with its built-in GDB integration would have been incredibly helpful. This provides a much better debugging experience, allowing me to set breakpoints, inspect task states, and step through code execution. However, the API difference of needing to manually configure every peripheral (like ADC setup, GPIO initialization, Wi-Fi provisioning) without the higher-level Arduino functions would have increased developement time significantly. 

Queue Depth & Memory Footprint

How did you size your sensor-data queue (length & item size)? Show one experiment where the queue nearly overflowed and explain how you detected or mitigated it.
The hr_data_queue was configured with a length of 50 and an item size of sizeof(int) to balance performance and resource usage. This decision took into account the heart rate sensor's sampling rate, which is defined by HR_SAMPLE_RATE_MS as 17 milliseconds—resulting in about 58.8 readings per second. However, the dataLoggingTask only logs every 10th reading, effectively consuming data from the queue at a much slower rate of around 5.88 items per second. To accommodate temporary delays—such as interruptions by higher-priority tasks or vTaskDelay periods—the queue was given a depth of 50 items. This provides a buffer of about 0.85 seconds of sensor data (50 items / 58.8 items per second), helping to ensure no data is lost during short delays. Since sizeof(int) is small, the total memory used by the queue (approximately 200 bytes) is minimal and well within acceptable limits for most embedded systems.
Queue overflow:
void dataLoggingTask(void* parameters) {
  int sensor_value;
  Serial.println("Data Logger: Heart rate data logging started");

  while (1) {
    if (xQueueReceive(hr_data_queue, &sensor_value, pdMS_TO_TICKS(100)) == pdTRUE) { // Increased wait time
      current_hr_reading = sensor_value;
      // Simulate a busy logging operation or a low-priority task being pre-empted
      vTaskDelay(pdMS_TO_TICKS(50)); // Artificial delay longer than HR_SAMPLE_RATE_MS
      static int log_counter = 0;
      if (++log_counter >= 10) {
        log_counter = 0;
        xSemaphoreTake(serial_mutex, portMAX_DELAY);
        Serial.print("Data Logger: HR=");
        Serial.print(sensor_value);
        Serial.print(" Queue_Count=");
        Serial.println(uxQueueMessagesWaiting(hr_data_queue)); // Check current queue depth
        xSemaphoreGive(serial_mutex);
      }
    } else {
      // If queue receive times out, it means no data arrived or queue was empty
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.println("Data Logger: Queue empty or timed out waiting for data.");
      xSemaphoreGive(serial_mutex);
    }
  }
}
By adding a vTaskDelay(pdMS_TO_TICKS(50)) in the dataLoggingTask, which is longer than the 17 ms sampling rate of the heartRateMonitorTask, the queue would begin to grow over time. This can be observed by monitoring uxQueueMessagesWaiting(hr_data_queue) via serial output, where the Queue_Count would increase steadily as it nears the maximum queue size (MAX_HR_EVENTS, set to 50).Then once reached kept at that point and never updated. 
There a few ways to mitigate this. One option is increasing the queue depth to handle occasional data bursts. Another is optimizing the dataLoggingTask by reducing its execution time or increasing its priority so it can keep up with incoming data. A third approach is implementing flow control, where the heartRateMonitorTask checks the result of xQueueSend and takes action if the queue is full. In this project, a 0-tick delay is used with xQueueSend, meaning it won’t block and will discard data if the queue is full. 

Debug & Trace Toolkit

List the most valuable debug technique you used (e.g., esp_log_level, vTaskGetInfo,  print-timestamp).
The most valuable debug technique I used was a combination of esp_log_level and uxQueueMessagesWaiting / xSemaphoreGetCount. With these I was able to understand the execution flow, identify which tasks were running, and observe the state of synchronization primitives.

Show a short trace/log excerpt that helped you verify correct task sequencing or uncover a bug.
Data Logger: HR=58 Mode=EMERGENCY Alerts=2
HR Monitor: ⚠️ TACHYCARDIA DETECTED - Heart rate too high!
HR Monitor: Current reading: 102
Response: 💓 Processing heart rate alert
Response: Total alerts today: 3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=102 Mode=EMERGENCY Alerts=3
Data Logger: HR=94 Mode=EMERGENCY Alerts=3
HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low!
HR Monitor: Current reading: 30
Response: 💓 Processing heart rate alert
Response: Total alerts today: 4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Data Logger: HR=0 Mode=EMERGENCY Alerts=4
Emergency: 🚨 EMERGENCY BUTTON PRESSED!
Response: 🔄 System mode changed to: NORMAL MONITORING
Data Logger: HR=0 Mode=NORMAL Alerts=4
This shows: 
Tasks are initializing in a reasonable order.
dataLoggingTask is regularly reporting HR and system state.
A "TACHYCARDIA DETECTED" message from HR Monitor is immediately followed by "Processing heart rate alert" from Event Response, indicating the hr_alert_semaphore correctly triggered the event handler.
An "EMERGENCY BUTTON PRESSED!" (simulated via web) is followed by "System mode changed," confirming the emergency_semaphore correctly updated the system mode.

Domain Reflection
Relate one design decision directly to your chosen theme’s stakes (e.g., “missing two heart-rate alerts could delay CPR by ≥1 s”).
Briefly propose the next real feature you would add in an industrial version of this system.

A key design decision in this heart rate monitoring system is the careful assignment of task priorities, especially given the high-stakes nature of healthcare applications. The emergencyButtonTask is given the highest priority, followed by the heartRateMonitorTask at priority 5. This ensures that critical events such as a heart rate falling outside safe limits or a nurse pressing the emergency button are detected and responded to immediately. In a medical setting, even a one-second delay in handling these events could mean missing the chance to initiate life-saving actions like CPR or alerting medical personnel, potentially resulting in severe patient harm. Assigning these tasks the highest priorities ensures they are never delayed by lower-priority processes like data logging or web server operations.

Looking ahead, an essential feature to add in an industrial-grade version of this system would be persistent event logging to non-volatile memory, such as an SD card or internal flash. In a real healthcare environment, having a secure, time-stamped record of heart rate anomalies, emergency button presses, and system state changes is vital for post-event review, regulatory compliance, and audits. This feature would require a dedicated FreeRTOS task for writing data to storage, along with a separate queue to handle structured log entries containing timestamps, event types, and relevant details like heart rate values. 

Web-Server Data Streaming: Benefits & Limitations

Describe two concrete advantages of pushing live sensor data to the on-board ESP32 web server (e.g., richer UI, off-board processing, remote monitoring, etc.).
Identify two limitations or trade-offs this decision introduces for a real-time system (think: added latency, increased heap/stack usage, potential priority inversion, Wi-Fi congestion, attack surface, maintenance of HTTP context inside RTOS tasks, etc.).

Streaming live sensor data to the onboard ESP32 web server has several practical benefits. First, it allows for remote monitoring through a browser, giving healthcare professionals or family members easy access to the patient’s heart rate and alert status without needing special software. The web dashboard—showing information like system mode, alert status, and heart rate offers a much clearer and more user-friendly interface compared to blinking LEDs or serial output, improving both usability and awareness of the patient’s condition.

Additionally, even though this project currently displays only basic data, the web server sets the stage for more advanced features in the future. For example, the data could be sent to an external server using WebSockets for long-term storage, trend analysis, or real-time graphs. This approach would offload heavy processing from the ESP32, which is limited in resources, and enable more powerful data visualization and analytics on a separate system.
Support your points with evidence from your own timing/heap logs or an experiment e.g., measure extra latency or increased CPU load when the server is actively streaming data versus idle.
Running a web server on the ESP32 introduces increased memory usage and CPU load, which can affect the performance of real-time tasks. Handling HTTP requests and maintaining a Wi-Fi connection requires significant resources heap memory for connection buffers and stack memory for the server task—along with CPU time to manage everything. This creates competition for limited system resources, potentially impacting critical operations if not managed properly.

In testing, it was noticeable that when a browser was actively connected and refreshing data, the wifiServerTask consumed more CPU time. Without careful handling through task priorities and vTaskDelay periods, this could lead to lower-priority but important tasks being delayed. For instance, the wifiServerTask has a stack size of 4096 bytes, much larger than other tasks like systemHeartbeatTask, which uses only 1024 bytes. This reflects the web server’s heavier memory requirements. While exact performance measurements were difficult to capture without advanced profiling tools, there was a clear increase in heap usage when Wi-Fi was connected and a client was active.

There’s also a risk of priority inversion if the wifiServerTask or underlying network drivers use shared resources protected by mutexes. If a higher-priority task needs access to a mutex held by a lower-priority task like the web server, it can get blocked, slowing down time-sensitive operations. Additionally, if web traffic is heavy, it can cause Wi-Fi congestion, introducing delays in data transmission. Although this wasn’t a major issue in this local monitoring system, it could be seen if heart rate updates on the dashboard lag during intense web activity.

Conclude with one design change you would implement if your system had to stream data reliably under heavy load
One design improvement I would make to support reliable data streaming under heavy load is to separate the web server’s data handling from the core RTOS tasks using a dedicated buffer or queue. This would help prevent the web server from interfering with time-sensitive operations.

Rather than allowing the web server to directly access shared global variables like current_hr_reading or alert_count, I would introduce a high-priority "Data Aggregator Task." This task would collect data from the existing sensor and logging queues, format it as needed, and write it into a fixed-size circular buffer or a dedicated queue reserved for the web server. The wifiServerTask would then only read from this buffer when needed, ensuring it doesn’t impact real-time tasks. Additionally, using WebSockets for data transmission could improve efficiency and reduce overhead compared to frequent HTTP polling, making it better suited for continuous streaming.
