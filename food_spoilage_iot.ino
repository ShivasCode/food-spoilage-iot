#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <RTClib.h>
#include "DHT.h"
#include <ArduinoJson.h>
#include <SD.h>

const int chipSelect = 5;  
#define         DHTPIN                  32  // DHT22 sensor data pin
#define         DHTTYPE                 DHT22 // Define DHT22 sensor type
#define         LED_PIN                 12    // LED pin to control

File dataFile;

DHT dht(DHTPIN, DHTTYPE);

RTC_DS1307 rtc;

// Wi-Fi credentials
const char* ssid = "SKYFiber_MESH_3189";
const char* password = "548209325";

// MQTT credentials
const char* mqtt_server = "192.168.55.100";
const int mqtt_port = 1883;
const char* mqtt_user = "admin";
const char* mqtt_pass = "admin";

WiFiClient espClient;
PubSubClient client(espClient);

const char* token = "1b4980f3491893dbff45774c86555c583c987700";

String selectedFood = "";  // Store the selected food type
bool startMonitoring = false;  // Control flag to check if monitoring should start

// Food spoilage thresholds
const float TEMP_THRESHOLD = 32.0;       // Temperature threshold
const float METHANE_THRESHOLD = 120.0;       // Methane threshold
const float AMMONIA_THRESHOLD = 100.0;

// Variables for spoilage detection
bool spoilageTimerRunning = false; // Timer running flag
bool ammoniaSpoilageTimerRunning = false; 

unsigned long thresholdStartMillis = 0;  
unsigned long ammoniaThresholdStartMillis = 0;  


unsigned long tempStartMillis = 0;
bool tempTimerRunning = false;

unsigned long humidityStartMillis = 0;
bool humidityTimerRunning = false;

unsigned long tempWarningStartMillis = 0;
bool tempWarningTimerRunning = false;

unsigned long humidityWarningStartMillis = 0;
bool humidityWarningTimerRunning = false;

unsigned long storageStartMillis = 0;
bool storageTimerRunning = false; 

//1000 = 1 second
const unsigned long SPOILAGE_DELAY = 5L * 60 * 1000;  // 5 minutes
const unsigned long AMMONIA_SPOILAGE_DELAY = 5L * 60 * 1000;  // 5 minutes

// const unsigned long TEMP_RISK_DELAY = 2L * 60 * 60 * 1000;  // 2 hours in milliseconds
const unsigned long TEMP_RISK_DELAY = 1L * 60 * 1000;  // 3 minutes

// const unsigned long STORAGE_RISK_DELAY = 3L * 24 * 60 * 60 * 1000;  // 3 days in milliseconds
const unsigned long STORAGE_RISK_DELAY = 1L * 60 * 1000;  // 3 minutes in milliseconds


unsigned long lastReconnectAttempt = 0;  // Timestamp for the last reconnect attempt
const unsigned long RECONNECT_INTERVAL = 30000;  // 30 seconds delay between reconnect attempts


// const unsigned long storeInterval = 5L * 60 * 1000; // 1 minute in milliseconds
const unsigned long storeInterval = 5000; // 1 minute in milliseconds

unsigned long lastStoreTime = 0; // Initialize to 0 at the start

unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 10000;  // 10 seconds

bool tempWarningTriggered = false;
bool humidityWarningTriggered = false;


bool wifiConnected = false;

unsigned long lastMonitoringMillis = 0;
// const unsigned long MONITORING_INTERVAL = 1000; // 5 minutes
const unsigned long MONITORING_INTERVAL = 5000; // 5 minutes



void setup() {
    Serial.begin(115200);
    // dht setup
    dht.begin();
    // wire setup
    Wire.begin();

    // wifi setup
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    // mqtt server connection
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setBufferSize(2048);

    // rtc connection
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }

    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running, setting time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // microSD card initialization
    if (!SD.begin(chipSelect)) {
      Serial.println("SD Card initialization failed. Check the SD card or wiring.");
    }
    Serial.println("SD Card initialized successfully.");
       
    // Check if data.txt already exists
    if (!SD.exists("/data.txt")) {
      Serial.println("data.txt not found. Creating an empty data.txt...");

      dataFile = SD.open("/data.txt", FILE_WRITE);  
      if (dataFile) {
        dataFile.close();
        Serial.println("Empty data.txt created successfully.");
      } else {
        Serial.println("Error creating data.txt.");
      }
    } else {
      Serial.println("data.txt already exists. Skipping creation.");
    }

    // Turn on led to indicate food safe
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(LED_PIN, HIGH);
}

void callback(char* topic, byte* payload, unsigned int length) {
    String receivedPayload;

    // Build the received payload string
    for (unsigned int i = 0; i < length; i++) {
        receivedPayload += (char)payload[i];
    }

    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    Serial.println(receivedPayload);

    // Check if the topic matches
    if (String(topic) == "sensor/monitoring/" + String(token)) {
        // Parse the received payload (JSON)
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, receivedPayload);
        if (error) {
            Serial.print("JSON parse failed: ");
            Serial.println(error.f_str());
            return;
        }

        // Check for "food_type" in the payload
        if (doc.containsKey("food_type")) {
            selectedFood = doc["food_type"].as<String>();
            Serial.print("Selected food type: ");
            Serial.println(selectedFood);
            
            // Start monitoring if a valid food type is received
            if (selectedFood != "") {
                startMonitoring = true;
            }
        }
        if (doc.containsKey("start_monitoring")) {
            bool startMonitoringFlag = doc["start_monitoring"];
            
            if (!startMonitoringFlag) {
                resetMonitoringState();

            } 
        }
    } else {
        Serial.println("Ignored message from an unexpected topic.");
    }
}
void sendHeartbeat() {
  if (client.connected()) {
    String topic = "device/status/" + String(token);  // Construct the topic with the token
    String jsonPayload = "{\"status\": \"online\"}";  // Change status to "online"
    
    // Publish the status to the constructed topic
    client.publish(topic.c_str(), jsonPayload.c_str());

    Serial.println("Heartbeat sent: " + jsonPayload);
  } else {
    Serial.println("MQTT client not connected");
  }
}


void loop() {
    if (Serial.available() > 0) {
          char command = Serial.read(); // Read input from Serial monitor
          
          // Check for connection command 
          if (command == 'A') {
              connectWiFi();
          } 
          // Check for disconnection command
          else if (command == 'B') {
              disconnectWiFi();
          }
      }

      if (!client.connected()) {
          reconnect();
        }
        // Store sensor data locally while disconnected
        client.loop();  // Handle MQTT communication if connected
        if (millis() - lastHeartbeat >= heartbeatInterval) {
            lastHeartbeat = millis();
            sendHeartbeat();  // Send heartbeat
          }
        if (startMonitoring) {
            Serial.println("Monitoring started...");
            if (lastMonitoringMillis == 0 || millis() - lastMonitoringMillis >= MONITORING_INTERVAL) {
            lastMonitoringMillis = millis(); // Update the last execution time
            float humidity = dht.readHumidity();
            float temperature = dht.readTemperature();
            double methane = analogRead(33); // Store analog read as a float
            double ammonia = analogRead(35);
            DateTime now = rtc.now();
            String spoilageStatus = "Food is Fresh";
            String spoilageStatusWarningTemp = "Temperature is safe";
            String MethaneStatusMessage = "Methane level is within safe limits.";
            String TemperatureStatusMessage = "Temperature is within the safe range.";
            String StorageStatusMessage = "Food storage time is within the recommended duration.";
            String AmmoniaStatusMessage = "Ammonia level is within safe limits.";
            // Print the sensor value to the Serial Monitor
            Serial.print("Analog Reading: ");
            Serial.println(methane);
            if (isnan(humidity) || isnan(temperature)) {
                Serial.println("Failed to read from DHT sensor!");
                return;
            }
            if (!storageTimerRunning) {
                storageStartMillis = millis();
                storageTimerRunning = true;
                Serial.println("Storage timer started.");
            } else {
                // Check if the storage time has exceeded the spoilage delay
                if (millis() - storageStartMillis >= STORAGE_RISK_DELAY) {
                    Serial.println("Notify user that food stored over 3 days");
                    spoilageStatus = "Food is at Risk";
                    StorageStatusMessage = "Food is at risk due to being stored for over 3 days."; 
                } else {
                    unsigned long elapsedStorageTime = millis() - storageStartMillis;
                    Serial.print("Storage time elapsed: ");
                    Serial.print(elapsedStorageTime / 1000); // Convert to seconds for readability
                    Serial.println(" seconds.");
                }
            }

            if (methane > METHANE_THRESHOLD) {
              spoilageStatus = "Food is at Risk";
              MethaneStatusMessage = "Methane threshold exceeded. Food at risk.";
              if (!spoilageTimerRunning) {
                // Start the timer when the threshold is reached
                thresholdStartMillis = millis();
                spoilageTimerRunning = true;
                Serial.println("Methane above threshold, starting spoilage timer.");
            } else {
                // Check how long the readings have stayed in the threshold range
                if (millis() - thresholdStartMillis >= SPOILAGE_DELAY) {
                    Serial.println("Food is spoiled!");
                    startMonitoring = false;
                    spoilageStatus = "Food is Spoiled";
                    MethaneStatusMessage = "Food is Spoiled Due To High Methane";
                } else {
                // Track and display the time the methane has been above the threshold
                unsigned long elapsedMethaneTime = millis() - thresholdStartMillis;
                Serial.print("Methane time above threshold: ");
                Serial.print(elapsedMethaneTime / 1000); // Convert to seconds for readability
                Serial.println(" seconds.");
        }
            }
        } else {
            // Reset spoilage detection
            spoilageTimerRunning = false; 
            thresholdStartMillis = 0;

        }

        if (ammonia > AMMONIA_THRESHOLD) {
            spoilageStatus = "Food is at Risk";
            AmmoniaStatusMessage = "Ammonia threshold exceeded. Food at risk.";
            if (!spoilageTimerRunning) {     
                // Start the timer when the threshold is reached
                ammoniaThresholdStartMillis = millis();
                ammoniaSpoilageTimerRunning = true;
                Serial.println("Ammonia above threshold, starting spoilage timer.");
            } else {
                // Check how long the readings have stayed in the threshold range
                if (millis() - ammoniaThresholdStartMillis >= AMMONIA_SPOILAGE_DELAY) {
                    Serial.println("Food is spoiled due to ammonia!");
                    startMonitoring = false;
                    spoilageStatus = "Food is Spoiled";
                    AmmoniaStatusMessage = "Food is Spoiled Due To High Ammonia";
                } else {
                    // Track and display the time the ammonia has been above the threshold
                    unsigned long elapsedAmmoniaTime = millis() - ammoniaThresholdStartMillis;
                    Serial.print("Ammonia time above threshold: ");
                    Serial.print(elapsedAmmoniaTime / 1000); // Convert to seconds for readability
                    Serial.println(" seconds.");
                }
            }
        } else {
            // Reset spoilage detection
            ammoniaSpoilageTimerRunning = false;
            ammoniaThresholdStartMillis = 0;
        }


        // ----- High Temperature Spoilage Timer Logic -----
        // ----- Temperature Warning and Spoilage Detection -----
        if (temperature > TEMP_THRESHOLD) {
            spoilageStatusWarningTemp = "Food at Risk Due to High Temperature";
            TemperatureStatusMessage = "Food at Risk Due to High Temperature"; //notify user

            Serial.println("Temperature exceeds threshold.");

            // Send warning notification once
            if (!tempWarningTriggered) {
                Serial.println("Temperature warning notification: food is at risk.");
                tempWarningTriggered = true;
            }

            // Start spoilage timer if not already running
            if (!tempTimerRunning) {
                tempStartMillis = millis();
                tempTimerRunning = true;
                Serial.println("Starting temperature food risk timer.");
            }

            // Check if spoilage delay has passed
            if (millis() - tempStartMillis >= TEMP_RISK_DELAY) {
                Serial.println("Food is at risk due to high temperature!");
                spoilageStatus = "Food is at Risk";
                TemperatureStatusMessage = "Food has been exposed to high temperature for over 2 hours. Spoilage risk detected."; //notify user
            } else {
              // Track and display the time the temperature has been above the threshold
              unsigned long elapsedTempTime = millis() - tempStartMillis;
              Serial.print("Temperature time above threshold: ");
              Serial.print(elapsedTempTime / 1000); // Convert to seconds for readability
              Serial.println(" seconds.");
          }
        } else {
            // Reset timers and flags when temperature drops below threshold
            if (tempTimerRunning || tempWarningTriggered) {
                spoilageStatusWarningTemp = "Temperature is safe";
                tempWarningTriggered = false;
                tempTimerRunning = false;
                tempStartMillis = 0;
                Serial.println("Temperature dropped. Temperature timers reset.");
            }
        }




        if (spoilageStatus == "Food is Fresh" || spoilageStatus == "Food is at Risk") {
            digitalWrite(LED_PIN, HIGH);  // Turn on the LED
          }
          // Condition 2: If the food is spoiled, turn the LED off
          else if (spoilageStatus == "Food is Spoiled") {
            resetMonitoringState();

            digitalWrite(LED_PIN, LOW);  // Turn off the LED
          }

        String jsonPayload = "{\"token\": \"" + String(token) + "\"" +
                     ", \"temperature\": " + String(temperature) +
                     ", \"humidity\": " + String(humidity) +
                     ", \"methane\": " + String(methane) +
                     ", \"ammonia\": " + String(ammonia) +
                     ", \"food_type\": \"" + selectedFood + "\"" +
                     ", \"spoilage_status\": \"" + spoilageStatus + "\"" +
                     ", \"spoilage_status_warning_temp\": \"" + spoilageStatusWarningTemp + "\"" +
                     ", \"methane_status_message\": \"" + MethaneStatusMessage + "\"" +
                     ", \"ammonia_status_message\": \"" + AmmoniaStatusMessage + "\"" +
                     ", \"temperature_status_message\": \"" + TemperatureStatusMessage + "\"" +
                     ", \"storage_status_message\": \"" + StorageStatusMessage + "\"}";

            String topic = "sensor/data/" + String(token);
            client.publish(topic.c_str(), jsonPayload.c_str());

            Serial.println("Data sent: " + jsonPayload);

            }   else {
        // Print remaining time for debug purposes
        unsigned long remainingTime = MONITORING_INTERVAL - (millis() - lastMonitoringMillis);
        Serial.print("Next monitoring in: ");
        Serial.print(remainingTime / 1000);  // Convert to seconds for readability
        Serial.println(" seconds.");
               }
        } else {
              // Reset the timer when monitoring is stopped
              Serial.println("Monitoring is disabled.");
              lastMonitoringMillis = 0;
          }

          delay(5000);
}

void storeDataLocally() {
  Serial.print("start monitoring: ");
  Serial.println(startMonitoring);
  if(startMonitoring){
    Serial.println("Storing monitoring data locally");
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    double methane = analogRead(33); // Store analog read as a float
    DateTime now = rtc.now();
    String spoilageStatus = "Food is Fresh";
    String spoilageStatusWarningTemp = "Temperature is safe";
    String MethaneStatusMessage = "Methane level is within safe limits.";
    String TemperatureStatusMessage = "Temperature is within the safe range.";
    String StorageStatusMessage = "Food storage time is within the recommended duration.";
    String AmmoniaStatusMessage = "Ammonia level is within safe limits.";
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
          }
            if (!storageTimerRunning) {
                storageStartMillis = millis();
                storageTimerRunning = true;
                Serial.println("Storage timer started.");
            } else {
                // Check if the storage time has exceeded the spoilage delay
                if (millis() - storageStartMillis >= STORAGE_RISK_DELAY) {
                    Serial.println("Food is at risk due to prolonged storage!");
                    // String spoilagePayload = "{\"food_type\": \"" + selectedFood + "\", \"status\": \"spoiled\", \"reason\": \"storage\"}";
                    startMonitoring = false;
                    // client.publish("sensor/notifications", spoilagePayload.c_str());
                } else {
                    unsigned long elapsedStorageTime = millis() - storageStartMillis;
                    Serial.print("Storage time elapsed: ");
                    Serial.print(elapsedStorageTime / 1000); // Convert to seconds for readability
                    Serial.println(" seconds.");
                }
            }



            if (methane > METHANE_THRESHOLD) {
              spoilageStatus = "Food is at Risk";
              if (!spoilageTimerRunning) {
                // Start the timer when the threshold is reached
                thresholdStartMillis = millis();
                spoilageTimerRunning = true;
                Serial.println("Methane above threshold, starting spoilage timer.");
            } else {
                // Check how long the readings have stayed in the threshold range
                if (millis() - thresholdStartMillis >= SPOILAGE_DELAY) {
                    Serial.println("Food is spoiled!");
                    String spoilagePayload = "{\"food_type\": \"" + selectedFood + "\", \"status\": \"spoiled\"}";
                    startMonitoring = false;
                    spoilageStatus = "Food is Spoiled";
                    MethaneStatusMessage = "Methane threshold exceeded. Food at risk.";
                } else {
                // Track and display the time the methane has been above the threshold
                unsigned long elapsedMethaneTime = millis() - thresholdStartMillis;
                Serial.print("Methane time above threshold: ");
                Serial.print(elapsedMethaneTime / 1000); // Convert to seconds for readability
                Serial.println(" seconds.");
                }
            }
        } else {
            // Reset spoilage detection
            spoilageTimerRunning = false; 
            thresholdStartMillis = 0;

        }
        // ----- High Temperature Spoilage Timer Logic -----
        // ----- Temperature Warning and Spoilage Detection -----
        if (temperature > TEMP_THRESHOLD) {
            spoilageStatusWarningTemp = "Food at Risk Due to High Temperature";
            Serial.println("Temperature exceeds threshold.");

            // Send warning notification once
            if (!tempWarningTriggered) {
                Serial.println("Temperature warning notification: food is at risk.");
                tempWarningTriggered = true;
            }

            // Start spoilage timer if not already running
            if (!tempTimerRunning) {
                tempStartMillis = millis();
                tempTimerRunning = true;
                Serial.println("Starting temperature spoilage timer.");
            }

            // Check if spoilage delay has passed
            if (millis() - tempStartMillis >= TEMP_RISK_DELAY) {
                Serial.println("Food is spoiled due to high temperature!");
                spoilageStatus = "Food is at Risk";
          
            } else {
              // Track and display the time the temperature has been above the threshold
              unsigned long elapsedTempTime = millis() - tempStartMillis;
              Serial.print("Temperature time above threshold: ");
              Serial.print(elapsedTempTime / 1000); // Convert to seconds for readability
              Serial.println(" seconds.");
          }
        } else {
            // Reset timers and flags when temperature drops below threshold
            if (tempTimerRunning || tempWarningTriggered) {
                spoilageStatusWarningTemp = "Temperature is safe";
                tempWarningTriggered = false;
                tempTimerRunning = false;
                tempStartMillis = 0;
                Serial.println("Temperature is safe. Timers reset.");
            }
        }

        if (spoilageStatus == "Food is Fresh" || spoilageStatus == "Food is at Risk") {
            digitalWrite(LED_PIN, HIGH);  // Turn on the LED
          }
          // Condition 2: If the food is spoiled, turn the LED off  
          else if (spoilageStatus == "Food is Spoiled") {
            resetMonitoringState();

            digitalWrite(LED_PIN, LOW);  // Turn off the LED
          }

          String jsonPayload = "{"
                     "\"temperature\": " + String(temperature) + ", "
                     "\"humidity\": " + String(humidity) + ", "
                     "\"methane\": " + String(methane) + ", "
                     "\"food_type\": \"" + selectedFood + "\", "
                     "\"spoilage_status\": \"" + spoilageStatus + "\", "
                     "\"spoilage_status_warning_temp\": \"" + spoilageStatusWarningTemp + "\", "
                     "\"methane_status_message\": \"" + MethaneStatusMessage + "\", "
                     "\"temperature_status_message\": \"" + TemperatureStatusMessage + "\", "
                      "\"ammonia_status_message\": \"" + AmmoniaStatusMessage + "\", "
                     "\"storage_status_message\": \"" + StorageStatusMessage + "\""
                     "}";
  

    // Save data to SD card since MQTT is not connected
    saveDataToSD(jsonPayload);
  }
}

void saveDataToSD(String data) {
    dataFile = SD.open("/data.txt", FILE_APPEND);  // Open file in append mode
    if (dataFile) {
        dataFile.println(data);
        dataFile.close();
        Serial.println("Data stored locally: " + data);
    } else {
        Serial.println("Error opening data.txt for writing");
    }
}


void sendDataFromSDCard() {
    File dataFile = SD.open("/data.txt");  // Replace "data.txt" with your file name

    if (dataFile) {
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n');  // Read each line from the file
            String topic = "sensor/data/" + String(token);  // Adjust the topic as needed
            
            // Publish the line to the MQTT topic
            client.publish(topic.c_str(), line.c_str());
            Serial.println("Sent data: " + line);  // Debug print
            delay(100);  // Add a delay of 100ms between messages

        }
        dataFile.close();  // Close the file after reading
        if (SD.remove("/data.txt")) {
            Serial.println("File deleted successfully");  // Debug: Confirm file deletion
        } else {
            Serial.println("Failed to delete the file");  // Debug: Handle deletion failure
        }
    } else {
        Serial.println("Error opening file for reading");
    }
}


void reconnect() {
    // Check if WiFi is disconnected
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi is not connected. Storing data locally.");

    unsigned long currentMillis = millis();
    Serial.print("Current millis: ");
    Serial.println(currentMillis);
    Serial.print("Last store time: ");
    Serial.println(lastStoreTime);
    Serial.print("Store interval: ");
    Serial.println(storeInterval);

    // Check if enough time has passed since last store
    if (currentMillis - lastStoreTime >= storeInterval) {
        lastStoreTime = currentMillis;  // Update the last store time
        Serial.println("Calling storeDataLocally...");
        storeDataLocally();  // Attempt to store data
    } else {
        Serial.println("storeDataLocally not called. Interval not met.");
    }
    return; // Exit the function since WiFi needs to reconnect first
}

    // Attempt to reconnect to MQTT if WiFi is connected
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");

        // Store data locally during MQTT reconnect attempts
        if (millis() - lastStoreTime >= storeInterval) {
            lastStoreTime = millis();
            storeDataLocally();
        }

        if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
            Serial.println("Connected to MQTT.");
            client.subscribe(("sensor/monitoring/" + String(token)).c_str());  // Subscribe to the topic
            
            // Send stored data from the SD card
            sendDataFromSDCard();
        } else {
            Serial.print("Failed to connect to MQTT, rc=");
            Serial.println(client.state());
            delay(5000); // Wait before retrying
        }
    }
}
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Already connected to WiFi.");
        return;
    }

    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    
    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 10000; // 10 seconds timeout

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected successfully.");
        wifiConnected = true;
    } else {
        Serial.println("\nWiFi connection failed.");
    }
}

void disconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi is already disconnected.");
        return;
    }

    Serial.println("Disconnecting from WiFi...");
    WiFi.disconnect();
    wifiConnected = false;
    Serial.println("WiFi disconnected.");
}

void resetMonitoringState() {
  startMonitoring = false; // Stop monitoring

  // Reset spoilage detection timers and flags
  spoilageTimerRunning = false;
  thresholdStartMillis = 0;

  ammoniaSpoilageTimerRunning = false;
  ammoniaThresholdStartMillis = 0;


  tempStartMillis = 0;
  tempTimerRunning = false;


  storageStartMillis = 0;
  storageTimerRunning = false;

  // Reset warning timers and flags
  tempWarningStartMillis = 0;
  tempWarningTimerRunning = false;
  tempWarningTriggered = false;
}



