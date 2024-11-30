#include <WiFi.h>


#include <PubSubClient.h>
#include <Wire.h>
#include <RTClib.h>
#include "DHT.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <MQUnifiedsensor.h>



const int chipSelect = 5;  

#define         Board                   ("ESP-32")
#define         MQ4_Pin                 33 // MQ-4 sensor pin
#define         MQ137_Pin               35 // MQ-137 sensor pin
#define         Type                    ("MQ-4")
#define         Voltage_Resolution      (5)  // MQ sensors are generally 5V, though ESP32 uses 3.3V
#define         ADC_Bit_Resolution      (12)
#define         RatioMQ4CleanAir        (4.4)
#define         DHTPIN                  32  // DHT22 sensor data pin
#define         DHTTYPE                 DHT22 // Define DHT22 sensor type
#define LED_PIN 12             // LED pin to control

MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ4_Pin, Type);


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
const float TEMP_THRESHOLD = 25.0;       // Temperature threshold
const float HUMIDITY_THRESHOLD = 50.0;   // Humidity threshold
const float METHANE_THRESHOLD = 1.0;       // Methane threshold
const float AMMONIA_THRESHOLD = 5.0;       //Ammonia threshold

// Variables for spoilage detection
bool spoilageTimerRunning = false; // Timer running flag
unsigned long thresholdStartMillis = 0;  

unsigned long tempStartMillis = 0;
bool tempTimerRunning = false;

unsigned long humidityStartMillis = 0;
bool humidityTimerRunning = false;

unsigned long tempWarningStartMillis = 0;
bool tempWarningTimerRunning = false;

unsigned long humidityWarningStartMillis = 0;
bool humidityWarningTimerRunning = false;

unsigned long ammoniaThresholdStartMillis = 0;
bool ammoniaTimerRunning = false;

unsigned long storageStartMillis = 0;
bool storageTimerRunning = false; 

//1000 = 1 second
const unsigned long SPOILAGE_DELAY = 20000;  // Time in milliseconds to confirm spoilage
const unsigned long TEMP_SPOILAGE_DELAY = 100000; 
const unsigned long HUMIDITY_SPOILAGE_DELAY = 10000; 
const unsigned long AMMONIA_SPOILAGE_DELAY = 5000; 
const unsigned long STORAGE_SPOILAGE_DELAY = 24L * 60 * 60 * 1000; 


const unsigned long TEMP_WARNING_DELAY = 5000; 
const unsigned long HUMIDITY_WARNING_DELAY = 5000; 



unsigned long lastReconnectAttempt = 0;  // Timestamp for the last reconnect attempt
const unsigned long RECONNECT_INTERVAL = 30000;  // 30 seconds delay between reconnect attempts


const unsigned long storeInterval = 5000; // 1 minute in milliseconds
unsigned long lastStoreTime = 0; // Initialize to 0 at the start

unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 10000;  // 10 seconds

bool tempWarningTriggered = false;
bool humidityWarningTriggered = false;

const float VCC = 5.0;
const float RL = 10.0;

const float R0 = 20.0;
const float A = 116.602;
const float B = -2.769;

bool wifiConnected = false;


void setup() {
    Serial.begin(115200);
    dht.begin();
    Wire.begin();
    MQ4.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ4.setA(1012.7); MQ4.setB(-2.786); // Set calibration values for MQ-4
    MQ4.init(); 

  // Serial.print("Calibrating please wait.");

  // float calcR0 = 0;
  // for(int i = 1; i <= 10; i++) {
  //   MQ4.update();
  //   calcR0 += MQ4.calibrate(RatioMQ4CleanAir);
    
  //   Serial.print("calcR0: ");
  //   Serial.print(calcR0);
  //   Serial.print(", RatioMQ4CleanAir: ");
  //   Serial.println(RatioMQ4CleanAir);

  // }
  float preCalibratedR0 = 10.0;  
  MQ4.setR0(preCalibratedR0);
    Serial.print("R0 Value: ");
    Serial.println(preCalibratedR0);
  // Serial.println("  done!");

  // if (isinf(calcR0)) { 
  //   Serial.println("Warning: Connection issue, R0 is infinite. Check wiring.");
  //   while (1);
  // }
  // if (calcR0 == 0) {
  //   Serial.println("Warning: Connection issue, R0 is zero. Check wiring.");
  //   while (1);
  // }
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setBufferSize(512);
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }

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

    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running, setting time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
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
              startMonitoring = false; // Stop monitoring
              // Reset spoilage detection timers and flags
              spoilageTimerRunning = false;
              thresholdStartMillis = 0;

              tempStartMillis = 0;
              tempTimerRunning = false;

              humidityStartMillis = 0;
              humidityTimerRunning = false;

              ammoniaThresholdStartMillis = 0;
              ammoniaTimerRunning = false;

              storageStartMillis = 0;
              storageTimerRunning = false;

              // Reset warning timers and flags
              tempWarningStartMillis = 0;
              tempWarningTimerRunning = false;
              tempWarningTriggered = false;

              humidityWarningStartMillis = 0;
              humidityWarningTimerRunning = false;
              humidityWarningTriggered = false;
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
            MQ4.update();
            float humidity = dht.readHumidity();
            float temperature = dht.readTemperature();
            float methane = MQ4.readSensor();
            int analogValue = analogRead(MQ137_Pin); 
            DateTime now = rtc.now();
            float voltage = analogValue * (VCC / 4095.0); // Convert to voltage
            float RS = RL * (VCC - voltage) / voltage;
            float ratio = RS / R0;
            float ammonia = A * pow(ratio, B);
            if (isnan(humidity) || isnan(temperature)) {
                Serial.println("Failed to read from DHT sensor!");
                return;
            }

            String spoilageStatus = "Food is Fresh";
            String spoilageStatusWarningTemp = "Temperature is safe";
            String spoilageStatusWarningHumidity = "Humidity is safe";
            String spoilageStatusMethane = "Methane is safe";
            String spoilageStatusAmmonia = "Ammonia is safe";


            if (!storageTimerRunning) {
                storageStartMillis = millis();
                storageTimerRunning = true;
                Serial.println("Storage timer started.");
            } else {
                // Check if the storage time has exceeded the spoilage delay
                if (millis() - storageStartMillis >= STORAGE_SPOILAGE_DELAY) {
                    Serial.println("Food is spoiled due to prolonged storage!");
                    // String spoilagePayload = "{\"food_type\": \"" + selectedFood + "\", \"status\": \"spoiled\", \"reason\": \"storage\"}";
                    spoilageStatus = "Food is Spoiled due to Storage Time";
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
                    spoilageStatusMethane = "Food is Spoiled Due To High Methane";
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
          spoilageStatusAmmonia = "Food is at Risk Due to High Ammonia";
          if (!ammoniaTimerRunning) {
              ammoniaThresholdStartMillis = millis();
              ammoniaTimerRunning = true;
              Serial.println("Ammonia above threshold, starting spoilage timer.");
          } else if (millis() - ammoniaThresholdStartMillis >= AMMONIA_SPOILAGE_DELAY) {
              Serial.println("Food is spoiled due to ammonia!");
              String spoilagePayload = "{\"food_type\": \"" + selectedFood + "\", \"status\": \"spoiled\"}";
              startMonitoring = false;
              spoilageStatus = "Food is Spoiled";
              spoilageStatusAmmonia = "Food is Spoiled Due To High Ammonia";
          } else {
        // Track and display the time the ammonia has been above the threshold
        unsigned long elapsedAmmoniaTime = millis() - ammoniaThresholdStartMillis;
        Serial.print("Ammonia time above threshold: ");
        Serial.print(elapsedAmmoniaTime / 1000); // Convert to seconds for readability
        Serial.println(" seconds.");
    }
        } else {
            ammoniaTimerRunning = false;
            ammoniaThresholdStartMillis = 0;
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
            if (millis() - tempStartMillis >= TEMP_SPOILAGE_DELAY) {
                Serial.println("Food is spoiled due to high temperature!");
                spoilageStatus = "Food is Spoiled";

                // Stop monitoring
                startMonitoring = false;
                tempTimerRunning = false;
                tempStartMillis = 0;
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
                     ", \"spoilage_status_warning_humidity\": \"" + spoilageStatusWarningHumidity + "\"}";      

            String topic = "sensor/data/" + String(token);
            client.publish(topic.c_str(), jsonPayload.c_str());

            Serial.println("Data sent to topic: " + topic);
            Serial.println("Data sent: " + jsonPayload);


        }

    delay(10000);  // 10-second delay before next loop
}

void storeDataLocally() {
  if(startMonitoring){
    Serial.println("Storing monitoring data locally");
    MQ4.update();
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    float methane = MQ4.readSensor();
    int analogValue = analogRead(MQ137_Pin); 
    DateTime now = rtc.now();
    float voltage = analogValue * (VCC / 4095.0); // Convert to voltage
    float RS = RL * (VCC - voltage) / voltage;
    float ratio = RS / R0;
    float ammonia = A * pow(ratio, B);

    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

            String spoilageStatus = "Food is Fresh";
            String spoilageStatusWarningTemp = "Temperature is safe";
            String spoilageStatusWarningHumidity = "Humidity is safe";
            String spoilageStatusMethane = "Methane is safe";
            String spoilageStatusAmmonia = "Ammonia is safe";

            if (!storageTimerRunning) {
                storageStartMillis = millis();
                storageTimerRunning = true;
                Serial.println("Storage timer started.");
            } else {
                // Check if the storage time has exceeded the spoilage delay
                if (millis() - storageStartMillis >= STORAGE_SPOILAGE_DELAY) {
                    Serial.println("Food is spoiled due to prolonged storage!");
                    // String spoilagePayload = "{\"food_type\": \"" + selectedFood + "\", \"status\": \"spoiled\", \"reason\": \"storage\"}";
                    spoilageStatus = "Food is Spoiled due to Storage Time";
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
                    spoilageStatusMethane = "Food is Spoiled Due To High Methane";
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
          spoilageStatusAmmonia = "Food is at Risk Due to High Ammonia";
          if (!ammoniaTimerRunning) {
              ammoniaThresholdStartMillis = millis();
              ammoniaTimerRunning = true;
              Serial.println("Ammonia above threshold, starting spoilage timer.");
          } else if (millis() - ammoniaThresholdStartMillis >= AMMONIA_SPOILAGE_DELAY) {
              Serial.println("Food is spoiled due to ammonia!");
              String spoilagePayload = "{\"food_type\": \"" + selectedFood + "\", \"status\": \"spoiled\"}";
              startMonitoring = false;
              spoilageStatus = "Food is Spoiled";
              spoilageStatusAmmonia = "Food is Spoiled Due To High Ammonia";
          } else {
        // Track and display the time the ammonia has been above the threshold
        unsigned long elapsedAmmoniaTime = millis() - ammoniaThresholdStartMillis;
        Serial.print("Ammonia time above threshold: ");
        Serial.print(elapsedAmmoniaTime / 1000); // Convert to seconds for readability
        Serial.println(" seconds.");
    }
        } else {
            ammoniaTimerRunning = false;
            ammoniaThresholdStartMillis = 0;
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
            if (millis() - tempStartMillis >= TEMP_SPOILAGE_DELAY) {
                Serial.println("Food is spoiled due to high temperature!");
                spoilageStatus = "Food is Spoiled";

                // Stop monitoring
                startMonitoring = false;
                tempTimerRunning = false;
                tempStartMillis = 0;
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
                     ", \"spoilage_status_warning_humidity\": \"" + spoilageStatusWarningHumidity + "\"}";      




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
        }
        dataFile.close();  // Close the file after reading
        SD.remove("/data.txt");  // Optional: Remove the file after sending its contents
    } else {
        Serial.println("Error opening file for reading");
    }
}


void reconnect() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        // storeDataLocally();  // Store data locally during disconnection
        if (millis() - lastStoreTime >= storeInterval) {
            lastStoreTime = millis();  // Update the last store time
            storeDataLocally();        // Store data locally
        }
  
        if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
            Serial.println("connected");  
            client.subscribe(("sensor/monitoring/" + String(token)).c_str());  // Subscribe to the food selection topic
             
            // Send stored data from the SD card
            sendDataFromSDCard();
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(10000);  // Wait for 10 seconds before retrying
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

