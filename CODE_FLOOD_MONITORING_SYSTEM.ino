#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>

// Sensor pins
const int soilMoistureAnalogPin = A0;  // Soil Moisture Sensor (Analog pin)
const int trigPin = 5;                 // Ultrasonic Sensor Trigger pin
const int echoPin = 4;                 // Ultrasonic Sensor Echo pin
const int dhtPin = 12;                 // DHT11 Sensor pin
const int vibrationPin = 13;           // Vibration Sensor pin
const int flowSensorPin = 14;          // Water Flow Sensor pin (D5 in NodeMCU)
const int lampPin = 2;                 // Lamp pin to indicate flood alert (D4 on NodeMCU)

// DHT11 Sensor setup
#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

// Flood detection threshold (in cm)
const long floodThreshold = 10;  // Distance less than 10 cm indicates probability of flood
const long lampThreshold = 15;   // Distance less than 15 cm to glow LED

// ThingSpeak settings
const char* ssid = "abcd";  // Your WiFi SSID
const char* password = "12345678";  // Your WiFi Password
const char* apiKey = "41S8TG184QJQ413V";  // Your ThingSpeak Write API Key
unsigned long channelNumber =  2665580;  // Your ThingSpeak Channel Number

WiFiClient client;

// Variables for water flow sensor
volatile int pulseCount = 0;
float flowRate = 0;
unsigned long lastFlowTime = 0;
const float calibrationFactor = 4.5;  // Calibration factor for your specific flow sensor

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);

    // Set up sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(vibrationPin, INPUT_PULLUP);  // Vibration sensor with pullup
    pinMode(flowSensorPin, INPUT_PULLUP); // Water flow sensor with pullup
    pinMode(lampPin, OUTPUT);             // Set the lamp pin as OUTPUT

    // Initialize DHT11 sensor
    dht.begin();

    // Connect to WiFi
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");

    // Initialize ThingSpeak
    ThingSpeak.begin(client);

    // Attach interrupt to water flow sensor
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
}

void loop() {
    // *** Ultrasonic Sensor - Distance Measurement ***
    long duration, distance;

    // Clear the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Trigger the ultrasonic pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pin
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;  // Calculate distance in cm
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Probability of Flood Alert
    if (distance < floodThreshold) {
        Serial.println("Alert: There is a high probability of flooding!");
    } else {
        Serial.println("No Flood Detected.");
    }

    // Turn on/off the indicator lamp based on distance
    if (distance < lampThreshold) {
        digitalWrite(lampPin, HIGH);  // Turn ON the lamp
        Serial.println("Lamp ON: Distance less than 15 cm");
    } else {
        digitalWrite(lampPin, LOW);   // Turn OFF the lamp
        Serial.println("Lamp OFF: Distance greater than or equal to 15 cm");
    }

    // *** DHT11 Sensor - Temperature and Humidity ***
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
    } else {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println("°C");

        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println("%");
    }

    // *** Vibration Sensor ***
    int vibrationState = digitalRead(vibrationPin);
    bool isVibrating = (vibrationState == LOW);  // Check if vibration is detected

    if (isVibrating) {
        Serial.println("Alert: High Vibration Detected!");
    } else {
        Serial.println("Normal: No problem detected.");
    }

    // *** Soil Moisture Sensor ***
    int soilMoistureAnalogValue = analogRead(soilMoistureAnalogPin);  // Read analog value
    Serial.print("Soil Moisture (Analog): ");
    Serial.println(soilMoistureAnalogValue);

    // Calculate soil moisture percentage (0-100%)
    int soilMoisturePercent = map(soilMoistureAnalogValue, 1023, 0, 0, 100);
    Serial.print("Soil Moisture (%): ");
    Serial.print(soilMoisturePercent);
    Serial.println("%");

    // Determine soil moisture condition
    if (soilMoisturePercent >= 71) {
        Serial.println("Alert: Wet condition detected. Potential risk of flood.");
    } else if (soilMoisturePercent >= 21) {
        Serial.println("Moist Condition.");
    } else {
        Serial.println("Dry Condition.");
    }

    // *** Water Flow Sensor ***
    unsigned long currentTime = millis();
    if (currentTime - lastFlowTime >= 1000) {  // Calculate flow every second
        detachInterrupt(digitalPinToInterrupt(flowSensorPin));

        // Calculate flow rate in liters/min (flowRate = (pulseCount / calibrationFactor) / seconds)
        flowRate = (pulseCount / calibrationFactor);
        pulseCount = 0;  // Reset pulse count

        Serial.print("Water Flow Rate: ");
        Serial.print(flowRate);
        Serial.println(" L/min");

        lastFlowTime = currentTime;
        attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
    }

    // Send data to ThingSpeak
    ThingSpeak.setField(1, distance);            // Field 1: Distance (Ultrasonic)
    ThingSpeak.setField(2, temperature);         // Field 2: Temperature (DHT11)
    ThingSpeak.setField(3, humidity);            // Field 3: Humidity (DHT11)
    ThingSpeak.setField(4, soilMoisturePercent); // Field 4: Soil Moisture Percentage
    ThingSpeak.setField(5, isVibrating ? 1 : 0); // Field 5: Vibration Status (1 = High Vibration)
    ThingSpeak.setField(6, flowRate);            // Field 6: Water Flow Rate (L/min)

    int result = ThingSpeak.writeFields(channelNumber, apiKey);
    if (result == 200) {
        Serial.println("Data sent to ThingSpeak successfully.");
    } else {
        Serial.print("Failed to send data to ThingSpeak. Error code: ");
        Serial.println(result);
    }

    // Delay for next reading
    delay(10000);  // Send data every 10 seconds (adjust as needed)
}