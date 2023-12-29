#include <ArduinoJson.h> 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = " "; // Enter your WiFi name
const char *password = " ";  // Enter WiFi password


// MQTT Broker
const char *mqtt_broker = " ";// broker address 
const char *mqtt_username = " "; // username for authentication
const char *mqtt_password = " ";// password for authentication
const int mqtt_port =  ;// port of MQTT over TCP

WiFiClient espClient;
PubSubClient client(espClient);

// Wind Speed Sensor
volatile byte rpmcount; // count signals
volatile unsigned long last_micros;
unsigned long timeold;
unsigned long timemeasure = 300.00; // seconds
unsigned long timeNow;
int countThing = 0;
int GPIO_pulse = 14; // Pin
float rpm, rps;     // frequencies
float velocity_ms;  //m/s
float calibration_value = 5.0; //This value is obtained from comparing with the manufacturer's anemometer sensor
volatile boolean flag = false;
String windSpeedState;

// Wind Direction Sensor
#define RX2 16 // pin RX2
#define TX2 17 // pin TX2
String data, arah_angin, s_angin, arah_derajat;
int a, b;

// Raindrop Sensor
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 4095;  // sensor maximum
int sensorReading;
String raindropState;

// Temperature Sensor
const int oneWireBus = 4;  // Pin
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
float temperatureC;
String temperatureState;

// Fuzzy Logic Function for Wind Speed Classification
String classifyWindSpeed(float windSpeed)
{
  // Membership functions
  float calm = std::max(0.0, std::min((0.2 - windSpeed) / 0.2, 1.0));                
  float slightlyCalm = std::max(0.0, std::min((1.5 - windSpeed) / 1.2, 1.0));        
  float slightBreeze = std::max(0.0, std::min((3.3 - windSpeed) / 1.7, 1.0));        
  float gentleBreeze = std::max(0.0, std::min((5.4 - windSpeed) / 2.1, 1.0));        
  float moderateBreeze = std::max(0.0, std::min((7.9 - windSpeed) / 2.5, 1.0));    
  float coolBreeze = std::max(0.0, std::min((10.7 - windSpeed) / 2.8, 1.0));      
  float strongBreeze = std::max(0.0, std::min((13.8 - windSpeed) / 2.9, 1.0));    
  float approachingStrongWind = std::max(0.0, std::min((17.1 - windSpeed) / 3.4, 1.0)); 
  float strongWind = std::max(0.0, std::min((20.7 - windSpeed) / 3.3, 1.0));       
  float veryStrongWind = std::max(0.0, std::min((24.4 - windSpeed) / 3.6, 1.0));    
  float stormyWind = std::max(0.0, std::min((28.4 - windSpeed) / 3.8, 1.0));       
  float violentStorm = std::max(0.0, std::min((32.6 - windSpeed) / 3.7, 1.0));   
  float hurricaneForce = std::max(0.0, std::min((36.0 - windSpeed) / 3.5, 1.0));   

  // Classification
  if (calm > 0.0)
  {
    return "Tenang";
  }
  else if (slightlyCalm > 0.0)
  {
    return "Sedikit Tenang";
  }
  else if (slightBreeze > 0.0)
  {
    return "Sedikit Hembusan Angin";
  }
  else if (gentleBreeze > 0.0)
  {
    return "Hembusan Angin Pelan";
  }
  else if (moderateBreeze > 0.0)
  {
    return "Hembusan Angin Sedang";
  }
  else if (coolBreeze > 0.0)
  {
    return "Hembusan Angin Sejuk";
  }
  else if (strongBreeze > 0.0)
  {
    return "Hembusan Angin Kuat";
  }
  else if (approachingStrongWind > 0.0)
  {
    return "Mendekati Kencang";
  }
  else if (strongWind > 0.0)
  {
    return "Kencang";
  }
  else if (veryStrongWind > 0.0)
  {
    return "Kencang Sekali";
  }
  else if (stormyWind > 0.0)
  {
    return "Badai";
  }
  else if (violentStorm > 0.0)
  {
    return "Badai Dahsyat";
  }
  else
  {
    return "Badai Topan";
  }
}

void ICACHE_RAM_ATTR rpm_anemometer()
{
  flag = true;
}

void setup()
{
  // Wind Speed Setup
  pinMode(GPIO_pulse, INPUT_PULLUP);
  digitalWrite(GPIO_pulse, LOW);
  Serial.begin(115200);
  detachInterrupt(digitalPinToInterrupt(GPIO_pulse));                         // force to initiate Interrupt on zero
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); //Initialize the interrupt pin
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  timeNow = 0;

  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  // Wind Direction Setup
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);

  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  
  //MQTT Broker Connection
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected()) {
    String client_id = "ESP32Client-";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT broker connected");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
  // Temperature Setup
  sensors.begin();
} 

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //Wind Speed Loop
  if (flag == true)
  {
    if (long(micros() - last_micros) >= 5000)
    {
      rpmcount++;
      last_micros = micros();
    }
    flag = false;
  }
  //Measure RPM
  if ((millis() - timeold) >= timemeasure * 1000)
  {
    countThing++;
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse)); // Disable interrupt when calculating

    rps = float(rpmcount) / float(timemeasure); // rotations per second
    velocity_ms = rps * calibration_value; // m/s
    Serial.print("Velocity = ");
    Serial.print(velocity_ms);
    Serial.print(" m/s");
    Serial.println("   ");

    windSpeedState = classifyWindSpeed(velocity_ms);
    Serial.println("Status Kecepatan Angin = " + windSpeedState);

    if (countThing == 1)
    {
      countThing = 0;
    }

    // Raindrop Loop
    sensorReading = analogRead(32);
    Serial.print("Intensitas Hujan = ");
    Serial.println(sensorReading);
    int range = map(sensorReading, sensorMin, sensorMax, 0, 2);
    switch (range) {
      case 0:
        raindropState = "Hujan Lebat";
        Serial.println(raindropState);
        break;
      case 1:
        raindropState = "Hujan Ringan";
        Serial.println(raindropState);
        break;
      case 2:
        raindropState = "Tidak Hujan";
        Serial.println(raindropState);
        break;
    }

    // Temperature Loop
    sensors.requestTemperatures(); 
    temperatureC = sensors.getTempCByIndex(0);
    Serial.print("Temperature = ");
    Serial.print(temperatureC);
    Serial.println("C");
    if (temperatureC >= 35)
    {
      temperatureState = "Panas";
      Serial.println(temperatureState);
    }
    else if (temperatureC <= 34 && temperatureC >= 24)
    {
      temperatureState = "Normal";
      Serial.println(temperatureState);
    }
    else if (temperatureC <= 23)
    {
      temperatureState = "Dingin";
      Serial.println(temperatureState);
    }

    // Wind Direction Loop
    if (Serial2.available()) // If there is data received from the sensor
    {
      data = Serial2.readString(); // Data received from the sensor starting with * and ending with #, example *1#
      a = data.indexOf("*"); // a is the index of *
      b = data.indexOf("#"); // b is the index of #
      s_angin = data.substring(a + 1, b); // remove * and # to get the value of wind direction
      if (s_angin.equals("1")) {
        arah_angin = "Utara     ";
        arah_derajat = "0";
      }
      if (s_angin.equals("2")) {
       arah_angin = "Timur Laut";
       arah_derajat = "45";
      }
      if (s_angin.equals("3")) {
       arah_angin = "Timur     ";
       arah_derajat = "90";
      }
      if (s_angin.equals("4")) {
        arah_angin = "Tenggara  ";
        arah_derajat = "135";
      }
      if (s_angin.equals("5")) {
        arah_angin = "Selatan   ";
        arah_derajat = "180";
      }
      if (s_angin.equals("6")) {
        arah_angin = "Barat Daya";
        arah_derajat = "225";
      }
      if (s_angin.equals("7")) {
        arah_angin = "Barat     ";
        arah_derajat = "270";
      }
      if (s_angin.equals("8")) {
        arah_angin = "Barat Laut";
        arah_derajat = "315";
      }
    Serial.println(arah_angin);
    Serial.println(arah_derajat + "°");
    Serial.println("  ");
    }
    timeold = millis();
    rpmcount = 0;
    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // enable interrupt
    publishAllSensors();
  }  
}

void publishAllSensors() {
  StaticJsonDocument<200> doc;
  doc["wind_speed"] = velocity_ms;
  doc["windspeed_state"] = windSpeedState; 
  doc["raindrop_intensity"] = sensorReading;
  doc["raindrop_state"] = raindropState;
  doc["temperature"] = temperatureC;
  doc["temperature_state"] = temperatureState;
  doc["wind_direction"] = arah_angin;
  doc["direction_degree"] = arah_derajat;


  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  client.publish("datasensor", jsonBuffer); 
}

void publishMQTT(const char* topic, float value) {
  char msg[50];
  dtostrf(value, 6, 2, msg);
  client.publish(topic, msg);
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP32Client-";
    clientId += String(WiFi.macAddress());
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Reconnected to MQTT broker");
    } else {
      Serial.print("MQTT reconnect failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}