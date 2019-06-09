/*
  Home automation - livingroom node

  Data gathering:
    Temperature & humidity
    Light amount

  Data publishing:
    Temperature & humidity
    Light amount
    Forward of Alexa command to turn on/off super lamp network (temporary)

  Topic subscription:

  Node description:
    The node gathers and publishes temperature, humidity and light amount information
    The node receives light requests in three different ways:
      * MQTT topic
    The node publishes the current state of the light (ON/OFF)
*/

/* Includes */
#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamString.h>
#include <DHTesp.h>
#include <Wire.h>
#include <BH1750.h>
#include "OTA_updater_ESP12.h" //Does not work well

/* Network settings */
const char* ssid = "";
const char* password = "";

/* MQTT settings */
const char* mqtt_server = "192.168.2.118";

/* Debugging */
#define DEBUG_ENABLED 1

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN D1
#define DHTTYPE DHT11   // DHT 11
#define LIGHT_SENSOR_PIN_1 D6
#define LIGHT_SENSOR_PIN_2 D7
#define DOOR_STATUS_PIN D2
#define MEASUREMENT_DELTA 0.01 //Minimum change in measurements (%) to publish the data

/* Ensure a minimum publish rate */
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every 10 minutes
#define NUM_PUBLISH_TOPICS 3
#define DEFAULT_POLLING_PERIOD 20000 //10 seconds
#define WARNING_POLLING_PERIOD 20000 //Currently not used

/* Logical states */
/* Signal smoothing filters */
/*
 * The signal smoothing filter is implemented as a circular buffer.
 * For simplicity, the last position of the buffer contains always the index
 * of the oldest element in the buffer.
 */
 #define BUFFER_SIZE 20
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];

/* Temperature measurement (average after filtering) */
//Improvement: create a struct
float temperature = 0;
float temperature_old = 99999; //Just a large initial value. Different from temperature

/* Humidity measurement (average after filtering) */
//Improvement: create a struct
float humidity = 0;
float humidity_old = 99999;

/* Light measurement */
float light_amount = 0;
float light_amount_old = 99999;

/* Period to enter the node_specific_loop */
uint32_t polling_period = DEFAULT_POLLING_PERIOD;
unsigned long last_iteration = 0;

/* Ensure a minimum publish rate */
unsigned long last_publish[NUM_PUBLISH_TOPICS];

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
DynamicJsonBuffer jsonBuffer(50);

/* List of subscribed topics */
#define NUM_SUBSCRIBED_TOPICS 0

String topic_subscribe_list[NUM_SUBSCRIBED_TOPICS] = {
};

/* OTA Web Server */
OTAUpdater_ESP12 updater;
const char* ota_url = "livingroom_node";

/* Door sensor */
volatile bool door_status_open = false;
bool door_status_open_old = false;

/* Temperature and humdity sensor */
DHTesp temperature_sensor;

/* Light sensor */
BH1750 lightMeter;

/* Setup */
void setup() {

#if DEBUG_ENABLED
  Serial.begin(115200);
#endif

  setup_wifi();
  setup_hardware();
  setup_mqtt();
  setup_OTA();

  last_publish[0] = 0; //Temperature
  last_publish[1] = 0; //Humidity
  last_publish[2] = 0; //Light

  /* Ensure that the circular buffer starts at the index 0 */
  temperature_buffer[BUFFER_SIZE] = 0;
  humidity_buffer[BUFFER_SIZE] = 0;

  /* ISR for door open/close detection */
  attachInterrupt(digitalPinToInterrupt(DOOR_STATUS_PIN), door_detection_isr, CHANGE);

  /* Publish a boot event for error tracking and debugging */
  bool publish_succeeded = false;
  while(!publish_succeeded)
  {
    publish_succeeded = client.publish("livingroom_node/boot", "");
    if(!publish_succeeded) network_loop();
    yield();
  }

  blink_led(50);
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);

#if DEBUG_ENABLED
  Serial.println("MQTT setup completed");
#endif
}

/* Connect to the wireless network */
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

#if DEBUG_ENABLED
   Serial.println("Wifi setup completed");
#endif
}

/* Configure the hardware */
void setup_hardware()
{
  /* Built in LED */
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, HIGH); //This means led OFF

 /* Initialize I2C bus */
  Wire.begin(LIGHT_SENSOR_PIN_1,LIGHT_SENSOR_PIN_2);
  lightMeter.begin();

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT11);
 pinMode(DOOR_STATUS_PIN, INPUT_PULLUP);


#if DEBUG_ENABLED
  Serial.println("Hardware setup completed");
#endif
}

/* Setup web server for Over The Air updates */
void setup_OTA()
{
  updater.begin(ota_url);
}

/* Subscribe to a defined list of topics */
void subscribe_topics()
{
  for(uint8_t i = 0; i < NUM_SUBSCRIBED_TOPICS; i++)
  {
    client.subscribe(topic_subscribe_list[i].c_str());
  }
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("LivingroomClient")) {
      Serial.println("connected");
      /* Resubscribe */
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  client.publish("livingroom_node/reconnect", "");
}

void blink_led(uint16_t delay_ms)
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
}

/* Handle the network part of the loop */
void network_loop()
{

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  /* OTA loop */
  updater.OTA_handle();
}

/* Average an array. Remove oldest element. Add newest element. Filter out NaN values */
float smoothing_filter(float* measurement_buffer, float measurement)
{
  float avg = 0;
  uint8_t measurement_counter = 0;

  //Get index of the oldest element, stored in position 0 of the buffer
  uint8_t oldest_idx = (uint8_t)(measurement_buffer[BUFFER_SIZE]);

  //Update index of the oldest element
  uint8_t next_idx;
  if( (oldest_idx+1) >= BUFFER_SIZE ) next_idx = 0;
  else next_idx = oldest_idx + 1;
  measurement_buffer[BUFFER_SIZE] = next_idx;

  //Replace oldest element by newest in the FIFO
  measurement_buffer[oldest_idx] = measurement;

  //Average the buffer skipping NaN values, unless ALL are NaNs
  for(uint8_t i=0; i<BUFFER_SIZE; i++)
  {
    float val = measurement_buffer[i];
    if(!isnan(val) && (val != 0))
    {
      avg += val;
      measurement_counter++;
    }
  }

  if(measurement_counter == 0) avg = NAN;
  else avg /= measurement_counter;

  //Return averaged value
#if DEBUG_ENABLED
  Serial.print("Averaged value: ");
  Serial.println(avg);
  Serial.print("Filtered values: ");
  Serial.println(measurement_counter);
#endif
  return avg;
}

/* Get the amount of light */
void get_light_amount()
{
  light_amount = lightMeter.readLightLevel();

#if DEBUG_ENABLED
  Serial.print("Light amount: ");
  Serial.println(light_amount);
#endif
}

/* Get temperature and humidity */
void get_temperature()
{
  /* Instant measurement */
  float tmp_humidity = temperature_sensor.getHumidity();
  float tmp_temperature = temperature_sensor.getTemperature();

  /* Filter and average */
  temperature = smoothing_filter(temperature_buffer,tmp_temperature);
  humidity = smoothing_filter(humidity_buffer,tmp_humidity);

#if DEBUG_ENABLED
  Serial.print("Instant temperature: ");
  Serial.println(tmp_temperature);
  Serial.print("Averaged temperature: ");
  Serial.println(temperature);
  Serial.print("Instant humidity: ");
  Serial.println(tmp_humidity);
  Serial.print("Averaged humidity: ");
  Serial.println(humidity);
#endif
}

/* Get the status of the door */
void get_door_status()
{
  /* Poll sensor */
  int polled_value = digitalRead(DOOR_STATUS_PIN);

  /* Save old status for change detection */
  door_status_open_old = door_status_open;

  if(polled_value)
  {
    door_status_open = true;
  }
  else door_status_open = false;

  #if DEBUG_ENABLED
  Serial.print("Door detection: ");
  Serial.println(polled_value);
  #endif
}

void door_detection_isr()
{
  last_iteration = 0;
  #if DEBUG_ENABLED
  Serial.println("Door detected via interrupt");
  #endif
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get temperature and humidity */
  get_temperature();

  /* Get the amount of light */
  get_light_amount();

  /* Get door status */
  get_door_status();
}

/* Node-specific logic */
void node_logic()
{

}

/* Publish sensor information */
void publish_status()
{
#if DEBUG_ENABLED
  Serial.println("Publishing messages");
#endif

/* Compute time since last publish for each topic */
  unsigned long time_since_publish;
  bool publish_this[NUM_PUBLISH_TOPICS];
  for(uint8_t i=0; i<NUM_PUBLISH_TOPICS; i++)
  {
    time_since_publish = millis() - last_publish[i];
    if(time_since_publish > MINIMUM_PUBLISH_RATE) publish_this[i] = true;
    else publish_this[i] = false;
  }

  /* Publish temperature */
  if( (abs(temperature - temperature_old) > abs(MEASUREMENT_DELTA*temperature)) || publish_this[0])
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in temperature larger than 1%. Publishing temperature");
    #endif
    client.publish("livingroom_node/temperature", String(temperature).c_str() );
    temperature_old = temperature;
    last_publish[0] = millis();
  }

  /* Publish humidity */
  if( (abs(humidity - humidity_old) > abs(MEASUREMENT_DELTA*humidity)) || publish_this[1] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in humidity larger than 1%. Publishing humidity");
    #endif
    client.publish("livingroom_node/humidity", String(humidity).c_str() );
    humidity_old = humidity;
    last_publish[1] = millis();
  }

  /* Publish light amount */
  if( (abs(light_amount - light_amount_old) > abs(MEASUREMENT_DELTA*light_amount)) || publish_this[2] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in light amount larger than 1%. Publishing light amount");
    #endif

    String msg = "{\"light\":\"" + String(light_amount) + "\"}";

    client.publish("livingroom_node/light", msg.c_str());
    light_amount_old = light_amount;
    last_publish[2] = millis();
  }

  /* Publish door open information with status update */
  if(door_status_open != door_status_open_old)
  {
#if DEBUG_ENABLED
    Serial.print("Change in door status: ");
    Serial.println(door_status_open);
#endif

    /* Ensure that the critical topics get published */
    bool publish_succeeded = false;
    while(!publish_succeeded)
    {
      publish_succeeded = client.publish("livingroom_node/door_status", String(door_status_open).c_str());
      if(!publish_succeeded) network_loop();
      yield();
    }
  }
}

/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
 */
void node_specific_loop()
{
  /* Sensor polling */
  poll_sensors();

  /* Internal logic */
  node_logic();

  /* Publish sensor information */
  publish_status();
}

void loop() {

  /* Connection handling */
  network_loop();

  long now = millis();
  if( (now - last_iteration) > polling_period )
  {
    /* Node specific loop */
    node_specific_loop();

    /* Update last iteration time */
    last_iteration = now;
  }
}
