/*
  Home automation - terrace node

  Data gathering:
    Temperature & humidity
    Light amount

  Data publishing:
    Temperature & humidity
    Light amount

  Topic subscription:
    Led light control

  Node description:
    The node gathers and publishes temperature, humidity and light amount
    The impements a LED control with static effects
*/

#include <rom/rtc.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <Wire.h>
#include <BH1750.h>
#include <ArduinoJson.h>
#include "common_datatypes.h"
#include "LED_controller.h"
#include "OTA_updater_ESP32.h"
#include "config.h"

/* Network settings */
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.2.118";

/* Logical states */
/* Signal smoothing filters */
/*
 * The signal smoothing filter is implemented as a circular buffer.
 * For simplicity, the last position of the buffer contains always the index
 * of the oldest element in the buffer.
 */
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];

/* Light measurement (no averaging needed) */
state_tracker<float> light_amount;

/* Temperature measurement (average after filtering) */
state_tracker<float> temperature;

/* Humidity measurement (average after filtering) */
state_tracker<float> humidity;

/* Period to enter the node_specific_loop */
uint32_t polling_period = DEFAULT_POLLING_PERIOD;
long last_iteration = 0;

/* Ensure a minimum publish rate */
unsigned long last_publish[NUM_PUBLISH_TOPICS];

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long last_alive_tx = 0;
unsigned long last_alive_rx = 0;

/* List of subscribed topics */
#define NUM_SUBSCRIBED_TOPICS 7

String topic_subscribe_list[NUM_SUBSCRIBED_TOPICS] = {
  "terrace_node/mode_request",
  "terrace_node/light_intensity",
  "terrace_node/light_color",
  "terrace_node/effect_delay",
  "terrace_node/effect_speed",
  "terrace_node/alive_rx",
  "terrace_node/initcommrx"
};

/* OTA settings */
OTAUpdater_ESP32 updater;
String IPAddress_string;
String MACAddress_string;
const char* ota_url;

/* Temperature and humidity sensor */
DHTesp temperature_sensor;

/* Light sensor */
BH1750 lightMeter(0x23);

/* LED status */
state_tracker<lamp_status> lamp_state;

/* LED Controller */
LEDController LED_controller(&lamp_state.val);

/* System state*/
system_state_var sysState;
init_struct initState;

/* Setup */
void setup()
{
#if 1
  Serial.begin(115200);
#endif

  setup_wifi();
  delay(100);
  setup_mqtt();
  delay(100);
  setup_hardware();

  /* Initial LED configuration */
  /* Initial configuration of the lamp when the system is booted */
  lamp_state.val.color.R = 0;
  lamp_state.val.color.G = RGB_DEFAULT;
  lamp_state.val.color.B = 0;
  lamp_state.val.brightness = 1;
  lamp_state.val.effect_delay = 50;
  lamp_state.val.effect_speed = 50;
  lamp_state.val.streaming = false;
  lamp_state.val.effect_amount = 1;

  lamp_state.old = lamp_state.val;

  sysState = STARTUP;

  /* Configuration state */
  initState.hasStarted = false;
  initState.isCompleted = false;

  /* Setup finished. Show leds */
  LED_controller.setLeds(lamp_state.val.color,0,NUM_LEDS/3);

  /* Ensure that the circular buffer starts at the index 0 */
  temperature_buffer[BUFFER_SIZE] = 0;
  humidity_buffer[BUFFER_SIZE] = 0;

}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ;
}

/* Subscribe to a defined list of topics */
void subscribe_topics()
{
  for(uint8_t i = 0; i < NUM_SUBSCRIBED_TOPICS; i++)
  {
    client.subscribe(topic_subscribe_list[i].c_str());
  }
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  client.setCallback(callback);
  /* Subscribe to topics */
  subscribe_topics();

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

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  MACAddress_string = WiFi.macAddress();

  /* Translate the IP address to String to have a unique name for MQTT client */
  IPAddress_string = IpAddress2String(WiFi.localIP());


#if DEBUG_ENABLED
   Serial.println("Wifi setup completed");
#endif
}

/* Configure the hardware */
void setup_hardware()
{

  /* In-built LED */
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, HIGH); //This means led OFF

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT22);

 /* Initialize I2C bus */
  Wire.begin();
  lightMeter.begin();

 /* LED initialization */
 LED_controller.setup();

#if DEBUG_ENABLED
  Serial.println("Hardware setup completed");
#endif
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TerraceClient"))
    {
      Serial.println("connected");
      subscribe_topics();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_OTA()
{
  updater.begin(ota_url);
}

/* Configure the callback function for a subscribed MQTT topic */
void callback(char* topic, byte* payload, unsigned int length) {

  /* Print message (debugging only) */
#if 1
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  /* Parse JSON object */
  StaticJsonDocument<256> root;
  DeserializationError error = deserializeJson(root, payload);

  /* Test if parsing succeeded */
  if (error) {
    Serial.print("deserializeMsgPack() failed: ");
    Serial.println(error.c_str());
  }

  /* Filter for topics */
  if( strcmp(topic,"terrace_node/mode_request") == 0 )
  {
    lamp_state.val.lamp_mode = root["mode"];
    Serial.println(lamp_state.val.lamp_mode);
  }

  else if(strcmp(topic,"terrace_node/light_intensity") == 0)
  {
    int rcv = root["intensity"];

    if(rcv == 0) rcv = 255;

    else
    {
      rcv = 11 - rcv;
    }

    lamp_state.val.brightness = rcv;
    Serial.println(rcv);
  }

  else if(strcmp(topic,"terrace_node/effect_delay") == 0)
  {
    int rcv = root["delay"];
    lamp_state.val.effect_amount = rcv;
    rcv = rcv * 10;
    lamp_state.val.effect_delay = rcv; //Delay in ms
    Serial.println(rcv);
  }

  else if(strcmp(topic,"terrace_node/effect_speed") == 0)
  {

    int rcv = root["speed"];
    rcv = 1000 - 10 * rcv;
    lamp_state.val.effect_speed = rcv; //Delay in ms
    Serial.println(rcv);
  }

  else if(strcmp(topic,"terrace_node/light_color") == 0)
  {

    lamp_state.val.color.R = root["R"];
    lamp_state.val.color.G = root["G"];
    lamp_state.val.color.B = root["B"];

    // Output to serial monitor
#if 1
    Serial.println(lamp_state.val.color.R);
    Serial.println(lamp_state.val.color.G);
    Serial.println(lamp_state.val.color.B);
#endif
  }

  else if(strcmp(topic,"terrace_node/initcommrx") == 0)
  {
    const char* mac_request = root["mac_origin"];

    if(strcmp(mac_request,MACAddress_string.c_str()) == 0)
    {
      ota_url = root["OTA_URL"];
      lamp_state.val.lamp_mode = root["mode"];
    }

    initState.isCompleted = true;
  }
  else if(strcmp(topic,"terrace_node/alive_rx") == 0)
  {
    last_alive_rx = millis();
  }
}

/* Handle the network part of the loop */
/* The network stuff works atomically */
void network_loop()
{

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  if( (now - last_alive_tx)> ALIVE_PERIOD)
  {
    client.publish("terrace_node/alive_tx", "");
    last_alive_tx = now;
  }

  if( (now - last_alive_rx)> (3*ALIVE_PERIOD))
  {
    Serial.println("Lost MQTT connection. Reboot");
    ESP.restart();
  }
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
  light_amount.val = lightMeter.readLightLevel();

#if DEBUG_ENABLED
  Serial.print("Light amount: ");
  Serial.println(light_amount.val);
#endif
}

/* Get temperature and humidity */
void get_temperature()
{
  /* Instant measurement */
  float tmp_humidity = temperature_sensor.getHumidity();
  float tmp_temperature = temperature_sensor.getTemperature();

  /* Filter and average */
  temperature.val = smoothing_filter(temperature_buffer,tmp_temperature);
  humidity.val = smoothing_filter(humidity_buffer,tmp_humidity);

#if DEBUG_ENABLED
  Serial.print("Instant temperature: ");
  Serial.println(tmp_temperature);
  Serial.print("Averaged temperature: ");
  Serial.println(temperature.val);
  Serial.print("Instant humidity: ");
  Serial.println(tmp_humidity);
  Serial.print("Averaged humidity: ");
  Serial.println(humidity.val);
#endif
}

/* Update the polling period depending on the status of the system */
/* If the average measurement after filtering is NaN, reduce the polling period */
/* This feature is currently not used */
void update_polling_period()
{
  if(isnan(temperature.val) || isnan(humidity.val))
  {
    #if DEBUG_ENABLED
    Serial.println("Polling period in WARNING mode");
    #endif
    polling_period = WARNING_POLLING_PERIOD;
  }
  else
  {
    #if DEBUG_ENABLED
    Serial.println("Polling period in DEFAULT mode");
    #endif
    polling_period = DEFAULT_POLLING_PERIOD;
  }
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get temperature and humidity */
  get_temperature();

  /* Get the amount of light (only if request received) */
  get_light_amount();

}


/* Node-specific logic */
/* Empty. Keep for future improvements/features */
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

  /* Publish temperature*/
  if( (abs(temperature.val - temperature.old) > abs(MEASUREMENT_DELTA*temperature.val)) || publish_this[0] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in temperature larger than 1%. Publishing temperature");
    #endif
    client.publish("terrace_node/temperature", String(temperature.val).c_str() );
    temperature.old = temperature.val;
    last_publish[0] = millis();
  }

  /* Publish humidity */
  if( (abs(humidity.val - humidity.old) > abs(MEASUREMENT_DELTA*humidity.val)) || publish_this[1] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in humidity larger than 1%. Publishing humidity");
    #endif
    client.publish("terrace_node/humidity", String(humidity.val).c_str() );
    humidity.old = humidity.val;
    last_publish[1] = millis();
  }

  /* Publish light amount */
  if( (abs(light_amount.val - light_amount.old) > abs(MEASUREMENT_DELTA*light_amount.val)) || publish_this[2] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in light amount larger than 1%. Publishing light amount");
    #endif
    client.publish("terrace_node/light", String(light_amount.val).c_str() );
    light_amount.old = light_amount.val;
    last_publish[2] = millis();
  }
}

/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
 * This loop is executed atomically
 */
void node_specific_loop()
{
  /* Enter the node_specific_loop with the specified frequency */
  long now = millis();
  if( (now - last_iteration) > polling_period )
  {
    /* Sensor polling */
    poll_sensors();

    /* Internal logic */
    node_logic();

    /* Publish sensor information */
    publish_status();

    /* Update last iteration time */
    last_iteration = now;
  }
}

void status_update()
{

  /* Check difference in mode request */
  if(lamp_state.val.lamp_mode != lamp_state.old.lamp_mode)
  {
    /* Finish previous effect */
    LED_controller.end_effect();

    /* Streaming request */
    if(lamp_state.val.lamp_mode == 1)
    {
      Serial.println("ON request received");
      lamp_state.val.color.R = RGB_DEFAULT;
      lamp_state.val.color.G = RGB_DEFAULT;
      lamp_state.val.color.B = RGB_DEFAULT;
      lamp_state.val.effect_delay = 50;
      lamp_state.val.effect_speed = 50;
      lamp_state.val.streaming = false;
    }

    Serial.print("Received change request to mode ");
    Serial.println(lamp_state.val.lamp_mode);

    lamp_state.old.lamp_mode = lamp_state.val.lamp_mode;
    LED_controller.update_mode();
  }

  else if(lamp_state.val.brightness != lamp_state.old.brightness)
  {
    Serial.print("Received change request to brightness level ");
    Serial.println(lamp_state.val.brightness);

    lamp_state.old.brightness = lamp_state.val.brightness;
    LED_controller.update_mode();
  }

  else if(lamp_state.val.color.R != lamp_state.val.color.R || lamp_state.val.color.G != lamp_state.old.color.G || lamp_state.val.color.B != lamp_state.old.color.B)
  {
    Serial.print("Received change request to color: ");
    Serial.println(lamp_state.val.color.R);
    Serial.println(lamp_state.val.color.G);
    Serial.println(lamp_state.val.color.B);

    lamp_state.old.brightness = lamp_state.val.brightness;

    LED_controller.update_mode();

    lamp_state.old.color.R = lamp_state.val.color.R;
    lamp_state.old.color.G = lamp_state.val.color.G;
    lamp_state.old.color.B = lamp_state.val.color.B;
  }
}

void publish_initcomm()
{
  StaticJsonDocument<256> root_send;

  root_send["mac"] = MACAddress_string.c_str();
  root_send["ip"] = IPAddress_string.c_str();

  char JSONmessageBuffer[256];
  serializeJson(root_send, JSONmessageBuffer);

  client.publish("terrace_node/initcomm_tx", JSONmessageBuffer);
}

void initComm()
{
  /* Send a MQTT request */
  if(!initState.hasStarted)
  {
    LED_controller.setLeds(lamp_state.val.color,0,(NUM_LEDS*2)/3);

    publish_initcomm();

    initState.hasStarted = true;
    initState.elapsed_time = millis();
  }

  /* Wait asynchronously for the answer */
  else if(initState.hasStarted)
  {
    /* Check if answer was received */
    if(initState.isCompleted)
    {

      client.unsubscribe("terrace_node/initcommrx");

      sysState = NORMAL;
      setup_OTA();

      delay(100);

      LED_controller.setAllLeds(lamp_state.val.color,0);

      lamp_state.val.color.R = RGB_DEFAULT;
      lamp_state.val.color.G = RGB_DEFAULT;
      lamp_state.val.color.B = RGB_DEFAULT;

      return;
    }
    /* Timeout. Show error and reset */
    else if( (millis() - initState.elapsed_time) > initState.timeout )
    {
      lamp_state.val.color.R = RGB_DEFAULT;
      lamp_state.val.color.G = 0;
      lamp_state.val.color.B = 0;
      LED_controller.setAllLeds(lamp_state.val.color,0);

      Serial.println("Error in communication setup. Restarting ESP32");

      ESP.restart();
    }
  }
}

void loop() {

  switch(sysState)
  {
    case STARTUP:
      network_loop();
      initComm();
      break;

    case NORMAL:
      network_loop();
      updater.OTA_handle();
      status_update();
      LED_controller.feed();
      node_specific_loop();
      break;
    default:
    break;
  }
}
