/*
  Home automation - bedroom node

  Data gathering:
    Temperature & humidity
    Window state

  Data publishing:
    Temperature & humidity

  Topic subscription:
    bedroom_node/light_state
    bedroom_node/light_color
    bedroom_node/light_intensity


  Node description:
    The node gathers and publishes temperature and humidity information
    The node receives light requests in three different ways:
      * MQTT topic
      * WebSocket request (Sinric --> Alexa)
      * By claping twice (only ON/OFF switch)
    The node publishes the state of the window

*/

/* Includes */
#include <WebSocketsClient.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamString.h>
#include <DHTesp.h>
#include "common_datatypes.h"
#include "frequency_utilities.h"
#include "clapDetection.h"
#include "LED_controller.h"
#include "OTA_updater_ESP32.h"

/* Network settings */
const char* ssid = "";
const char* password = "";

/* MQTT settings */
const char* mqtt_server = "192.168.2.118";

/* Debugging */
#if (DEBUG_TRACES_CLAP == 1)
#include "serialFreqDisplay.h" //Only for frequency debugging
#endif

/* Logical states */
/* Signal smoothing filters */
/*
 * The signal smoothing filter is implemented as a circular buffer.
 * For simplicity, the last position of the buffer contains always the index
 * of the oldest element in the buffer.
 */
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];

/* Temperature measurement (average after filtering) */
state_tracker<float> temperature;

/* Humidity measurement (average after filtering) */
state_tracker<float> humidity;

/* Window state variables */
state_tracker<bool> door_status_open;

/* Enable clap detection */
bool enable_clap = true;

/* Period to enter the node_specific_loop */
uint32_t polling_period = DEFAULT_POLLING_PERIOD;
unsigned long last_iteration = 0;

/* Ensure a minimum publish rate */
unsigned long last_publish[NUM_PUBLISH_TOPICS];

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
WebSocketsClient webSocket;
DynamicJsonBuffer jsonBuffer(250);
unsigned long last_alive_tx = 0;
unsigned long last_alive_rx = 0;
uint8_t mqtt_reconnect_counter = 0;


/* List of subscribed topics */
#define NUM_SUBSCRIBED_TOPICS 8

String topic_subscribe_list[NUM_SUBSCRIBED_TOPICS] = {
  "bedroom_node/mode_request",
  "bedroom_node/light_intensity",
  "bedroom_node/light_color",
  "bedroom_node/effect_delay",
  "bedroom_node/effect_speed",
  "bedroom_node/alive_rx",
  "bedroom_node/initcommrx",
  "bedroom_node/enable_clap"
};

/* Temperature and humidity sensor */
DHTesp temperature_sensor;

/* OTA settings */
OTAUpdater_ESP32 updater;
String IPAddress_string;
String MACAddress_string;
const char* ota_url = "bedroom_node";

/* LED Controller */
state_tracker<lamp_status> lamp_state;
LEDController LED_controller(&lamp_state.val);

/* Audio signal spectrum display */
#if (DEBUG_TRACES_CLAP == 1)
//SerialFreqDisplay displ(THRESHOLD, NSAMPLES/2);
#endif

/* Audio signal analyzer */
FrequencyUtilities FreqUtilities;

/* Clap detection features */
clapDetection clap_detector(&FreqUtilities);

/* System state*/
system_state_var sysState;
init_struct initState;

/* ------------------------- */

/* Setup */
void setup() {

#if 1
  Serial.begin(115200);
#endif

  setup_wifi();
  delay(100);
  setup_mqtt();
  delay(100);
  setup_hardware();

  /* ISR for window Open/Closed detection */
  attachInterrupt(digitalPinToInterrupt(DOOR_SENSOR_PIN), door_detection_isr, CHANGE);

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

  /* Share the serial printer object for debugging functionalities */
  HardwareSerial* hwPrint;
  hwPrint = &Serial;
  FreqUtilities.begin(hwPrint);

}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ;
}

#if 0
/* Print reset reason */
void print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}

/* Get the reset reason for each core */
void get_reset_reason()
{
  reset_reason_0 = rtc_get_reset_reason(0);
  reset_reason_1 = rtc_get_reset_reason(1);

  Serial.println("CPU0 reset reason:");
  print_reset_reason(reset_reason_0);

  Serial.println("CPU1 reset reason:");
  print_reset_reason(reset_reason_0);
}
#endif

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

#if  (DEBUG_TRACES == 1)
  Serial.println("MQTT setup completed");
#endif
}

/* Setup Over The Air updates */
void setup_OTA()
{
  updater.begin(ota_url);
}

/* Connect to the wireless network */
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint8_t try_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    try_counter++;
    if(try_counter > 10) ESP.restart();
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

#if  (DEBUG_TRACES == 1)
   Serial.println("Wifi setup completed");
#endif
}


/* Configure the hardware */
void setup_hardware()
{
  /* Built in LED */
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, LOW); //This means led OFF

  /* Door sensor */
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT22);

 /* LED initialization */
 LED_controller.setup();

#if  (DEBUG_TRACES == 1)
  Serial.println("Hardware setup completed");
#endif
}

/* Configure the callback function for a subscribed MQTT topic */
void callback(char* topic, byte* payload, unsigned int length) {

  /* Print message (debugging only) */
#if  (DEBUG_TRACES == 1)
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  /* Parse JSON object */
  JsonObject& root = jsonBuffer.parseObject(payload);

  /* Filter for topics */
  if( strcmp(topic,"bedroom_node/mode_request") == 0 )
  {
    lamp_state.val.lamp_mode = root["mode"];
    Serial.println(lamp_state.val.lamp_mode);
  }

  else if(strcmp(topic,"bedroom_node/light_intensity") == 0)
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

  else if(strcmp(topic,"bedroom_node/effect_delay") == 0)
  {
    int rcv = root["delay"];
    lamp_state.val.effect_amount = rcv;
    rcv = rcv * 10;
    lamp_state.val.effect_delay = rcv; //Delay in ms
    Serial.println(rcv);
  }

  else if(strcmp(topic,"bedroom_node/effect_speed") == 0)
  {

    int rcv = root["speed"];
    rcv = 1000 - 10 * rcv;
    lamp_state.val.effect_speed = rcv; //Delay in ms
    Serial.println(rcv);
  }

  else if(strcmp(topic,"bedroom_node/light_color") == 0)
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

  else if(strcmp(topic,"bedroom_node/initcommrx") == 0)
  {
    const char* mac_request = root["mac_origin"];

    if(strcmp(mac_request,MACAddress_string.c_str()) == 0)
    {
      ota_url = root["OTA_URL"];
      lamp_state.val.lamp_mode = root["mode"];
    }

    initState.isCompleted = true;
  }
  else if(strcmp(topic,"bedroom_node/enable_clap") == 0)
  {
    int rcv = root["enable_clap"];

    if(rcv == 1) enable_clap = true;
    else enable_clap = false;
  }

  else if(strcmp(topic,"bedroom_node/alive_rx") == 0)
  {
    last_alive_rx = millis();
  }
}

/* Reconnect to the MQTT broker */
void reconnect()
{
  // Loop until we're reconnected

  if (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("BedroomClient"))
    {
      Serial.println("connected");
      //Resubscribe
      subscribe_topics();
      mqtt_reconnect_counter = 0;
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 100 ms");
      // Wait 5 seconds before retrying
      delay(100);
      if(++mqtt_reconnect_counter > 10)  ESP.restart();
    }

  }

}

/* Handle the network part of the loop */
void network_loop()
{

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  /* Alive check */
  unsigned long now = millis();

  if( (now - last_alive_tx)> ALIVE_PERIOD)
  {
    client.publish("bedroom_node/alive_tx", "");
    last_alive_tx = now;
  }

  if( (now - last_alive_rx)> (3*ALIVE_PERIOD))
  {
    Serial.println("Lost MQTT connection. Reboot");
    ESP.restart();
  }

}

/* Average an array. Remove oldest element. Add newest element. Filter out NaN values */
float smoothing_filter(float* measurement_buffer, float measurement)
{

  float avg = 0;
  uint8_t measurement_counter = 0;

  //Get index of the oldest element, stored in last position of the buffer
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
#if  (DEBUG_TRACES == 1)
  Serial.print("Averaged value: ");
  Serial.println(avg);
  Serial.print("Filtered values: ");
  Serial.println(measurement_counter);
#endif
  return avg;
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

#if (DEBUG_TRACES == 1)
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

/* Get the status of the door */
void get_door_status()
{
  /* Poll sensor */
  int polled_value = digitalRead(DOOR_SENSOR_PIN);

  /* Save old status for change detection */
  door_status_open.old = door_status_open.val;

  if(polled_value)
  {
    door_status_open.val = true;
  }
  else door_status_open.val = false;

  #if  (DEBUG_TRACES == 1)
  Serial.print("Window state : ");
  Serial.println(polled_value);
  #endif
}

/* Window state change ISR */
void door_detection_isr()
{
  /* Jump directly into the node_specific_loop in the next loop iteration */
  last_iteration = 0;
  #if  (DEBUG_TRACES == 1)
  Serial.println("Window state change detected via interrupt");
  #endif
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get temperature and humidity */
  get_temperature();

  /* Get window state */
  get_door_status();
}

void publish_mode(int lamp_mode)
{
  /* Ensure that the critical topics get published */
  client.publish("bedroom_node/lamp_mode_feedback", String(lamp_mode).c_str());
}

/* Node-specific logic */
void node_logic()
{
  /* Process double clap detected --> Switch ON/OFF */
  if(clap_detector.getDetectionType() == DOUBLE_CLAP)
  {
    if(lamp_state.val.lamp_mode) lamp_state.val.lamp_mode = 0;
    else lamp_state.val.lamp_mode = 1;
    clap_detector.resetDetectionType();
    /* Publish mode to update Dashboard */
    publish_mode(lamp_state.val.lamp_mode);
  }
  /* Process triple clap detected --> Change color */
  else if(clap_detector.getDetectionType() == TRIPLE_CLAP)
  {
    if(lamp_state.val.lamp_mode == 1) lamp_state.val.lamp_mode = 10;
    else if(lamp_state.val.lamp_mode > 9)
    {
      if(++lamp_state.val.lamp_mode > 25) lamp_state.val.lamp_mode = 10;
    }

    clap_detector.resetDetectionType();

    /* Publish mode to update Dashboard */
    publish_mode(lamp_state.val.lamp_mode);
  }
}

/* Publish sensor information */
void publish_status()
{
#if  (DEBUG_TRACES == 1)
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
  if( (abs(temperature.val - temperature.old) > abs(MEASUREMENT_DELTA*temperature.val)) || publish_this[0] )
  {
    #if  (DEBUG_TRACES == 1)
    Serial.println("Detected change in temperature larger than 1%. Publishing temperature");
    #endif
    client.publish("bedroom_node/temperature", String(temperature.val).c_str() );
    temperature.old = temperature.val;
    last_publish[0] = millis();
  }

  /* Publish humidity */
  if( (abs(humidity.val - humidity.old) > abs(MEASUREMENT_DELTA*humidity.val)) || publish_this[1] )
  {
    #if  (DEBUG_TRACES == 1)
    Serial.println("Detected change in humidity larger than 1%. Publishing humidity");
    #endif
    client.publish("bedroom_node/humidity", String(humidity.val).c_str() );
    humidity.old = humidity.val;
    last_publish[1] = millis();
  }

  /* Publish door open information with status update */
  if(door_status_open.val != door_status_open.old)
  {
#if  (DEBUG_TRACES == 1)
    Serial.print("Change in door status: ");
    Serial.println(door_status_open.val);
#endif

    /* Ensure that the critical topics get published */
    client.publish("bedroom_node/door_status", String(door_status_open.val).c_str());
  }
}

/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
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
    if(enable_clap) node_logic();

    /* Publish sensor information */
    publish_status();

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
      //lamp_state.val.color.R = RGB_DEFAULT;
      //lamp_state.val.color.G = RGB_DEFAULT;
      //lamp_state.val.color.B = RGB_DEFAULT;
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
  StaticJsonBuffer<128> jsonBuffer_send;
  JsonObject& root_send = jsonBuffer_send.createObject();

  root_send["mac"] = MACAddress_string.c_str();
  root_send["ip"] = IPAddress_string.c_str();

  char JSONmessageBuffer[256];
  root_send.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

  client.publish("bedroom_node/initcomm_tx", JSONmessageBuffer);
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

      client.unsubscribe("bedroom_node/initcommrx");

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

void loop()
{

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
      /* Clap detection state machine */
      if(enable_clap)
      {
        if(clap_detector.feed()) last_iteration = 0;
      }
      break;
    default:
    break;
  }
}
