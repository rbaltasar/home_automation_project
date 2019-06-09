/*
  Home automation - <node name>

  Data gathering:
    

  Data publishing:
   

  Topic subscription:
    
  Node description:    

*/

/* Includes */
#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamString.h>

/* Network settings */
const char* ssid = "";
const char* password = "";

/* MQTT settings */
const char* mqtt_server = "";

/* Sinric settings - Only if Alexa compatible device is emulated*/
#define API_KEY ""
#define DEVICE_ID ""
#define SERVER_URL "iot.sinric.com"
#define SERVER_PORT 80
#define HEARTBEAT_INTERVAL 300000 // 5 Minutes

/* Debugging */
#define DEBUG_ENABLED 0

/* Hardware settings */
#define SENSOR_A D1
#define SENSOR_B D2
#define MEASUREMENT_DELTA 0.01 //Minimum change in measurements (%) to publish the data. Reduce network load

/* Ensure a minimum publish rate */
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every 10 minutes
#define NUM_PUBLISH_TOPICS <number of topics to publish>
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
float <your_buffer>[BUFFER_SIZE+1];

/* Global variables */


/* Web server states --- Sinric */
uint64_t heartbeatTimestamp = 0;
bool isConnected = false;
void setPowerStateOnServer(String deviceId, String value);

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
WebSocketsClient webSocket;
DynamicJsonBuffer jsonBuffer(50); //Only if needed

/* Global objects (e.g: sensors, actuators...) */

/* Setup */
void setup() {

#if 1
  Serial.begin(115200);
#endif

  setup_wifi();
  setup_sinric();
  setup_hardware();
  setup_mqtt();

  /* Ensure that the circular buffer starts at the index 0 */
  <your_buffer>[BUFFER_SIZE] = 0;

  /* Publish a boot event for error tracking and debugging */
  bool publish_succeeded = false;
  while(!publish_succeeded)
  {
    publish_succeeded = client.publish("<node_name>/boot", "");
    if(!publish_succeeded) network_loop();
    yield();
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
  client.subscribe("<node_name>/<topic_name>");
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

#if DEBUG_ENABLED
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
   Serial.println("Wifi setup completed");
#endif
}

/* Configure SINRIC */
void setup_sinric()
{
  //Server address, port and URL
  webSocket.begin("iot.sinric.com", 80, "/");

  //Event handler
  webSocket.onEvent(webSocketEvent);
  webSocket.setAuthorization("apikey", API_KEY);

  //Reconnection interval
  webSocket.setReconnectInterval(5000);

#if DEBUG_ENABLED
   Serial.println("Sinric setup completed");
#endif
}

/* Configure the hardware */
void setup_hardware()
{
  /* Built in LED */

  /* Setup pins */
 

#if DEBUG_ENABLED
   Serial.println("Hardware setup completed");
#endif
}

/* Configure the callback function for a subscribed MQTT topic */
void callback(char* topic, byte* payload, unsigned int length) {

  /* Print message (debugging only) */
#if DEBUG_ENABLED
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
  
  /* Jump into the node-specific loop right at the next loop iteration */
  last_iteration = 0;
}

/* Reconnect to the MQTT broker */
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("<node_name>"))
    {
      Serial.println("connected");
      //Resubscribe
      client.subscribe("<node_name>/<topic_name>");
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

/* Handle the network part of the loop */
void network_loop()
{
#if DEBUG_ENABLED
  unsigned long current_time = millis();
#endif

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  /* WebSocket loop */
  webSocket.loop();

  if(isConnected)
  {
    uint64_t now = millis();
    // Send heartbeat in order to avoid disconnections during ISP resetting IPs over night. Thanks @MacSass
    if((now - heartbeatTimestamp) > HEARTBEAT_INTERVAL)
    {
      heartbeatTimestamp = now;
      webSocket.sendTXT("H");
    }
  }

#if 0
  current_time = millis() - current_time;
  Serial.print("Duration network_loop: ");
  Serial.println(current_time);
#endif
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
#if DEBUG_ENABLED
  Serial.print("Averaged value: ");
  Serial.println(avg);
  Serial.print("Filtered values: ");
  Serial.println(measurement_counter);
#endif
  return avg;
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Poll the sensors */

  /* Store value in memory */

}

/* Node-specific logic */
void node_logic()
{
  
}

/* Publish sensor information */
void publish_status()
{

}

/* Handle WebSocket event */
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type)
  {
    case WStype_DISCONNECTED:
    {
      isConnected = false;
      Serial.printf("[webSocketEvent] Webservice disconnected from server!\n");
      break;
    }
    case WStype_CONNECTED:
    {
      isConnected = true;
      Serial.printf("[webSocketEvent] Service connected to server at url: %s\n", payload);
      Serial.printf("[webSocketEvent] Waiting for commands from server ...\n");
      break;
    }
    case WStype_TEXT:
    {
      Serial.printf("[webSocketEvent] get text: %s\n", payload);
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.parseObject((char*)payload);
      String deviceId = json ["deviceId"];
      String action = json ["action"];

      //Filter events not addressed to this device
      if(deviceId != DEVICE_ID) return;

      if(action == "setPowerState")
      {
        
      }
      else if(action == "AdjustBrightness")
      {
        
      }
      else if(action == "SetBrightness")
      {
        
      }
      else if(action == "SetColor")
      {
        double hue, saturation, brightness;
        hue = json ["value"]["hue"];
        saturation = json ["value"]["saturation"];
        brightness = json ["value"]["brightness"];        
      }
      else if(action == "IncreaseColorTemperature")
      {
        
      }
      else if(action == "IncreaseColorTemperature")
      {
        
      }
      else if(action == "SetColorTemperature")
      {
        //This request is equivalent to set to White        
      }

      break;
    }
    case WStype_BIN:
    {
      Serial.printf("[webSocketEvent] get binary length: %u\n", length);
      break;
    }
  }
}


/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
 */
void node_specific_loop()
{
#if DEBUG_ENABLED
  /* Performance measurement */
  unsigned long current_time = millis();
#endif

  /* Sensor polling */
  poll_sensors();

  /* Internal logic */
  node_logic();

  /* Publish sensor information */
  publish_status();

#if DEBUG_ENABLED
  current_time = millis() - current_time;
  Serial.print("Duration node loop: ");
  Serial.println(current_time);
#endif
}

void loop() {

  /* Connection handling */
  network_loop();

  /* Enter the node_specific_loop with the specified frequency */
  long now = millis();
  if( (now - last_iteration) > polling_period )
  {
    /* Node specific loop */
    node_specific_loop();

    /* Update last iteration time */
    last_iteration = now;
  }
}
