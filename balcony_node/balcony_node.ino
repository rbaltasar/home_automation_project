/*
  Home automation - balcony node

  Data gathering:
    Door status

  Data publishing:
    Door status

  Logic:

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* Network settings */
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.2.118";

/* Debugging */
#define DEBUG_ENABLED 0

/* Hardware settings */
#define DOOR_STATUS_PIN D2

/* Sleep time to reduce power consumption */
#define SLEEP_TIME 5000

/* Logical states */
volatile bool door_status_open = false;
bool door_status_open_old = false;
bool enter_loop = false;

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
#if 1
  Serial.begin(115200);
#endif

  attachInterrupt(digitalPinToInterrupt(DOOR_STATUS_PIN), door_detection_isr, CHANGE);

  setup_wifi();
  setup_hardware();
  setup_mqtt();

/* Publish a boot event for error tracking and debugging */
  bool publish_succeeded = false;
  while(!publish_succeeded)
  {
    publish_succeeded = client.publish("balcony_node/boot", "");
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

  /* Define callback function */

  /* Subscribe to topics */

}

/* Connect to the wireless network */
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T);

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
 /* Setup pins */
 pinMode(DOOR_STATUS_PIN, INPUT_PULLUP);

 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH);

#if DEBUG_ENABLED
  Serial.println("Hardware setup completed");
#endif
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("BalconyClient",NULL,NULL,0,2,0,0,1)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  client.publish("balcony_node/reconnect", "");
}

/* Handle the network part of the loop */
void network_loop()
{
#if DEBUG_ENABLED
  unsigned long current_time = millis();
#endif

  /* Enter critical area */
  noInterrupts();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  /* Exit critical area */
  interrupts();

#if DEBUG_ENABLED
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
  enter_loop = true;
  #if DEBUG_ENABLED
  Serial.println("Door detected via interrupt");
  #endif
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get the status of the door */
  get_door_status();
}

/* Node-specific logic */
/* If presence has been detected before door opens --> somebody left */
/* If presence has been detected after door opens --> somebody came in */
void node_logic()
{

}

/* Publish sensor information */
void publish_status()
{
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
      publish_succeeded = client.publish("balcony_node/door_status", String(door_status_open).c_str());
      if(!publish_succeeded) network_loop();
      yield();
    }
  }
}


/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
 * This loop is executed atomically
 */
void node_specific_loop()
{
#if DEBUG_ENABLED
  /* Performance measurement */
  unsigned long current_time = millis();
#endif

  /* Enter critical area */
  noInterrupts();

  /* Sensor polling */
  poll_sensors();

  /* Internal logic */
  node_logic();

  /* Publish sensor information */
  publish_status();

  enter_loop = false;

  /* Leave critical area */
  interrupts();

#if DEBUG_ENABLED
  current_time = millis() - current_time;
  Serial.print("Duration node loop: ");
  Serial.println(current_time);
#endif
}

void loop() {

  /* Connection handling */
  network_loop();

  /* Do loop only under request (interrupt) */
  if( enter_loop ) node_specific_loop();

  /* Enter light sleep - reduce power consumption */
  delay(SLEEP_TIME);
}
