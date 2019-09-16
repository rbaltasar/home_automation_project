#if !defined CONFIG_H
#define CONFIG_H

/* Debugging */
#define DEBUG_ENABLED 1

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN 4
#define DOOR_SENSOR_PIN 2
#define DHTTYPE DHT22   // DHT 11
#define LIGHT_SENSOR_PIN_1 6
#define LIGHT_SENSOR_PIN_2 7
#define LED_BUILTIN 2

/* Measurement settings */
#define MEASUREMENT_DELTA 0.01 //Minimum change in measurements (%) to publish the data
#define BUFFER_SIZE 20

/* Ensure a minimum publish rate */
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every 10 minutes
#define NUM_PUBLISH_TOPICS 3
#define DEFAULT_POLLING_PERIOD 20000 //20 seconds
#define WARNING_POLLING_PERIOD 20000 //20 seconds. Currently not used
#define SLEEP_TIME 1000

/* LED Settings */
#define COLOR_MODE GRB //GRB
#define PRINT_DELAY 37
#define RGB_DEFAULT 255
#define NUM_LEDS 180
#define LED_PIN 14

/* Communication settings (MQTT) */
#define INIT_COMM_TIMEOUT 60000
#define ALIVE_PERIOD 30000
#define BLINK_PERIOD 2000
#define HANDSHAKE_ATTEMPT_INTERVAL 3000

#endif
