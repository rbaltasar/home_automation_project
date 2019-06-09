
#if !defined CONFIG_H
#define CONFIG_H

/* Hardware configuration */
#define DOOR_SENSOR_PIN 5
#define TEMPERATURE_SENSOR_PIN 4
#define DHTTYPE DHT22   // DHT 11
#define LED_BUILTIN 2

/* LED Settings */
#define COLOR_MODE GRB //GRB
#define PRINT_DELAY 37
#define RGB_DEFAULT 10
#define NUM_LEDS 96
#define LED_PIN 21

/* Measurement settings */
#define MEASUREMENT_DELTA 0.01 //Minimum change in measurements (%) to publish the data
#define BUFFER_SIZE 10
 
/* Ensure a minimum publish rate */
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every 10 minutes
#define NUM_PUBLISH_TOPICS 2
#define DEFAULT_POLLING_PERIOD 20000 //10 seconds
#define WARNING_POLLING_PERIOD 20000 //Currently not used

/* Communication settings (MQTT) */
#define INIT_COMM_TIMEOUT 3000
#define ALIVE_PERIOD 30000
#define BLINK_PERIOD 2000

/* FFT configuration */
#define NSAMPLES 64
#define SAMPLING_FREQUENCY 10000

/* Clap detection config */
#define THRESHOLD_CLAP 710

/* Debugging */
#define DEBUG_TRACES 1
#define DEBUG_TRACES_TIME 0
#define DEBUG_TRACES_FREQ 0
#define DEBUG_TRACES_FREQ_SPECTRUM 0
#define DEBUG_TRACES_RGB 0
#define DEBUG_TRACES_LED 0
#define DEBUG_TRACES_CLAP 0

#endif
