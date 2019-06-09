# Livingroom node

## Functionality
This node is responsible for sensing the temperature, humidity and light amount in the living room, as well as monitoring the status of the balcony door.

The node supports OTA software updates, under the URL "http://livingroom_node.local"

## Implementation
The node-specific functionality (including publish information) is executed periodically.
The temperature & humidity sensor can only be polled once in a second (otherwise we get weird NaN measurements), so this would be our bottleneck for the polling frequency of this node.
On the other hand, as we are controlling a lamp, we want it to be responsive, so we don't need to wait until the next polling iteration to see the change in the lamp.
To implement both functionalities with as few code as possible, we enter the node-specific loop (polling, logic and publish) if any of these conditions are met:
* Time between iterations happened (polling period).
* A light change request (Alexa/MQTT) has been received.

Due to our code simplification, a light change request also triggers the full node-specific loop, including polling the sensors. If this request happens right after the periodic execution of the node-specific loop, we might have polled to quickly the DHT11 sensor, receiving a NaN as output.
This is no issue because this situation will not happen often, and the sensor signal is filtered as we can see next.

The sensors used in this node are very sensible to any disturbance or wrong wiring, resulting to a NaN return. Working with NaN values or see them displayed in the Dashboard is a little bit ugly, and therefore each node that uses these sensors implements a signal smoothing algorithm that buffers the N latest measurements, discards the M NaN measurements out of the buffer and then averages the N-M remaining values, smoothing the published sensor information and ignoring sporadic measurement errors.

## Known issues

## Future improvements

## Used hardware
* Nodemcu ESP12
* DH11
* BH1750
* Door sensor

## External libraries used
* PubSubClient: https://pubsubclient.knolleary.net
* Arduino DHT library: https://learn.adafruit.com/dht
* ArduinoJson: https://github.com/bblanchon/ArduinoJson
* BH1750 library: https://github.com/claws/BH1750

## Schematic
![Alt text](wiring.png)
