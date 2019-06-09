import time
import paho.mqtt.client as paho
import sys

#define callback
def on_message(client, userdata, message):
    time.sleep(1)
    print("received message =",str(message.payload.decode("utf-8")))


event = int(sys.argv[1])

client= paho.Client("clientAttic")
client.on_message=on_message
print("connecting to broker ")
client.connect("192.168.2.118",1883)#connect
client.loop_start() #start loop to process received messages
print("publishing ")
client.publish("attic_node/door_status",event)#publish
client.disconnect() #disconnect
client.loop_stop() #stop loop
