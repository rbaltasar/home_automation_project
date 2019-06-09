import time
import paho.mqtt.client as paho

#define callback
def on_message(client, userdata, message):
    time.sleep(1)
    print("received message =",str(message.payload.decode("utf-8")))

client= paho.Client("client-001")
client.on_message=on_message
print("connecting to broker ")
client.connect("192.168.2.118",1883)#connect
client.loop_start() #start loop to process received messages
print("publishing ")
client.publish("entrance_node/door_status",0)#publish
time.sleep(10)
client.publish("entrance_node/door_status",1)
client.publish("entrance_node/entrance_detected",1)
client.disconnect() #disconnect
client.loop_stop() #stop loop
