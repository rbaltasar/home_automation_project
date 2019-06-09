import time
import sys
import paho.mqtt.client as paho

#define callback
def on_message(client, userdata, message):
    time.sleep(1)
    print("received message =",str(message.payload.decode("utf-8")))

def test_1():

    print("-----------Test case 1------------")
    print("Wait 61 seconds")
    #time.sleep(61)
    print("Publish entrance event")
    client.publish("entrance_node/entrance_detected",1)
    print("Wait 20 seconds")
    time.sleep(20)
    print("Publish PersonA detection event")
    client.publish("presence_detection/PersonA",1)
    print("Expectation: PersonA Greeting")

    print("Wait 5 seconds")
    time.sleep(5)
    print("Publish PersonB detection event")
    client.publish("presence_detection/PersonB",1)
    print("Expectation: PersonB Greeting")

    print("Wait 30 seconds")
    time.sleep(30)
    print("Publish presence of PersonB and PersonA")
    client.publish("presence_detection/PersonB",1)
    client.publish("presence_detection/PersonA",1)
    print("Wait 10 seconds")
    time.sleep(10)
    print("Publish exit event")
    client.publish("entrance_node/exit_detected",1)
    print("Expectation: Generic goodbye")
    print("Wait 61 seconds")
    time.sleep(61)
    print("Publish presence of PersonA")
    client.publish("presence_detection/PersonA",1)
    print("Wait 5 seconds")
    time.sleep(5)
    print("Publish exit event")
    client.publish("entrance_node/exit_detected",1)
    print("Expectation: PersonA goodbye")
    print("Wait 61 seconds")
    time.sleep(61)
    print("Publish presence of PersonB")
    client.publish("presence_detection/PersonB",1)
    print("Wait 5 seconds")
    time.sleep(5)
    print("Publish exit event")
    client.publish("entrance_node/exit_detected",1)
    print("Expectation: PersonB goodbye")

    print("-----------Test case 1 completed------------")

def test_2():

    print("Wait 61 seconds")
    #time.sleep(61)
    print("Publish PersonA detection event")
    client.publish("presence_detection/PersonA",1)
    print("Wait 10 seconds")
    time.sleep(10)
    print("Publish entrance event")
    client.publish("entrance_node/entrance_detected",1)
    print("Expectation: PersonA Greeting")
    print("Wait 10 seconds")
    time.sleep(10)
    print("Publish PersonB detection event")
    client.publish("presence_detection/PersonB",1)
    print("Expectation: PersonB Greeting")

    print("Wait 61 seconds")
    time.sleep(61)
    print("Publish exit event")
    client.publish("entrance_node/exit_detected",1)
    print("Expectation: Generic goodbye")
    print("Wait 1 seconds")
    time.sleep(1)
    print("Publish PersonA detection event")
    client.publish("presence_detection/PersonA",1)
    print("Expectation: Generic greeting")
    print("Wait 1 seconds")
    time.sleep(1)
    print("Publish exit event")
    client.publish("entrance_node/exit_detected",1)
    print("Expectation: PersonA goodbye")

    print("Wait 61 seconds")
    time.sleep(61)
    print("Publish presence of PersonB and PersonA")
    client.publish("presence_detection/PersonB",1)
    client.publish("presence_detection/PersonA",1)
    print("Wait 5 seconds")
    time.sleep(5)
    print("Publish entrance event")
    client.publish("entrance_node/entrance_detected",1)
    print("Expectation: PersonA Greeting")
    print("Expectation: PersonB Greeting")


    print("-----------Test case 2 completed------------")


if __name__== "__main__":


    test_num = int(sys.argv[1])
    print("Test to run: ", test_num)

    client= paho.Client("client-001")
    client.on_message=on_message
    print("connecting to broker ")
    client.connect("192.168.2.118",1883)#connect
    print("connected!")
    print("Starting test cases!")
    client.loop_start() #start loop to process received messages


    # Test case 1
    if(test_num is 1 or test_num is 0):
        test_1()
    if(test_num is 2 or test_num is 0):
        test_2()


    print("-----------Finished test------------")
    client.disconnect() #disconnect
    client.loop_stop() #stop loop
