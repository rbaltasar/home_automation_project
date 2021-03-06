import time
import paho.mqtt.client as paho
import json
from datetime import datetime
from threading import Thread
from random import randrange

#--------------------------------------------#
#Class to set random static effects in a     #
#defined set of lamps upon manual request or #
#an entrance event                           #
#--------------------------------------------#
class LampEffectPlugIn:

    def __init__(self):

        #Create client
        self._client = paho.Client("LampEffectPlugIn")
        #Define callback for received topics
        self._client.on_message =  self.callback

        #Initialize local variables
        self._isThreadRunning = False
        self._stopThread = False
        self._isEndless = False
        self._isEffectUponEntrance = False
        self._ignoreNextModeRequest = False

        self._startTime = 0
        self._effectTime = 20 #1 min default
        self._currentEffect = 0

        #Import json effect list
        with open("effects_list.json") as fp:
          self._sequence_list = json.load(fp)
          #Todo: handle error

        #Get the number of available effects
        self._numEffects = len(self._sequence_list["effects"])
        self._mask = self._sequence_list["mask"]
        print("Loaded ", self._numEffects, " effects")

    def __del__(self):

        #Stop the plug-in
        self.stop()

    #Stop the pluggin: thread and MQTT connection
    def stop(self):

        #Stop the thread
        self.stopThread()
        #Stop the MQTT client
        print("Stopping MQTT client")
        self._client.disconnect() #disconnect
        self._client.loop_stop() #stop loop thread

    #Stop the working thread
    def stopThread(self):

        print("Stopping working thread")
        self._stopThread = True
        if(self._isThreadRunning):
          self._working_thread.join()

        self._isThreadRunning = False

    #Start the client
    def begin(self):

        print("Starting the addon")

        #Todo: IP address and port in config
        self._client.connect("192.168.2.118",1883)
        #Start client thread
        self._client.loop_start()
        print("Started MQTT client")

        #Subscribe to topics
        self._client.subscribe("lamp_pluggin/config")
        self._client.subscribe("lamp_pluggin/start")
        self._client.subscribe("lamp_network/mode_request")
        self._client.subscribe("entrance_node/entrance_detected")

    #Start a thread to trigger lamp effects
    #Only one running thread is allowed
    #Thread configuration parameters may be changed while the thread is running
    def startThread(self):

        #There is no thread running, start the thread
        if(not self._isThreadRunning):

          #Start thread
          print("Starting working thread")
          self._stopThread = False
          self._startTime = time.time()
          self._working_thread = Thread(target = self.working_thread)
          self._working_thread.start()
          self._isThreadRunning = True

        #There is a thread running but it already finished its job. Join old thread and start new one
        elif(self._stopThread is True):
          #Join old thread
          print("Joining old thread")
          self.stopThread()
          #Start new thread
          print("Starting new thread (recursive call)")
          self.startThread()

    #Send an effect request:
    # *Periodically until stop request if the the termination policy is configured to endless
    # *Once otherwise
    def working_thread(self):

      #Trigger the next effect
      self.trigger_next_effect()

      loopCounter = 0
      while self._stopThread is not True:

        isRandomColor = int(self._sequence_list["effects"][self._currentEffect]["isRandomColorTransition"])

        #Continue with the current effect
        if( (time.time() - self._startTime) < self._effectTime ):

          time.sleep(1)
          print("Thread iddle loop. Remaining: ", self._effectTime - (time.time() - self._startTime) )

          if(loopCounter == (self._effectTime / 5)):
              loopCounter = 0
              if(isRandomColor == 1):
                  self.triggerRandomColorChange()
          else:
              loopCounter += 1

        #Current effect timeout has expired
        else:

          print("Timeout")

          #Case of endless effect
          if self._isEndless is True:
            #Trigger the next effect
            self.trigger_next_effect()
            #Reset effect start time
            self._startTime = time.time()

          #Case of finite effect
          else:
            #Exit the thread loop. Thread will still appear as "running", but it will be handled with a new thread start request
            print("Request transition to OFF")
            self._ignoreNextModeRequest = True #We don't want to interpret our own change request as an external request to finish the Random mode
            request_string = json.dumps({"mode": 0, "id_mask": self._mask})
            print(request_string)
            self._client.publish("lamp_network/mode_request",request_string)
            print("Exit thread endless loop")
            self._stopThread = True


    def compute_next_effect(self):

      #For random change (tbd)

      #For linear change
      self._currentEffect += 1
      if(self._currentEffect == self._numEffects):
        self._currentEffect = 0

    #Trigger a random color change
    def triggerRandomColorChange(self):

      print("Publishing random color")
      r = randrange(255)
      g = randrange(255)
      b = randrange(255)
      request_string = json.dumps({"R": r, "G":g, "B":b, "id_mask": self._mask})
      print(request_string)
      self._client.publish("lamp_network/light_color",request_string)
      time.sleep(0.5)

    #Find another suitable effect and send a change request
    def trigger_next_effect(self):

      #Compute next effect for next iteration
      self.compute_next_effect()

      #Extract effect parameters
      mode = int(self._sequence_list["effects"][self._currentEffect]["mode"])
      speed = int(self._sequence_list["effects"][self._currentEffect]["speed"])
      delay = int(self._sequence_list["effects"][self._currentEffect]["delay"])
      r = int(self._sequence_list["effects"][self._currentEffect]["r"])
      g = int(self._sequence_list["effects"][self._currentEffect]["g"])
      b = int(self._sequence_list["effects"][self._currentEffect]["b"])

      print("Effect to be triggered: ", mode, ", ", speed, ", ", delay, ", ", r, ", ", g, ", ", b )

      #Special handling for "Bouncing Balls" effect: send the delay in advance for memory allocation
      if(mode == 25):
        print("Publish an initial delay only for mode 25")
        request_string = json.dumps({"delay": delay, "id_mask": self._mask})
        print(request_string)
        self._client.publish("lamp_network/effect_delay",request_string)
        time.sleep(0.5)

      #Publish mode change request
      self._ignoreNextModeRequest = True #We don't want to interpret our own change request as an external request to finish the Random mode
      request_string = json.dumps({"mode": mode, "id_mask": self._mask})
      print(request_string)
      self._client.publish("lamp_network/mode_request",request_string)

      #Publish speed
      request_string = json.dumps({"speed": speed, "id_mask": self._mask})
      print(request_string)
      self._client.publish("lamp_network/effect_speed",request_string)
      time.sleep(0.5)

      #Publish delay
      request_string = json.dumps({"delay": delay, "id_mask": self._mask})
      print(request_string)
      self._client.publish("lamp_network/effect_delay",request_string)
      time.sleep(0.5)

      #Publish speed
      request_string = json.dumps({"R": r, "G":g, "B":b, "id_mask": self._mask})
      print(request_string)
      self._client.publish("lamp_network/light_color",request_string)
      time.sleep(0.5)

    #MQTT callback with message information
    def callback(self, client, userdata, message):

        #Parse Json message
        received_msg = json.loads(message.payload)
        print("received message =",received_msg)

        #Handle entrance event
        if(message.topic == "entrance_node/entrance_detected"):

            print("Received an entrance event. Wait 10 seconds")
            time.sleep(10)
            if(self._isEffectUponEntrance):
              self.startThread() #If no endless effect has been started it will start a new thread. Otherwise nothing will happen

        #Handle configuration message
        elif(message.topic == "lamp_pluggin/config"):

            print("Received configuration message")
            self._isEffectUponEntrance = bool(received_msg['effect_enable'])
            print("New effect upon entrance: ", self._isEffectUponEntrance)
            self._effectTime = int(received_msg['effect_duration']) * 60
            print("New effect time: ", self._effectTime)

        #Handle transition to random mode
        elif(message.topic == "lamp_pluggin/start"):

            #Set variables for endless random effect
            print("Starting endless lamp pluggin")
            self._isEndless = True
            self.startThread()

        #Handle normal mode transition request
        elif(message.topic == "lamp_network/mode_request"):

            print("Received a mode request message")
            mode = int(received_msg['mode'])
            mask = int(received_msg['id_mask'])

            #Ignore an event triggered from our own mode change request
            #Ignore also events not targeted to the defined lamps
            if (self._ignoreNextModeRequest is False) and ((mask & self._mask) != 0):

              print("Handling mode transition request")
              #Reset variables
              self._isEndless = False
              #Stop thread
              self.stopThread()

            #Mode request is being ignored. Reset flag
            elif self._ignoreNextModeRequest is True:

              print("Ignoring the mode transition request")
              self._ignoreNextModeRequest = False


if __name__== "__main__":

    #Start lamp sequence addon
    addon = LampEffectPlugIn()
    addon.begin()

    #Endless loop. The plugging (communication and effect triggering) is handled asynchronously
    do_loop = True
    while do_loop:

        try:

            time.sleep(1)

        except KeyboardInterrupt:
                print "Ctrl-c received! Sending kill to threads..."
                #Stop the addon
                addon.stop()
                #Exit endless loop
                do_loop = False
