import time
import paho.mqtt.client as paho
from icalendar import Calendar, Event
from datetime import datetime
import urllib3

stuttgart_abfallkalendar_url = 'https://service.stuttgart.de/lhs-services/aws/api/ical?street=Mathildenstr.&streetnr=7A'
TIME_THRESHOLD = 50

#--------------------------------------------#
#Class process calendar event                #
#--------------------------------------------#
class CalendarPlugIn:

    def __init__(self, url):

        #Download calendar
        https = urllib3.PoolManager()
        req = https.request('GET',url)
        data = req.data
        self._calendar = Calendar.from_ical(data)

        self.Reset()

    def Reset(self):

        self._notify_biomull = False
        self._notify_papier = False
        self._notify_plastic = False
        self._notify_restmull = False

    def GetType(self, summary):
        return summary.split()[0]

    def ProcessCalendar(self):

        for event in self._calendar.walk('vevent'):

            date = event.get('dtstart')
            summary = event.get('summary')
            #print("Date: " + str(date.dt))
            #print(summary)

            # Process only events for the next day
            timediff = date.dt.replace(tzinfo=None) - datetime.utcnow()
            timediff = timediff.total_seconds() / (60 * 60 * 24) #To days
            #print("Diff: " + str(timediff))
            if(timediff < TIME_THRESHOLD):
                # Get type of the event
                event_type = self.GetType(summary)
                # Add to containers
                if(event_type == "Gelber"):
                    self._notify_plastic = True
                elif(event_type == "Altpapier"):
                    self._notify_papier = True
                elif(event_type == "Restmüll"):
                    self._notify_restmull = True
                elif(event_type == "Biomüll"):
                    self._notify_biomull = True

    def OpenMQTTConnection(self):

        #Create MQTT client
        self._client = paho.Client("GarbagePluggin")
        #Todo: IP address and port in config
        self._client.connect("192.168.2.118",1883)
        #Start client thread
        self._client.loop_start()
        print("Started MQTT client")
        time.sleep(0.1)

    def ProcessEvents(self):

        #Connect to MQTT broker
        self.OpenMQTTConnection()

        if(self._notify_plastic):
            self.NotifyPlastic()
            time.sleep(0.5)
        if(self._notify_papier):
            self.NotifyPapier()
            time.sleep(0.5)
        if(self._notify_restmull):
            self.NotifyRestmull()
            time.sleep(0.5)
        if(self._notify_biomull):
            self.NotifyBiomull()
            time.sleep(0.5)

    def NotifyPlastic(self):
        print("Notifying plastic")
        self._client.publish("smart_garbage/gelber","1",1)

    def NotifyPapier(self):
        print("Notifying papier")
        self._client.publish("smart_garbage/papier","1",1)

    def NotifyRestmull(self):
        print("Notifying restmull")
        self._client.publish("smart_garbage/restmull","1",1)

    def NotifyBiomull(self):
        print("Notifying biomull")
        self._client.publish("smart_garbage/biomull","1",1)


if __name__== "__main__":

    calendar_plugin = CalendarPlugIn(stuttgart_abfallkalendar_url)
    calendar_plugin.ProcessCalendar()
    calendar_plugin.ProcessEvents()
