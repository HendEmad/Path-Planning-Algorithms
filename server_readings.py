import math
import sys
import time
import os
import smtplib
import time
from time import sleep
from Adafruit_IO import MQTTClient, Client, Feed, Data

smtpUser = 'user1.python@gmail.com'
smtpPass = 'hwwa fgzb pkag lifk'
toAdd = ['hendemad981@gmail.com']
fromAdd = smtpUser
ADAFRUIT_IO_KEY = "aio_YKjP46RPqZrqTKlhReLrQORvkzvt"
ADAFRUIT_IO_USERNAME = "HendEmad"

current_state = 0
start_lat, start_lng, end_lat, end_lng = 0, 0, 0, 0


def connected(client):
    print('Connected to Adafruit IO!  Listening for changes...')
    client.subscribe('start_lat')
    client.subscribe('start_lng')
    client.subscribe('end_lat')
    client.subscribe('end_lng')
    client.subscribe('drone-deploy')


def disconnected(client):
    print('Disconnected from Adafruit IO!')
    sys.exit(1)


def message(client, feed_id, payload):
    global current_state, start_lat, start_lng, end_lng, end_lat
    if feed_id == 'drone-deploy':
        current_state = payload
    elif feed_id == 'start_lat':
        start_lat = payload
        print("Feed {0} received new value: {1}".format(feed_id, start_lat))
    elif feed_id == 'start_lng':
        start_lng = payload
        print("Feed {0} received new value: {1}".format(feed_id, start_lng))
    elif feed_id == 'end_lat':
        end_lat = payload
        print("Feed {0} received new value: {1}".format(feed_id, end_lat))
    elif feed_id == 'end_lng':
        end_lng = payload
        print("Feed {0} received new value: {1}".format(feed_id, end_lng))


client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

client.connect()
client.loop_background()
while True:
    if current_state == '1':
        R = 6371000
        start_lat = float(start_lat)
        start_lng = float(start_lng)
        end_lat = float(end_lat)
        end_lng = float(end_lng)
        # lat ==> b, long ==> a
        start_x = (R * math.cos(start_lat) * math.cos(start_lng)) / 1000
        start_y = (R * math.cos(start_lat) * math.sin(start_lng)) / 1000
        print("Received start location: ({0}, {1})".format(start_x, start_y))

        end_x = (R * math.cos(float(end_lat)) * math.cos(float(end_lng))) / 1000
        end_y = (R * math.cos(float(end_lat)) * math.sin(float(end_lng))) / 1000
        print("received end location: ({0}, {1})".format(end_x, end_y))

        subject = 'Sudden Cardiac Arrest Case'
        header = "To : " + str(toAdd) + "\n" + "From : " + str(fromAdd) + "\n" + "Subject: " + str(subject)
        body = "there is a sudden cardiac arrest case located in: https://www.google.com/maps/dir//"+str(end_y)+","+str(end_x)+" \nthat was happened at"+ time.ctime()
        print(header + '\n' + body)

        s = smtplib.SMTP('smtp.gmail.com',587)
        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(smtpUser , smtpPass)
        s.sendmail(fromAdd, toAdd, header + '\n\n' + body)
        # s.quit()
        # sys.exit(1)
        break

print("Start location in lat and lng = ({0}, {1})".format(start_lat, start_lng))
print("Start location in x and y = ({0}, {1})".format(start_x, start_y))

print("End location in lat and lng = ({0}, {1})".format(end_lat, end_lng))
print("End location in x and y = ({0}, {1})".format(end_x, end_y))

width = float(end_x) - float(start_x)
height = float(end_y) - float(start_y)
print("This path performs on area of ", width, "m x ", height, "m")

print(sys.getsizeof(start_x))  # 24
print(sys.getsizeof(start_y))  # 24
print(sys.getsizeof(end_x))  # 24
print(sys.getsizeof(end_y))  # 24
