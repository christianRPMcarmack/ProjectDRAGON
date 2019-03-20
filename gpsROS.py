import pynmea2
import serial, time
import csv
import numpy as np


ser = serial.Serial('/dev/ttyS5', 9600)
#ser2 = serial.Serial('/dev/ttyACM3', 9600)
i=1

longitude = []
latitude = []
longitude2 = []
latitude2 = []

f = open("gps.txt","w+")
fc = open("gps.csv", "w")
f2 = open("gps2.txt","w+")
fc2 = open("gps2.csv", "w")
print('started')

def nmea(serial_line,latitude,longitude,option):

    # Parse GNSS Data 
    msg = pynmea2.parse(serial_line)
    #print('Lat: ' + msg.lat + '  Lon: ' + msg.lon)

    msg.lat = float(msg.lat)/100
    msg.lon = float(msg.lon)/(-100)
    msg.lat = str(msg.lat)
    msg.lon = str(msg.lon)
    
    if option==1:
        # Write Data to gps.txt
        f.write('Lat: ' + msg.lat + '  Lon: ' + msg.lon + '\r\n' )
    
        # Write Data to gps.csv
        writer = csv.writer(fc)
        writer.writerows(zip(latitude, longitude))
    elif option==2:
        # Write Data to gps.txt
        f2.write('Lat: ' + msg.lat + '  Lon: ' + msg.lon + '\r\n' )
    
        # Write Data to gps.csv
        writer = csv.writer(fc2)
        writer.writerows(zip(latitude, longitude))
    

    # Store as Array
    latitude.append(msg.lat)
    longitude.append(msg.lon)

    return msg, latitude, longitude

while 1:
    serial_line = ser.readline()
    print("in while loop")
    #serial_line2 = ser2.readline()
    try:
        msg, latitude, longitude = nmea(serial_line,latitude,longitude,1)
        print(latitude)
        #msg2, latitude2, longitude2 = nmea(serial_line2,latitude2,longitude2,2)
        i = i+1
        if i==60:
            length = len(latitude)
            latmean = np.mean(latitude[length-60:])
            lonmean = np.mean(longitude[length-60:])
            i=0
            latlng = [latmean, lonmean]
            print('## Averaged Values ##############################')
            print('Lat: '+ latmean)
            print('Lon: '+ lonmean)
    except:
        pass
f.close() 
fc.close()   
ser.close()