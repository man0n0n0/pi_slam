import serial
from CalcLidarData import CalcLidarData
import matplotlib.pyplot as plt
import math
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='polar')
ax.set_title('lidar (exit: Key E)',fontsize=18)

plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)

ser = serial.Serial(port='COM69',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

tmpString = ""
angles = list()
distances = list()
prevLine = None

def the_callback(angles, distances):
    global prevLine
    if not prevLine is None:
        prevLine.remove()
    line = ax.scatter([-angle for angle in angles], distances, c="pink", s=5)
    ax.set_theta_offset(math.pi / 2)
    plt.pause(0.01)
    prevLine = line

while True:
    all_b = ser.read_all()
    for b in all_b:
        tmpInt = int( b)
        b = bytes([b])
        if (tmpInt == 0x54):
            tmpString +=  b.hex()+" "
            flag2c = True
            continue
        
        elif(tmpInt == 0x2c and flag2c):
            tmpString += b.hex()
            if(not len(tmpString[0:-5].replace(' ','')) == 90 ):
                tmpString = ""
                loopFlag = False
                flag2c = False
                continue

            lidarData = CalcLidarData(tmpString[0:-5])
            angles.extend(lidarData.Angle_i)
            distances.extend(lidarData.Distance_i)
            if(len(angles) > 50*12):
                split = [i for i in range(len(angles)-1) if angles[i+1]< angles[i]]
                first = angles[:split[0]+1]
                angles = angles[split[0]+1:]
                firstDist = distances[:split[0]+1]
                distances = distances[split[0]+1:]
                the_callback(first, firstDist)
            tmpString = ""
        else:
            tmpString += b.hex()+" "
        
        flag2c = False

ser.close()