# -*- coding: utf-8 -*-
#kate: indent-mode python; tab-width 4; space-indent true; indent-width 4

import sys

filename = sys.argv[1]

file = open(filename, 'r')

#dict of speed -> data
speeds = {}

speed = None
data = []

for line in file:
    #cleanup \n at end of line
    line = line.strip()
    
    if len(line) == 0:
        continue
    
    if line.find("Speed:") >= 0:
        #if we have data...assign it to the last speed
        if len(data) > 0:
            speeds[speed] = data
            
        speed = int(line.split(' ')[1])
        data = []
    elif speed != None:
        split = line.split(' ')
        time = float(split[0])
        vMag = float(split[1])
        dPos = float(split[2])
        
        data.append((time, vMag, dPos))

speeds[speed] = data

file.close()

def plotData(data):
    for d in data:
        v = d[1]
        t = d[0]
        
        if t > 1.5:
            break
        
        print t, v
    
    print ""
    
def calcVel(data):
    n = len(data)
    sample = int(n * .25 + .5)
    
    vSum = 0
    startT = data[n-sample][0]
    endT = data[n-1][0]
    dpSum = 0
    for i in range(n-sample, n):
        d = data[i]
        t = d[0]
        v = d[1]
        dp = d[2]
        
        vSum += v
        dpSum += dp
    
    vel = vSum/sample
    #print "\tDPVel: ", dpSum/(endT - startT)
    
    return vel

def calcAccel(data):
    c = 0
    oldV = 0
    oldT = 0
    accSum = 0
    for d in data:
        newV = d[1]
        newT = d[0]
        
        if newT < .15:
            continue
        
        if newT > .3:
            break
        
        accel = (newV - oldV)/(newT - oldT)
        
        oldV = newV
        oldT = newT
        
        if accel > 0:
            accSum += accel
            c += 1

    #print c
    accel = 0
    if c > 0:
        accel = accSum/c
    
    return accel


def velData():
    keys = speeds.keys()
    keys.sort()
    for s in keys:
        ##accel should not take into account lower speeds..they are unreliable
        #if s < 50:
        #    continue
        data = speeds[s]
        #print "Speed: %d\t" % s
        #accel = calcAccel(data)
        #print s, accel
        #accels.append(accel)
        
        #plotData(data)
        
        v = calcVel(data)
        print s,v
        
def accelData():
    accels = []
    
    keys = speeds.keys()
    keys.sort()
    for s in keys:
        ##accel should not take into account lower speeds..they are unreliable
        if s < 50:
            continue
        data = speeds[s]
        accel = calcAccel(data)
        print s, accel
        accels.append(accel)

        
    print "Accel:", sum(accels)/len(accels)

accelData()

velData()

#print "Speed:", 100
#plotData(speeds[100])

