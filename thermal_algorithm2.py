import numpy as np
import matplotlib.pyplot as plt

# Inputs
    # Mission inputs
takeoff = np.array([0,0, 500])
landing = np.array([7000,500, 200])
maxAltitude = 600
    # Glider inputs
groundSpeed = 32/3.6
glideRatio = 10
    # Thermal inputs
thermals = np.array([[500, 700], [600, 600], [700, 1000]])
thermalStrengths = np.array([3, 4, 1])
thermalWidths = np.array([0, 1, 2])
def plotThermal(core, radius):
    cx, cy = core
    x = []
    y = []
    for theta in np.arange(0, 2.1*np.pi, 0.1):
        x.append(cx + radius*(np.cos(theta)))
        y.append(cy + radius*(np.sin(theta)))
    plt.plot(x, y)
# Determining thermal radius
thermalRadius = np.zeros(thermalWidths.shape) 
for i, width in enumerate(thermalWidths):
    if width == 0:
        # Narrow thermal
        thermalRadius[i] = 90 + 23*thermalStrengths[i]
    elif width == 1:
        # Mean thermal
        thermalRadius[i] = 170 + 41*thermalStrengths[i]
    elif width == 2:
        # Wide thermal
        thermalRadius[i] = 320 + 74*thermalStrengths[i]
# Thermal function coefficients
    # Linear coefficients
a = -thermalStrengths/(thermalRadius - 60)
b = thermalStrengths-(60*a)
    # Quadratic coefficients
c = a/120
e = thermalStrengths - (c*60**2)

for index, thermal in enumerate(thermals):
    plotThermal(thermal, thermalRadius[index])
    
# Initiating time, position and speed variables
deltat = 1
time = np.arange(0, 300, deltat)
p = np.array([takeoff])
v = np.array([[groundSpeed, 0, -groundSpeed/glideRatio]])
alpha = 0
omega = 0
nearThermal = False

for i,t in enumerate(time):
    p = np.append(p, [p[i]+v[i]*deltat], axis=0)
    distanceToLanding = np.linalg.norm(landing-p[i])
    distanceToThermals = [np.linalg.norm(thermal-p[i, :2]) for thermal in thermals]
    # Check if the glider can glide straight to the landing zone
    if distanceToLanding < p[i,2]*glideRatio:
        newv = groundSpeed*(landing-p[i])/distanceToLanding
        newv[2] = -groundSpeed/glideRatio
        v = np.append(v, [newv], axis=0)
    else: 
        # Check if near a thermal
        for j, distance in enumerate(distanceToThermals):
            if distance<(2*thermalRadius[j]):
                nearThermal = True
            else:
                nearThermal = False        
        if not nearThermal:
            # Find the closest thermal that is also closer to landing than the 
            # current position
            for distance in np.sort(distanceToThermals):
                thisThermalIndex = distanceToThermals.index(distance)
                thisThermal = thermals[thisThermalIndex]
                thermalToLanding = np.linalg.norm(thisThermal-landing[:2])
                if thermalToLanding < distanceToLanding:
                    newv = [0,0,-groundSpeed/glideRatio]
                    newv[:2] = groundSpeed*(thisThermal-p[i,:2])/distance
                    break
            v = np.append(v, [newv], axis =  0)
        #If within less than two thermal radius from the core
        else:
            r = np.linalg.norm(thisThermal-p[i,:2])
            thisThermalRadius = thermalRadius[thisThermalIndex]
            deltaVZ = v[i,2] - v[i-1,2]
            vz = -groundSpeed/glideRatio
            #omega =   40*deltaVZ
            if r < thisThermalRadius:
                if deltaVZ<0:
                    omega = -10
                elif deltaVZ>=0:
                    omega = -7     
                if r > 60:
                    vz = vz + a[thisThermalIndex]*r + b[thisThermalIndex]
                else:
                    vz = vz +c[thisThermalIndex]*r**2 + e[thisThermalIndex]
            vx = v[i,0]*np.cos(np.deg2rad(omega)) - v[i,1]*np.sin(np.deg2rad(omega))     
            vy = v[i,0]*np.sin(np.deg2rad(omega)) + v[i,1]*np.cos(np.deg2rad(omega))
            v = np.append(v, [[vx, vy, vz]], axis =  0) 
    plt.scatter(p[i,0], p[i,1], s=1)
    
# plt.plot(p[:,0], p[:,1])
plt.scatter(thermals[:,0], thermals[:,1], s=10)
plt.grid()
plt.show()
    
