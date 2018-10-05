import numpy as np
import matplotlib.pyplot as plt

L,l = 1,0.5  #length and width of the boat
boot = 5     #5m meter of wire between the boat and the magnetometer
size_m = 0.5 #size of the magnetometer

class Boat():
    def __init__(self, X):
        self.x = X[0]
        self.y = X[1]
        self.theta = X[2]
        self.v = X[3]
        self.L = 1
        self.l = 0.5

    def disp(self):
        absc_r = [x+L*np.cos(theta), x+l/2*np.sin(theta), x-l/2*np.sin(theta), x+L*np.cos(theta)]
        ordo_r = [y+L*np.sin(theta), y-l/2*np.cos(theta), y+l/2*np.cos(theta), y+L*np.sin(theta)]

class Sensor():
    def __init__(self, X):
        self.x = X[0]
        self.y = X[1]
        self.theta = X[2]
        self.v = X[3]
        self.boot = 10
        self.size_m = 0.5


def draw_boat_and_magneto(Xr, Xm):
    [x , y , theta,  v] = Xr #state vector robot/zodiac
    [xm, ym, alpha, vm] = Xm #state vector magnetometer
    #Robot
    absc_r = [x+L*np.cos(theta), x+l/2*np.sin(theta), x-l/2*np.sin(theta), x+L*np.cos(theta)]
    ordo_r = [y+L*np.sin(theta), y-l/2*np.cos(theta), y+l/2*np.cos(theta), y+L*np.sin(theta)]
    #Wire
    absc_w = [x,xm]
    ordo_w = [y,ym]
    #magnetometer
    absc_m = [xm]
    ordo_m = [ym]

    #Affichage
    plt.plot(absc_r, ordo_r, 'r' , linewidth = 3.0)
    plt.plot(absc_w, ordo_w, 'b' , linewidth = 1.0)
    plt.plot(absc_m, ordo_m, 'bo', markersize = 10)
    return 0

def main():

    plt.figure()

    Xr = [5,5,np.pi/2,3]
    alpha = np.pi/4
    xm, ym = Xr[0] - boot*np.cos(Xr[2]-alpha), Xr[1] - boot*np.sin(Xr[2]-alpha)
    vm = Xr[3]*np.cos(Xr[2]-alpha)
    Xm = [xm,ym,alpha,vm]


    draw_boat_and_magneto(Xr, Xm)
    plt.show()
    print("Finished correctly")
    return 0

if __name__ == '__main__':
    main()
