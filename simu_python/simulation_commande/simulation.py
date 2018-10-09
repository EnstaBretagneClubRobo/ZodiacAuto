import numpy as np
import matplotlib.pyplot as plt
from roblib import *

L,l = 1,0.5  #length and width of the boat
boot = 2     #5m meter of wire between the boat and the magnetometer
size_m = 0.5 #size of the magnetometer

class Boat():
    def __init__(self, X):

        self.x = X[0] # abscisse du zodiac
        self.y = X[1] # ordonnee du ezodiac
        self.delta = X[2] # angle entre l'axe de la remorque et le zodiac 
        self.v = X[3] # vitesse du zodiac

        self.xm = X[4] # abscisse de la remorque
        self.ym = X[5] # ordonnee de la remorque
        self.theta = X [6] #angle entre les abscisses et la remorque
        self.vm= X[7] #vitesse de la remorque

        self.boot = 10 # longueur de cable
        self.size_m = 0.5
        self.L = 1 # longueur du bateau
        self.l = 0.5 # largeur du zodiac



def draw_boat_and_magneto(X):
    x, y, delta, v, xm, ym, theta, vm = X
    #[x , y , delta , v] = X[0:4]  #state vector robot/zodiac
    #[xm, ym, theta, vm ]= X[4:8]
    #state vector magnetometer
    #Robot
    absc_r = [x+L*np.cos(theta +delta), x+l/2*np.sin(theta +delta), x-l/2*np.sin(theta +delta), x+L*np.cos(theta +delta)]
    ordo_r = [y+L*np.sin(theta +delta), y-l/2*np.cos(theta +delta), y+l/2*np.cos(theta +delta), y+L*np.sin(theta +delta)]
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

def f(X,u):
    x, y, delta, v, xm, ym, theta, vm = X
    #epsilon = 0.1 # marge d'erreur pour le cable

    dx = v*np.cos(theta+delta)
    dy = v*np.sin(theta+delta)
    dv = 1
    ddelta = u
     
    if error(X) >= 0 : # garde la remorque à la bonne distance du zodiac
            #dxl, dyl =dx - boot*dtheta*np.sin(theta), dy - boot*dtheta*np.cos(theta)
            dxm, dym, dtheta, dvm = v*np.cos(delta)*np.sin(theta), v*np.cos(delta)*np.sin(theta), v*np.sin(delta), v*np.cos(delta)

    
    if error(X) < 0 : # diminue la vitesse de la remorque si le cable n'est pas tendu

        dxm, dym, dtheta, dvm = vm*np.cos(theta), vm*np.sin(theta),0, 0.5*vm

    return np.array([dx,
        dy,
        ddelta,
        dv,
        dxm,
        dym,
        dtheta,
        dvm])

def error(X):
    # retourne la distance entre la remorque et le bout de la corde du zodiac
    x, y, delta, v, xm, ym, theta, vm = X
    xl, yl = x - boot*np.cos(theta), y - boot*np.sin(theta) # coordonnees limite de la remorque
    return np.sqrt((xm-xl)*(xm-xl)+(ym-yl)*(ym-yl)) # distance entre les coords limite et calculé de la remorque


def control(X,w, dw):
    x, y, delta, v, xm, ym, theta, vm = X
    y = X[2] + X[6]
    v = w - y + dw
    return np.array([v])

def control_curve(X):
    x, y, delta, v, xm, ym, theta, vm = X
    y = delta+ np.arctan(y)
    u = y = sin(delta)/(1+y*y)
    return u
def main(): 

    X = np.array([0,2,2,1, 0, 0, 0, 0 ])
    fig = plt.figure()
    
    dt = 0.01
    
    #ax = init_figure(-40,40,-40,40)
    

    for t in np.arange(0,10,dt):
        #ax.cla()
        w = np.pi/2
        dw = 0  
        
        u=control(X,w, dw) 
        #u = control_curve(X)
        Xdot=f(X,u)
        X  = X + dt*Xdot
        if error(X) > 0 :
            X[4:6] = X[0] - boot*np.cos(X[6]), X[1] - boot*np.sin(X[6])
        
        draw_boat_and_magneto(X)
        #print(error(X))
        
        plt.pause(dt)  

    plt.show()

    print("Finished correctly")
    return 0

if __name__ == '__main__':
    main()
