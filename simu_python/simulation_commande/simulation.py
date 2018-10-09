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
    plt.plot(absc_r, ordo_r, 'r' , linewidth = 1.0)
    plt.plot(absc_w, ordo_w, 'b' , linewidth = 1.0)
    plt.plot(absc_m, ordo_m, 'bo', markersize = 1)
    return 0

def f(X,u):
    # fonction d'évolution de la remorque et du zodiac
    x, y, delta, v, xm, ym, theta, vm = X
    #epsilon = 0.1 # marge d'erreur pour le cable

    dx = v*np.cos(theta+delta)
    dy = v*np.sin(theta+delta)
    dv = 0
    ddelta = u
     
    if error(X) >= 0 : # garde la remorque à la bonne distance du zodiac
            #dxl, dyl =dx - boot*dtheta*np.sin(theta), dy - boot*dtheta*np.cos(theta)
            dxm, dym, dtheta, dvm = v*np.cos(delta)*np.sin(theta), v*np.cos(delta)*np.sin(theta), v*np.sin(delta), v*np.cos(delta)

    
    if error(X) < 0 : # diminue la vitesse de la remorque si le cable n'est pas tendu

        dxm, dym, dtheta, dvm = vm*np.cos(theta), vm*np.sin(theta),0, 0.3*vm

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
    #linearization feedback
    x, y, delta, v, xm, ym, theta, vm = X
    y = X[2] + X[6]
    u = w - y + dw - v*sin(delta)
    return u

def control_curve(X):
    x, y, delta, v, xm, ym, theta, vm = X
    cap_zodiac = delta + theta # cap réel du zodiac entre les abscisses et le zodiac
    
    u = -sawtooth(cap_zodiac-np.arctan2(-(x*x/100-1)*y-x, y)) + (((x*x/100-1)*y+x)*np.sin(cap_zodiac)+y*(x*y*np.cos(cap_zodiac)/50+(x*x/100-1)*np.sin(cap_zodiac)+np.cos(cap_zodiac)))/(y*y+((x*x/100-1)*y+x)*((x*x/100-1)*y+x))
    
    return u

def traj_sin(X):
    x, y, delta, v, xm, ym, theta, vm = X
    bruit = np.random.rand(1)
    cap_zodiac = delta+ theta # + bruit[0]/10 # cap mesuré avec du bruit
    a = 1
    b = np.cos(x/10) + np.sin(x/10) - y/10
    da = 0
    db = np.cos(cap_zodiac)*(np.cos(x/10)-np.sin(x/10))/10 - np.sin(cap_zodiac)/10
    u = -sawtooth(cap_zodiac-np.arctan2(0.5*b,0.5*a)-(0.25*b*da-0.25*a*db)/(0.25*a*a+0.25*b*b))

    return u



def champ_vect(p):
    x,y = p # van der pol vector field attractor
    return np.array([y, -(0.01*x*x-1)*y-x])

def main(): 

    X = np.array([0,2,0,1, 0, 0, 0, 0 ])
    fig = plt.figure()
    
    dt = 1
    
    k = np.linspace(0, 300, 100)
    plt.plot(k, 10*np.sin(k/10))    

    for t in np.arange(0,1000,dt):
        w = traj_sin(X)
        dw = 0
        
        u=control(X,w, dw) # controle avec Fb linearization
        #u = control_curve(X) # van der pols raté
        #u = control_sin(X) 
        Xdot=f(X,u)
        X  = X + dt*Xdot
        if error(X) > 0 :
            X[4:6] = X[0] - boot*np.cos(X[6]), X[1] - boot*np.sin(X[6])
        
        draw_boat_and_magneto(X)
        
        x, y, delta, v, xm, ym, theta, vm = X
        
        plt.pause(dt)  

    plt.show()

    print("Finished correctly")
    return 0

if __name__ == '__main__':
    main()
