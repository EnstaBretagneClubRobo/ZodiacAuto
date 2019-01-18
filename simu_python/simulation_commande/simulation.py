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
        self.delta = X[2] # angle entre l'axe des abscisses et le zodiac 
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
    #
    # dessine le bateau et le magnétomètre
    #
    x, y, delta, v, xm, ym, theta, vm = X
    #[x , y , delta , v] = X[0:4]  #state vector robot/zodiac
    #[xm, ym, theta, vm ]= X[4:8]
    #state vector magnetometer
    #Robot
    absc_r = [x+L*np.cos(delta), x+l/2*np.sin(delta), x-l/2*np.sin(delta), x+L*np.cos(delta)]
    ordo_r = [y+L*np.sin(delta), y-l/2*np.cos(delta), y+l/2*np.cos(delta), y+L*np.sin(delta)]
    #Wire
    absc_w = [x,xm]
    ordo_w = [y,ym]
    #magnetometer
    absc_m = [xm]
    ordo_m = [ym]

    #Affichage
    plt.plot(absc_r, ordo_r, 'r' , linewidth = 1.0)
    #plt.plot(absc_w, ordo_w, 'b' , linewidth = 1.0)
    #plt.plot(absc_m, ordo_m, 'bo', markersize = 1)
    return 0

def f(X,u):
    #
    # fonction d'évolution de la remorque et du zodiac
    #
    x, y, delta, v, xm, ym, theta, vm = X
    #epsilon = 0.1 # marge d'erreur pour le cable

    dx = v*np.cos(delta)
    dy = v*np.sin(delta)
    dv = 0
    ddelta = u
     # a refaire, probleme de confusion dans les def de theta et delta
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
    #
    # retourne la distance entre la remorque et le bout de la corde du zodiac
    #
    x, y, delta, v, xm, ym, theta, vm = X
    xl, yl = x - boot*np.cos(theta), y - boot*np.sin(theta) # coordonnees limite de la remorque
    return np.sqrt((xm-xl)*(xm-xl)+(ym-yl)*(ym-yl)) # distance entre les coords limite et calculé de la remorque


def control(X,w, dw):
    #
    #linearization feedback
    #
    x, y, delta, v, xm, ym, theta, vm = X
    y = X[2] 
    u =     (w ) + dw 
    return u

def control_curve(X):
    x, y, delta, v, xm, ym, theta, vm = X
    cap_zodiac = delta  # cap réel du zodiac entre les abscisses et le zodiac
    
    u = -sawtooth(cap_zodiac-np.arctan2(-(x*x/100-1)*y-x, y)) + (((x*x/100-1)*y+x)*np.sin(cap_zodiac)+y*(x*y*np.cos(cap_zodiac)/50+(x*x/100-1)*np.sin(cap_zodiac)+np.cos(cap_zodiac)))/(y*y+((x*x/100-1)*y+x)*((x*x/100-1)*y+x))
    
    return u

def control_traj_sin(X):
    #
    # renvoie le controle pour un suivi de sinus avec le slalom_paper
    #   
    x, y, delta, v, xm, ym, theta, vm = X
    bruit = np.random.rand(1)
    cap_zodiac = delta+ bruit[0]/10 # cap mesuré avec du bruit
    a = 1
    b = np.cos(x/10) + np.sin(x/10) - y/10
    da = 0
    db = 0.1*np.cos(cap_zodiac)*(np.cos(x/10)-np.sin(x/10))-0.1*np.sin(cap_zodiac)
    u = -sawtooth(cap_zodiac-np.arctan2(b,a))-(b*da-a*db)/(a*a+b*b)

    return u


def control_traj_circle(X):
    #
    # renvoie le controle pour un suivi de cercle avec le slalom_paper
    #
    x, y, delta, v, xm, ym, theta, vm = X
    bruit = np.random.rand(1)
    cap_zodiac = delta
    a = 1
    b = (1/20)*(x*x+y*y-5)-x
    da =    0
    
    try:
        db = (1/20)*(1/y)*(2*x*np.cos(cap_zodiac)-2*y*np.sin(cap_zodiac))-np.cos(cap_zodiac)
    except ValueError as err: 
        print("Division par zero  :", err)
        db = 0 
    u = -sawtooth(cap_zodiac-np.arctan2(b,a))-(b*da-a*db)/(a*a+b*b)


    return u




def champ_vect(p):
    x,y = p # van der pol vector field attractor
    return np.array([y, -(0.01*x*x-1)*y-x])

def main(): 

    X = np.array([1,1,0,1, 0, 0, 0, 0 ])
    fig = plt.figure()
    
    dt = 1
    
    k = np.linspace(0, 300, 100)
    plt.plot(k, 10*np.sin(k/10))    
    
    for t in np.arange(0,1000,dt):
        u = control_traj_sin(X)
        dw = 0
        
        #u=control(X,w, dw) # controle avec Fb linearization
        u = control_curve(X) # van der pols raté
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
