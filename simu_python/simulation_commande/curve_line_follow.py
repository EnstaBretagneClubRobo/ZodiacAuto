import numpy as np
import matplotlib.pyplot as plt
from roblib import *


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
    #absc_w = [x,xm]
    #ordo_w = [y,ym]
    #magnetometer
    #absc_m = [xm]
    #ordo_m = [ym]

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

    dxm, dym, dtheta, dvm = 0, 0, 0, 0


    return np.array([dx,
        dy,
        ddelta,
        dv,
        dxm,
        dym,
        dtheta,
        dvm])

def dist(x,y,wx, wy):
	#
	# retourne la distance entre deux points (x,y) (wx, wy)
	#
	return sqrt((x-wx)*(x-wx)+(y-wy)*(y-wy))

#liste des waypoints
waypoints = np.array([[0, 0], [10,0], [10,10], [20,20]]) 
# paramètre R pour l'interpolation
R = 5

def control_curve_line(X):
	#
	# renvoie un suivi de lignes et de courbes pour éviter les virages trop brutaux
	#
	x, y, delta, v, xm, ym, theta, vm = X #vecteur d'état
	i = 0
	print(i)
	while i < len(waypoints)-1:
		# tant qu'on  a pas parcouru tous les waypoints

		wx, wy = waypoints[i] # waypoint actuel
		wxs, wys = waypoints[i+1] #waypoint suivant
	
		if dist(x,y, wx, wy) < R :
			print('distance avec le waypoint', dist(x,y,wx, wy))
		# on interpole avec un polynome qui passe en trois points: 
		# le point d'entree du zod, le waypoint, et l'intersection entre 
		# le cercle de rayon R autour du waypoint actuel et la droite
		# qui va du waypoint au waypoint suivant
		# ...et on suit cette courbe

			#coeffs = coeffs_interpol(x,y, wx, wy, wxs, wys)
			a = 1
			b = 1
			da = 1
			db = 1
			u = -sawtooth(delta-np.arctan2(b,a))-(b*da-a*db)/(a*a+b*b)

		else: 
			# on suit le droite qui va du waypoint actuel au suivant

			m = (wys-wy)/(wxs-wx) # coeff directeur de la droite 
			p = wy - m*wx # ordonnee a l'origine

			a = 1
			b = m - m*x - p - y 
			da = 0
			db = - m*np.cos(delta) + np.sin(delta)

			u = -sawtooth(delta-np.arctan2(b,a))-(b*da-a*db)/(a*a+b*b)


			if (dist(x, y, wxs, wys) < R + 1) and (dist(x, y, wxs, wys) > R):
			# le waypoint actuel est le waypoint le plus proche en gros
			# on en change donc quand on se rapproche de trop près
				i = i + 1

	return u


def main(): 

    X = np.array([1,1,0,1, 0, 0, 0, 0 ])
    fig = plt.figure()
    
    dt = 1
    
    k = np.linspace(0, 300, 100)
    plt.plot(k, 10*np.sin(k/10))    
    print('début de simulation')

    for t in np.arange(0,1000,dt):
       
        
        print(t)
        #u=control(X,w, dw) # controle avec Fb linearization
        u = control_curve_line(X) # van der pols raté
        #u = control_sin(X) 
        print(X)
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