import numpy as np
import time
import random, sys
from PyQt5.QtCore import QPoint, QRect, QSize, Qt,QLine,QLineF,QPointF
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
"""
Point sur le ponton de guerlédan 48.199236, -3.014598  48.199231, -3.014573
Le même point en coordonnées pxiels sur l'image dans python (1689, 395)
image : 1906 933
On a une echelle de 50m:
1855-1795 = 60
"""
lat0= 48.199236
lon0=-3.014573

EARTH_RADIUS = 6371000.
OVERSHOOT = 20. #Dépassement total (on dépasse de overshoot/2 de chaque côté de la ligne) (en m)
LINE_SPACE = 8. #Ecart entre chaque ligne (en m)


class Window(QWidget):

    def __init__(self, parent = None):
        super().__init__()
        self.Points_trap=[]
        self.PointsF_trap=[]
        self.Lines_trap=[]
        self.type_trap=0 #Type = 1 trapèze avec paralleles horizontal, verticale pour type = 2
        self.MaxY=933
        self.rubberBand = QRubberBand(QRubberBand.Rectangle, self)
        self.origin = QPoint()
        # self.label = QLabel(self)
        self.pixmap = QPixmap('Lac_guerledan_petit_2.png')
        self.pixmap = self.pixmap.scaled(1600,750, Qt.KeepAspectRatio)

        #[self.width, self.height] = self.pixmap.size
        #print(self.pixmap.height())
        # pixSize.scale(self.size(), Qt.KeepAspectRatio)

        # Qrectgeo=QRect(50,50,1906,933)
        self.setWindowTitle("Lac de Guerlédan")
        self.setGeometry(0,0,self.pixmap.width(), self.pixmap.height())

        print(self.size())

        # self.label.setPixmap(self.pixmap)

        #self.resize(1800,800)

        # self.resize(self.pixmap.width(),self.pixmap.height())


        # oImage = QImage("Lac_guerledan_petit_2.png")
        # #sImage = oImage.scaled(QSize(1870,949))                   # resize Image to widgets size
        # palette = QPalette()
        # palette.setBrush(QPalette.Background, QBrush(oImage))                     # 10 = Windowrole
        # self.setPalette(palette)

        # self.label = QLabel('Test', self)                        # test, if it's really backgroundimage
        # self.label.setGeometry(50,50,200,50)




    def mousePressEvent(self, event):
        painter = QPainter(self.pixmap)
        # painter.begin(self.pixmap)
        print("ondessine")
        pen = QPen(Qt.green, 5)
        painter.setPen(pen)

        if event.button() == Qt.LeftButton:
            if len(self.Points_trap)==0:
                print("ajout du premier point")
                self.Points_trap.append(event.pos())
                self.PointsF_trap.append(QPointF(event.pos()))
                self.update()
            elif len(self.Points_trap)==1:
                print("ajout du deuxieme point")
                Point2=event.pos()
                Point1=self.Points_trap[0]
                # print(event.pos().setY(self.Points_trap[0].y()))
                if abs(Point2.x()-Point1.x())>=abs(Point2.y()-Point1.y()):
                    Point2.setY(self.Points_trap[0].y())
                    self.type_trap=1
                else :
                    Point2.setX(self.Points_trap[0].x())
                    self.type_trap=2
                self.Points_trap.append(Point2)
                self.Lines_trap.append(QLineF(self.Points_trap[0],Point2))
                self.PointsF_trap.append(QPointF(Point2))
                self.update()
            elif len(self.Points_trap)==2:
                print("ajout du troisieme point")
                self.Points_trap.append(event.pos())
                self.Lines_trap.append(QLineF(self.Points_trap[1],self.Points_trap[2]))
                self.PointsF_trap.append(QPointF(event.pos()))
                self.update()
            elif len(self.Points_trap)==3:
                Point4=event.pos()
                print("Ajout du quatrième et dernier point")
                Point3=self.Points_trap[2]
                # print(event.pos().setY(self.Points_trap[0].y()))
                if self.type_trap==1:
                    Point4.setY(self.Points_trap[2].y())
                else :
                    Point4.setX(self.Points_trap[2].x())
                self.Points_trap.append(Point4)
                self.Lines_trap.append(QLineF(self.Points_trap[2],Point4))
                self.Lines_trap.append(QLineF(self.Points_trap[3],self.Points_trap[0]))
                self.PointsF_trap.append(QPointF(Point4))
                self.update()

            for i in range(len(self.Points_trap)):
                painter.drawPoint(self.Points_trap[i])
            pen = QPen(Qt.red, 2)
            painter.setPen(pen)

            painter.drawLines(self.Lines_trap)
            painter.end()
            if len(self.Points_trap)>=4:
                    print("On a les 4points")
                    print(self.Points_trap)

                    self.createfile()


            # painter.drawPoints(self.PointsF_trap)

            # self.origin = QPoint(event.pos())
            # print(self.Points_trap)
            # self.rubberBand.setGeometry(QRect(self.origin, QSize()))
            # self.rubberBand.show()
        if event.button() == Qt.RightButton:
            painter.end()
            self.Points_trap=[]
            self.Lines_trap=[]
            self.type_trap=0
            self.pixmap = QPixmap('Lac_guerledan_petit_2.png')
            self.pixmap = self.pixmap.scaled(1600,750, Qt.KeepAspectRatio)
            self.repaint()
            print(self.Points_trap)


    # def virage(self,end_line_point, new_line_begin_nvert_to_lat_longpoint): # les lignes sont supposées longitudinales
    # 	dist = new_line_begin_point[1] - end_line_point[1]
    # 	virage_wp = [(end_line_point[0]+dist/4, end_line_point[1]+dist/4)]
    # 	virage_wp.append((virage_wp[0][0], virage_wp[0][1]+dist/2))
    # 	return virage_wp

    def genere_one_bloc(self, overshoot=OVERSHOOT, line_space=LINE_SPACE):
        Point1 = self.Points_trap[0]
        Point2 = self.Points_trap[1]
        Point3 = self.Points_trap[2]
        Point4 = self.Points_trap[3]
        print(Point1)
        print(Point2)
        print(Point3)
        print(Point4)
        if self.type_trap==1:
            Height = Point3.y()-Point1.y()
            Length = Point1.x()-Point2.x()
        elif self.type_trap==2:
            Height = Point3.x()-Point1.x()
            Length = Point1.y()-Point2.y()
        dx_d = Point3.x()-Point2.x() #difference en x gauche
        dx_g = Point4.x()-Point1.x() #difference en x droite
        # WP_L93 = [(self.Points_trap[0].x(), self.Points_trap[0].y())]
        # WP_L93.append((self.Points_trap[1].x(), self.Points_trap[1].y()))
        # WP_L93.append((self.Points_trap[2].x(), self.Points_trap[2].y()))
        # WP_L93.append((self.Points_trap[3].x(), self.Points_trap[3].y()))
    	# print(WP_L93)
        WP_L93=[]
        xg=[]
        yg=[]
        xd=[]
        yd=[]
        nb_lignes = abs(int(Height/line_space))
        # print("nombre de lignes :",nb_lignes)
        line_space = Height/nb_lignes
        # print(line_space)
        for n in range(nb_lignes):
            xg.append(self.Points_trap[0].x()+dx_g*n/nb_lignes)
            yg.append(self.Points_trap[0].y()+line_space*n)
            # print(n)
            xd.append(self.Points_trap[1].x()+dx_d*n/nb_lignes)
            yd.append(self.Points_trap[1].y()+line_space*n)
        # print(xg)
        # print("yhgga")
        # print(yg)
        for n in range(int(nb_lignes/2)):


            WP_L93.append((xg[n],yg[n]))
            WP_L93.append((xd[n],yd[n]))
            mediap1 = int(nb_lignes/2)+n
            WP_L93.append((xd[mediap1],yd[mediap1]))
            WP_L93.append((xg[mediap1],yg[mediap1]))

        if (nb_lignes%2)!=0:#Cas ou c'est impair
            ind_middle = int(nb_lignes/2)
            WP_L93.append((xg[ind_middle],yg[ind_middle]))
            WP_L93.append((xd[ind_middle],yd[ind_middle]))


        # for n in range(nb_lignes):
        #     if n%2 == 0: #From west to east
        #         WP_L93.append((WP_L93[-1][0]+Length+overshoot, WP_L93[-1][1]))
        #         if n>0 :Length -= (Point1.x()-Point4.x())/nb_lignes *(n-1)
        #         next_line_begin_point = (WP_L93[-1][0]+(Point3.x()-Point2.x())/2, WP_L93[-1][1] + int(nb_lignes/2+1)*line_space)
        #         Length += ((Point3.x()-Point2.x())/nb_lignes) * ((nb_lignes/2+1)+n)
        #         #WP_L93 += virage(WP_L93[-1], next_line_begin_point)
        #         WP_L93.append(next_line_begin_point)
        #     else: #From east to west
        #         WP_L93.append((WP_L93[-1][0]-Length-overshoot, WP_L93[-1][1]))
        #         Length -= ((Point3.x()-Point2.x())/nb_lignes) * (int(nb_lignes/2+1)+n-1)
        #         next_line_begin_point = (WP_L93[-1][0]-(Point1.x()-Point4.x())/2, WP_L93[-1][1] - int(nb_lignes/2)*line_space)
        #         Length += (Point1.x()-Point4.x())/nb_lignes *n
        #         # WP_L93 += virage(WP_L93[-1], next_line_begin_point)
        #         WP_L93.append(next_line_begin_point)
        #         WP_WGS84 = []
    	# for WP in WP_L93[:-3]: #Supprime le dernier virage
        # print(WP_L93)

        painter = QPainter(self.pixmap)
        # painter.begin(self.pixmap)
        pen = QPen(Qt.blue, 4)
        painter.setPen(pen)
        for i in range(len(WP_L93)):
            painter.drawPoint(WP_L93[i][0],WP_L93[i][1])


        WP_WGS84 = []
        for WP in WP_L93:
            # print(WP)
            WP_WGS84.append(cart_to_WGS84(WP[0]-1689,-WP[1]+395)) #(1689, 395)
        # print(WP_WGS84)
        # print(WP_L93)
        return WP_WGS84
    def createfile(self):
        n_bloc = 1

        with open("guerledan/guerledanv1_" + str(n_bloc) + ".gpx", 'w') as f:
        	with open("template.gpx", 'r') as template_f:
        		for l in template_f:
        			if l.find("$NAME") >= 0:
        				f.write(l.replace("$NAME", "brique_" + str(n_bloc)))
        			elif l.find("$WP") >= 0:
        				WP = self.genere_one_bloc()
        				for i in range(len(WP)):
        					with open("WP.gpx", 'r') as wp_f:
        						for l_wp in wp_f:
        							if l_wp.find("$LAT") >= 0:
        								line_with_lat = l_wp.replace("$LAT", str(WP[i][0]))
        								f.write(line_with_lat.replace("$LON", str(WP[i][1])))
        							elif l_wp.find("$NAME") >= 0:
        								f.write(l_wp.replace("$NAME", str(i+1)))
        							else:
        								f.write(l_wp)
        			else:
        				f.write(l)


    def paintEvent(self, Qpaintevent):
        painter = QPainter(self)

        painter.drawPixmap(0,0, self.pixmap)

        painter.end()

        # painter = QPainter(self.pixmap)
        # # painter.begin(self.pixmap)
        # print("ondessine")
        # pen = QPen(Qt.green, 5)
        # painter.setPen(pen)
        #
        # for i in range(len(self.Points_trap)):
        #     painter.drawPoint(self.Points_trap[i])
        #
        # # painter.drawPoints(self.PointsF_trap)
        # pen = QPen(Qt.red, 2)
        # painter.setPen(pen)
        #
        # painter.drawLines(self.Lines_trap)



def convertCoordMeters(x,y):
    """ Convert x,y points into lat and long"""
    return (50*x)/60, (50*y)/60

def WGS84_to_cart(lat, lon):
	x = (np.pi/180.)*EARTH_RADIUS*(lon-lon0)*np.cos((pi/180.)*lat)
	y = (np.pi/180.)*EARTH_RADIUS*(lat-lat0)
	return x, y

def cart_to_WGS84(x, y):
    x,y= convertCoordMeters(x,y)
    EPSILON=0.00000000001
    lat = y*180./np.pi/EARTH_RADIUS+lat0
    if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
        lon = 0
    else:
        lon = (x/EARTH_RADIUS)*(180./np.pi)/np.cos((np.pi/180.)*(lat))+lon0
    return lat, lon

if __name__ == "__main__":
    print("Veuillez a bien faire attention à commencer par donner les points qui indique les droites parrallèles.")
    random.seed()
    app = QApplication(sys.argv)
    window = Window()
    # window.setPixmap(create_pixmap())
    # window.resize(400, 300)
    window.show()

    sys.exit(app.exec_())
