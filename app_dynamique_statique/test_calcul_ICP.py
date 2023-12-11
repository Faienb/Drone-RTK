import numpy as np
import math
import Test_Ransac_pseudoICP_RPI as Ransac
import matplotlib.pyplot as plt


def gisement2(x1,y1,x2,y2):
    delta_x=x2-x1
    delta_y=y2-y1
    gis = math.atan2(delta_x,delta_y)
    if gis<0:
        gis+=(2*math.pi)
    return gis

def distance(point1,point2):
    d=math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
    return d

def make_P2(res_filename):
    with open(res_filename) as f:
        line = f.readline(); line = f.readline(); line = f.readline()
        P_global = np.array([])
        P_local = np.array([])
        while line:
            line = f.readline()
            if '$GNSS' in line:
                data = line.split(';')
                P_global = np.append(P_global,[float(data[4]),float(data[5]),float(data[6]),float(data[7])])
            if 'Tacheo' in line:
                data = line.split(';')
                #test nettoyage P_local pour enlever les points tacheo qui ont la même distance que celui d'avant
                #dist_tacheo = float(data[6])
                #if abs(dist_tacheo-dist_tacheo_avant)>=0.001:
                P_local = np.append(P_local,[float(data[7]),float(data[1]),float(data[2]),float(data[3]),float(data[4]),float(data[5]),float(data[6])])
                #dist_tacheo_avant = dist_tacheo
    return P_global, P_local

def filtre_bouge_pas_trop(P_global):
    point_avant = P_global[0]
    idx_to_remove =[]
    for i in range(1,len(P_global)):
        point = P_global[i]
        if distance(point[1:3], point_avant[1:3])<0.02:
            idx_to_remove.append(i)
        point_avant = P_global[i]
    P_lglobal = np.delete(P_global,idx_to_remove,axis=0)
    return P_lglobal

def calcul_residus_orientation(P_local,P_local_trans,G0):
    residus_orientation = []
    distance_plot = []
    somme = 0
    x_st = 2540621.753
    y_st = 1181302.435
    #x_st = P_local_trans[-1][0]
    #y_st = P_local_trans[-1][1]
    for mesure,coord in zip(P_local,P_local_trans):
        residus_orientation.append((gisement2(x_st,y_st,coord[0],coord[1])-mesure[4])*200/math.pi)
        d = distance(P_local_trans[-1],coord)
        distance_plot.append(d)
        somme = somme + d * (gisement2(x_st,y_st,coord[0],coord[1])-mesure[4])
    gisement_prop = somme/sum(distance_plot)*200/math.pi
    std = np.std(residus_orientation)
    return residus_orientation, distance_plot, gisement_prop, std
def filtre_detect_pause(P_global):#10cm
    point_avant = P_global[0]
    idx_to_remove =[]
    for i in range(1,len(P_global)):
        point = P_global[i]
        if distance(point[1:3], point_avant[1:3])>0.10:#seuil à 20cm/sec pour que ce soit considérer comme une pause
            idx_to_remove.append(i)
        point_avant = P_global[i]
    P_lglobal = np.delete(P_global,idx_to_remove,axis=0)
    return P_lglobal
def filtre_data_tacheo(P_global,P_local):
    new_P_local = np.array([])
    for point in P_global:
        time_GPS = point[0]
        for i in range(1,len(P_local)):
            point_tacheo_avant = P_local[i-1]
            point_tacheo = P_local[i]
            if point_tacheo[0]>time_GPS and point_tacheo[0]<time_GPS+0.5 and distance(point_tacheo_avant[1:3],point_tacheo[1:3])<=0.05:#car au moins deux fois plus de données tacheo que gps
                new_P_local = np.append(new_P_local,point_tacheo)
    new_P_local = new_P_local.reshape(int(len(new_P_local)/7),7)
    return new_P_local
'''
#ACQUISITION STATIQUE
res_filename = '3.txt'
P_global, P_local = make_P2(res_filename)
P_global = P_global.reshape(int(len(P_global)/4),4)
P_local = P_local.reshape(int(len(P_local)/7),7)
P_global = filtre_detect_pause(P_global)
P_local = filtre_data_tacheo(P_global,P_local)
threshold = 0.02

x_res,y_res,z_res,g0, fitness, inlier_rmse, len_correspondence, P_local_trans, correspondence = Ransac.ICP(P_global, P_local, threshold)

moyenne_z_GPS = np.mean(P_global[:,3])
moyenne_z_tache = np.mean(P_local[:,3])
trans_z = moyenne_z_tache-moyenne_z_GPS-0.05
z = 3000-trans_z
erreur = np.array([float(x_res),float(y_res),float(z_res)])-np.array([2540621.753,1181302.435,448.683])

residus_orientation, distance_plot, gisement_prop, std = calcul_residus_orientation(P_local,P_local_trans,float(g0))
print('Erreur en X : ',erreur[0],' Erreur en Y : ', erreur[1], 'Erreur en Z : ', erreur[2], ' Est : ', x_res,' Nord : ', y_res,' Altitude : ', z_res, ' Orientation : ', g0, 'Orientation prop : ',gisement_prop, 'std : ', std)
x = range(len(P_local))

fig, ax1 = plt.subplots()

ax2 = ax1.twinx()
ax1.plot(x, residus_orientation, 'g-')
ax2.plot(x, distance_plot, 'b-')

ax1.set_xlabel('Numéro de mesure')
ax1.set_ylabel('Orientation', color='g')
ax2.set_ylabel('Distance à la station', color='b')

#plt.plot(x, residus_orientation, label='Résidus en orientation')
#plt.legend()
plt.show()
'''


#ACQUISITION DYNAMIQUE
res_filename = '7.txt'
P_global, P_local = make_P2(res_filename)
P_global = P_global.reshape(int(len(P_global)/4),4)
P_local = P_local.reshape(int(len(P_local)/7),7)
threshold = 0.10

x_res,y_res,z_res,g0, fitness, inlier_rmse, len_correspondence, P_local_trans, correspondence = Ransac.ICP(P_global, P_local, threshold)
z_res = format(float(z_res)-0.05,'.3f')#ecart_antenne_prisme
'''moyenne_z_GPS = np.mean(P_global[:,3])
moyenne_z_tache = np.mean(P_local[:,3])
trans_z = moyenne_z_tache-moyenne_z_GPS-0.05
z = 3000-trans_z'''
erreur = np.array([float(x_res),float(y_res),float(z_res)])-np.array([2540621.753,1181302.435,448.683+0.237])

residus_orientation, distance_plot, gisement_prop, std = calcul_residus_orientation(P_local,P_local_trans,float(g0))
print('Erreur en X : ',erreur[0],' Erreur en Y : ', erreur[1], 'Erreur en Z : ', erreur[2], ' Est : ', x_res,' Nord : ', y_res,' Altitude : ', z_res, ' Orientation : ', g0, 'Orientation prop : ',gisement_prop, 'std : ', std)
x = range(len(P_local))

fig, ax1 = plt.subplots()

ax2 = ax1.twinx()
ax1.plot(x, residus_orientation, 'g-')
ax2.plot(x, distance_plot, 'b-')

ax1.set_xlabel('Numéro de mesure')
ax1.set_ylabel('Orientation', color='g')
ax2.set_ylabel('Distance à la station', color='b')

#plt.plot(x, residus_orientation, label='Résidus en orientation')
#plt.legend()
plt.show()
