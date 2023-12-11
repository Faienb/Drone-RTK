#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
import math
import itertools as it
import json
import open3d as o3d
import copy


def calcul_normals(liste_points):
    '''
    Calcul des normales de points d'un nuage de points

    Parameters
    ----------
    liste_points : numpy array of float
        liste de coordonnées
            X : Coordonnée Est [m].
            Y : Coordonnée Nord [m].
            Z : 0.
    Returns
    -------
    normals : numpy array
        Matrice des normales pour un nuage de points

    '''
    normals = np.array([[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1]])#ajout des normales pour les 5 premiers points
    for i in range(5,len(liste_points)-5):
        normal = np.cross([liste_points[i+5][0]-liste_points[i-5][0],liste_points[i+5][1]-liste_points[i-5][1],0],[0,0,1])
        normals = np.append(normals,[normal],axis = 0)
    normals = np.append(normals,[[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1]],axis=0)#ajout des normales pour les 5 derniers points
    return normals

def create_corres(P_local, P_global):
    '''
    Calcul des correspondances initiales entre les points du nuage local mesuré par la station totale et du nuage global mesuré par le GNSS

    Parameters
    ----------
    P_local : numpy array
        liste des coordonnées locales mesurées par station totale :
            X : Coordonnée Est locale [m].
            Y : Coordonnée Nord locale [m].
            Z : 0, l'altitude n'est pas transformée sur cette liste.
    P_global : numpy array
        liste des coordonnées globales mesurées par GNSS :
            X : Coordonnée Est transformée [m].
            Y : Coordonnée Nord transformée [m].
            Z : 0, l'altitude n'est pas transformée sur cette liste.

    Returns
    -------
    corres : open3D.utility.Vector2iVector
        Set de coorespondance d'index entre les 2 sets de nuages de points

    '''
    i=0
    corres = np.array([],dtype=np.int32)
    for point in P_global:
        time_GPS = point[0]
        idx = (np.abs(P_local[:,0]-time_GPS)).argmin()
        corres = np.append(corres,[idx,i])
        i+=1
    corres = corres.reshape(int(len(corres)/2),2)
    corres = o3d.utility.Vector2iVector(corres)#pour utiliser le format de open3d
    return corres

#Seulement utilisable sur un ordinateur (pas RPI), pour voir en 3D le résultat de l'ICP
def draw_registration_result(source, target, transformation):
    '''
    Fonction de développement de controle par plot de la tranformation de coordonnées locale vers globale par ICP Point to plane 2D\n
    Non utilisable sur RaspberryPi4

    Parameters
    ----------
    source : open3d.geometry.PointCloud
        Nuage de points Open3D, coordonnées locales
    target : o3d.geometry.PointCloud
        Nuage de points Open3D, coordonnées globales
    transformation : open3d.pipelines.registration.registration_ransac_based_on_correspondence.transformation
        Paramètres de transformation ICP 2D Point to plane Open3D

    Returns
    -------
    None.

    '''
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def ICP(P_global, P_local, threshold):
    '''
    Calcul de l'ICP point to point entre un set de point globaux mesurés par GNSS et un set de points locaux mesurés par station totale\n
    L'algorithme ICP point to point utilisé est celui de la librairie Open3D

    Parameters
    ----------
    P_global : numpy array
        liste des coordonnées globales mesurées par GNSS :
            X : Coordonnée Est transformée [m].
            Y : Coordonnée Nord transformée [m].
            Z : 0, l'altitude n'est pas transformée sur cette liste.
    P_local : numpy array
        liste des coordonnées locales mesurées par station totale :
            X : Coordonnée Est locale [m].
            Y : Coordonnée Nord locale [m].
            Z : 0, l'altitude n'est pas transformée sur cette liste.
    threshold : Seuil d'interruption des itérations de calcul de l'ICP
        DESCRIPTION.

    Returns
    -------
    X : float .3f
        Coordonnée Est de la station totale issue des résultats de l'ICP point to point [m]
    Y : float .3f
        Coordonnées Nord de la station totale issue des résultats de l'ICP point to point [m]
    Z : float .3f 
        Altitude de la station totale isssue des résultats de l'ICP point to point [m]
    g0 : float .4f
        Inconnue d'orientation de la station totale issue des résultats de l'ICP point to point
    fitness : float .1f
        Correspondance entre le set local et global après transformation exprimé en %
    RMSE : float .1f
        Ecart quadratique moyen issu des paramètres de transformation de l'ICP [mm]
    n : str
        Nombre de points de correspondance utilisé dans la transformation
    source_trans : open3D pointCloud
        Nuage de points de coordonnées locales transformés sur la base des paramètres de transformation de l'ICP
    correspondence_set : numpy array of int
        Matrice des index de correspondance entre les deux sets de mesure

    '''
    source_points = copy.deepcopy(P_local[:,1:4]) #point tacheo
    target_points = copy.deepcopy(P_global[:,1:4]) #points GPS
    #réduction des coord pour limiter la perte de précision lors des calculs matriciels
    dx_source = max(source_points[:,0])
    dy_source = max(source_points[:,1])
    dz_source = max(source_points[:,2])
    dx_target = max(target_points[:,0])
    dy_target = max(target_points[:,1])
    dz_target = max(target_points[:,2])
    source_points[:,0] = source_points[:,0] - dx_source
    target_points[:,0] = target_points[:,0] - dx_target
    source_points[:,1] = source_points[:,1] - dy_source
    target_points[:,1] = target_points[:,1] - dy_target
    source_points[:,2] = 0
    target_points[:,2] = 0
    threshold = threshold

    #initial alignement with ransac correspondence
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_points))
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_points))
    corres = create_corres(P_local,P_global)
    max_corr_distance = 0.50 #grossier, premier allignement
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_correspondence(source, target,corres,max_corr_distance)
    print("Initial alignment")
    print(result_ransac)
    trans_init = result_ransac.transformation
    evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
    print(evaluation)
    #draw_registration_result(source, target,result_ransac.transformation)
    #matrix_transfo = result_ransac.transformation
    #print(matrix_transfo)


    print('Calcul normal on target')
    normals = calcul_normals(target_points)
    target.normals = o3d.utility.Vector3dVector(normals)
    target.normalize_normals()

    #o3d.visualization.draw_geometries([target],point_show_normal=True)
    print("Point-to-plane ICP registration")
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPlane())#, o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
    print(reg_p2p)
    print("Transformation is:")
    matrix_transfo = reg_p2p.transformation
    print(matrix_transfo)
    fitness = reg_p2p.fitness
    rmse = reg_p2p.inlier_rmse
    correspondence_set = np.asarray(reg_p2p.correspondence_set)
    correspondence_set_len = len(reg_p2p.correspondence_set)#Use numpy.asarray() to access data
    #draw_registration_result(source, target,reg_p2p.transformation)
    station = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.array([[1000-dx_source,2000-dy_source,0]])))
    station = station.transform(matrix_transfo)
    station = np.asarray(station.points)+np.array([dx_target,dy_target,0])
    source = source.transform(matrix_transfo)
    source_trans = np.asarray(source.points)+np.array([dx_target,dy_target,0])

    #print(coord_3D)
    #coord_3D = np.asarray(coord_3D) + matrix_transfo[:,3]#Pour être vraiment rigoureux il faudrait appliquer la rotation sur cette coordonnée aussi, ou alors mettre la coordonnée globale approchée de la station dans l'ICP aussi
    alpha_supp = math.asin(matrix_transfo[0][1]) #rad #Pour être rigoureux, vérifier si cet angle est le bon en calculant les autres éléments de la matriciels
    alpha_supp2 = math.acos(matrix_transfo[0][0])
    if alpha_supp < 0.0 and alpha_supp2 < np.pi/2 :
        alpha_supp = 2*np.pi + alpha_supp
    elif alpha_supp < 0.0 and alpha_supp2 > np.pi/2 :
        alpha_supp = np.pi - alpha_supp
    else :
        alpha_supp = alpha_supp2
    #print(alpha_supp*200/math.pi)
    #print(np.asarray(source.points)[-1]+np.array([dx_source,dy_source,0])-np.array([2540621.753,1181302.435,0]))
    coord_3D_ICP = station[-1]#-np.array([2540621.753,1181302.435,0])

    #test ICP sur Altitude
    source_points = copy.deepcopy(P_local[:,[0,1,3]]) #point tacheo
    target_points = copy.deepcopy(P_global[:,[0,1,3]]) #points GPS
    #####################reduction des coord pour limiter la perte de précision lors des calculs matriciels################
    dx_source = max(source_points[:,0])#temps
    dy_source = max(source_points[:,1])#x
    dz_source = max(source_points[:,2])#z
    dx_target = max(target_points[:,0])
    dy_target = max(target_points[:,1])
    dz_target = max(target_points[:,2])



    #source_points = np.append(source_points, np.array([[0,0,3000]]), axis = 0)
    source_points[:,0] = source_points[:,0] - dx_source
    target_points[:,0] = target_points[:,0] - dx_target
    source_points[:,1] = 0
    target_points[:,1] = 0
    source_points[:,2] = source_points[:,2] - dz_source
    target_points[:,2] = target_points[:,2] - dz_target
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_points))
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_points))
    trans_init = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    #draw_registration_result(source, target,trans_init)
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, 0.05, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    matrix_transfo = reg_p2p.transformation
    print(matrix_transfo)
    #draw_registration_result(source, target,reg_p2p.transformation)
    station_z = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.array([[0,0,3000-dz_source]])))
    station_z = station_z.transform(matrix_transfo)
    source_trans_z = np.asarray(station_z.points)+np.array([dx_target,0,dz_target])
    z = source_trans_z[-1][2]
    #coord3D = []
    #coord_3D,dx,alpha = MoindresCarresHelmert2D(listePoints4L2,pos_st_local,z_st_local, distance_residus)
    #gis_loin = calcul_gisement(coord_3D, listePoints4L2)#pas mieux...
    return format(coord_3D_ICP[0],".3f"), format(coord_3D_ICP[1],".3f"),format(z,".3f"), format(alpha_supp*200/math.pi,".4f"), format(fitness*100,".1f"), format(rmse*1000,".1f"), str(int(correspondence_set_len)), source_trans, correspondence_set
