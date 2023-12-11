import numpy as np
import math
import open3d as o3d
import copy

def calcul_normals(liste_points):
    '''
    Fonction permettant de calculer les normales pour chaque points de la liste de points.

    Parameters
    ----------
        liste_points : list of Vector
            X : float
                Coordonnée Est du point [m].
            Y : float
                Coordonnée Nord du point [m].
            Z : float
                altitude du point [m].
    Returns
    -------
        normals : list of Vector
            X : float
                Coordonnée x de la normale [m].
            Y : float
                Coordonnée y de la normale [m].
            Z : float
                Coordonnée z de la normale [m].
    '''
    normals = np.array([[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1]])#ajout des normales pour les 5 premiers points
    for i in range(5,len(liste_points)-5):
        normal = np.cross([liste_points[i+5][0]-liste_points[i-5][0],liste_points[i+5][1]-liste_points[i-5][1],0],[0,0,1])
        normals = np.append(normals,[normal],axis = 0)
    normals = np.append(normals,[[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1]],axis=0)#ajout des normales pour les 5 derniers points
    return normals

def create_corres(P_local, P_global):
    '''
    Fonction permettant de Trouver une correspondence entre deux listes de points en fonction du temps.

    Parameters
    ----------
        P_local : list of Vector
            Time : float
                Temps GPS du point [s].
            X : float
                Coordonnée Est du point [m].
            Y : float
                Coordonnée Nord du point [m].
            Z : float
                Altitude RAN95 du point [m].
        P_global : list of Vector
            Time : float
                Temps GPS du point [s].
            X : float
                Coordonnée Est locale du point [m].
            Y : float
                Coordonnée Nord locale du point [m].
            Z : float
                Altitude locale du point [m].
            Hz : float
                Angle horizontal [rad].
            V : float
                Angle vertical [rad].
            Di : float
                Distance inclinée [m].
    Returns
    -------
        corres : list of Vector
            id_tacheo : int
                id du point tachéo correspondant au point GPS avec l'id_GPS [-].
            id_GPS : int
                id du point GPS correspondant au point tachéo avec l'id_tacheo [-].
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

def ICP(P_global, P_local, threshold):
    '''
    Fonction permettant de calculer une transformation en partant d'un set de points source (P_local) vers un set de points target (P_global).
    Cette fonction utilise le code de l'ICP de la librairie Open3D dans un premier temps en utilisant une ICP Point to Plane pour calculer une transformation planimétrique.
    Puis dans un seconde temps une ICP Point to Point pour calculer la transformation altimétrique (et accessoirement le décalage temporel entre les mesures GPS et tachéo).

    Parameters
    ----------
        P_global : list of Vector
            Time : float
                Temps GPS du point [s].
            X : float
                Coordonnée Est locale du point [m].
            Y : float
                Coordonnée Nord locale du point [m].
            Z : float
                Altitude locale du point [m].
            Hz : float
                Angle horizontal [rad].
            V : float
                Angle vertical [rad].
            Di : float
                Distance inclinée [m].
        P_local : list of Vector
            Time : float
                Temps GPS du point [s].
            X : float
                Coordonnée Est du point [m].
            Y : float
                Coordonnée Nord du point [m].
            Z : float
                Altitude RAN95 du point [m].
        threshold : float
                Seuil à partir du quel un point de la source est considéré comme correspondant à un point de la cible [m].
    Returns
    -------
        X : str
            Coordonnée Est transformée de la station [m].
        Y : str
            Coordonnée Nord transformée de la station [m].
        Z : str
            Altitude transformée (RAN95) de la station [m].
        alpha_supp : str
            Gisement du 0 du cercle de la station [gon].
        fitness : str
            Nombre de correspondences trouvées divisé par le nombre de points dans le set de points cible [%].
        rmse: str
            Précision calculée sur les points qui ont été appairés [mm].
        correspondence_set_len : str
            Nombre de couples de points correctement appairés [-].
        source_trans : list of Vector
            liste de la source transformée :
                X : Coordonnée Est transformée [m].
                Y : Coordonnée Nord transformée [m].
                Z : 0, l'altitude n'est pas transformée sur cette liste.
        correspondence_set : list of Vector
            id_source : int
                id du point source correspondant au point cible avec l'id_target [-].
            id_target : int
                id du point cible correspondant au point source avec l'id_source [-].

    '''
    #############################Calcul coord planimétrie#######################
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

    #Calcul normal on target
    normals = calcul_normals(target_points)
    target.normals = o3d.utility.Vector3dVector(normals)#utilise le format open3d
    target.normalize_normals()

    #ICP point to plane
    print("Point-to-plane ICP registration")
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPlane())#, o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
    print(reg_p2p)
    print("Transformation is :")
    matrix_transfo = reg_p2p.transformation
    print(matrix_transfo)
    fitness = reg_p2p.fitness
    rmse = reg_p2p.inlier_rmse
    correspondence_set = np.asarray(reg_p2p.correspondence_set)
    correspondence_set_len = len(reg_p2p.correspondence_set)

    #Transformation points tachéo et station
    station = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.array([[1000-dx_source,2000-dy_source,0]])))#utilise le format open3d
    station = station.transform(matrix_transfo)
    station = np.asarray(station.points)+np.array([dx_target,dy_target,0])
    source = source.transform(matrix_transfo)
    source_trans = np.asarray(source.points)+np.array([dx_target,dy_target,0])

    #Calcul du gisement
    alpha_supp = math.asin(matrix_transfo[0][1]) #rad
    alpha_supp2 = math.acos(matrix_transfo[0][0])
    if alpha_supp < 0.0 and alpha_supp2 < np.pi/2 :
        alpha_supp = 2*np.pi + alpha_supp
    elif alpha_supp < 0.0 and alpha_supp2 > np.pi/2 :
        alpha_supp = np.pi - alpha_supp
    else :
        alpha_supp = alpha_supp2
    coord_3D_ICP = station[-1]

    #########################ICP sur Altitude###################################
    source_points = copy.deepcopy(P_local[:,[0,1,3]]) #point tacheo
    target_points = copy.deepcopy(P_global[:,[0,1,3]]) #points GPS
    #Réduction des coord pour limiter la perte de précision lors des calculs matriciels
    dx_source = max(source_points[:,0])#temps
    dy_source = max(source_points[:,1])#x
    dz_source = max(source_points[:,2])#z
    dx_target = max(target_points[:,0])
    dy_target = max(target_points[:,1])
    dz_target = max(target_points[:,2])
    source_points[:,0] = source_points[:,0] - dx_source
    target_points[:,0] = target_points[:,0] - dx_target
    source_points[:,1] = 0
    target_points[:,1] = 0
    source_points[:,2] = source_points[:,2] - dz_source
    target_points[:,2] = target_points[:,2] - dz_target
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_points))#utilise le format open3d
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_points))

    trans_init = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])#matrice identité
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, 0.05, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    matrix_transfo = reg_p2p.transformation
    print(matrix_transfo)
    station_z = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.array([[0,0,3000-dz_source]])))
    station_z = station_z.transform(matrix_transfo)
    station_trans_z = np.asarray(station_z.points)+np.array([dx_target,0,dz_target])
    z = station_trans_z[-1][2]
    return format(coord_3D_ICP[0],".3f"), format(coord_3D_ICP[1],".3f"), format(z,".3f"), format(alpha_supp*200/math.pi,".4f"), format(fitness*100,".1f"), format(rmse*1000,".1f"), str(int(correspondence_set_len)), source_trans, correspondence_set
