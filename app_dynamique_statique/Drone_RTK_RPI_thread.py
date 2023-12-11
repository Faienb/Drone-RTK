#attention a activer la communication rs232 par Geocom sur l'appareil
#Utiliser  Test_Ransac du RPI sur le RPI
import time
import numpy as np
import serial
import math
import logging
import multiprocessing
import Test_Ransac_pseudoICP_RPI as Ransac
import json
import GeoComLibrary as com #libraire pour la communication GeoCom du tacheo
from datetime import datetime, timezone
from matplotlib.figure import Figure

TS = False #Je dois initialiser ca ici pour pas retourner d'erreurs dans les fonctions

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
def ell_param(a,f):
    '''
    Fonction permettant de calculer le paramètre d'excentricité et le demi petit axe d'une ellipse en fonction de du demi grand axe et de l'applatissement.

    Parameters
    ----------
        a : Float
            Demi grand axe de l'ellipse [m].
        f : Float
            Applatissement de l'ellipse [-].

    Returns
    -------
        e : float
            Excentricité de l'ellipse [-].
        b : float
            Demi petit axe de l'ellipse [m].

    '''
    b=a-a*f
    e=np.sqrt(a**2-b**2)/a

    return e,b

def get_ell_param(sys_ref = 'GRS80'):
    '''
    Fonction retournant les paramètres des ellispoides de référence GRS80 et Bessel 1841.

    Parameters
    ----------
        sys_ref : str, optional
            Ellispoide. The default is 'GRS80'.
                'GRS80' : Système de référence géodésique 1980
                'Bessel' : Bessel 1841

    Returns
    -------
        a : float
            Demi grand axe de l'ellipse [m].
        b : float
            Demi petit axe de l'ellipse [m].
        e : float
            Excentricité de l'ellipse [-].
        f : float
            Applatissement de l'ellipse [-].

    '''

    if sys_ref == 'Bessel':
        a_Bessel=6377397.155
        f_Bessel=1/299.1528128
        e_Bessel,b_Bessel = ell_param(a_Bessel,f_Bessel)

        return a_Bessel,b_Bessel,e_Bessel,f_Bessel

    if sys_ref == 'GRS80':
        a_GRS80 = 6378137.000
        f_GRS80=1/298.257222101
        e_GRS80,b_GRS80 = ell_param(a_GRS80,f_GRS80)

        return a_GRS80,b_GRS80,e_GRS80,f_GRS80


def ell2cart(a,e,lon_deg,lat_deg,h):
    '''
    Fonction permettant de convertir des longitude, latitude et hauteur ellispoidales en coordonnées cartésiennes géocentriques.

    Parameters
    ----------
        a : float
            Demi grand axe de l'ellipse [m].
        e : float
            Excentricité de l'ellipse [-].
        lon_deg : float
            Longitude [deg].
        lat_deg : TYPE
            Latitude [deg].
        h : TYPE
            Hauteur ellispoidale [m].

    Returns
    -------
        x : float
            X géocentrique [m].
        y : TYPE
            Y géocentrique [m].
        z : float
            Z géocentrique [m].

    '''
    lon_rad = lon_deg*np.pi/180.0
    lat_rad = lat_deg*np.pi/180.0

    RN = a/np.sqrt(1-e**2*np.sin(lat_rad)**2)
    x = (RN+h)*np.cos(lat_rad)*np.cos(lon_rad)
    y = (RN+h)*np.cos(lat_rad)*np.sin(lon_rad)
    z = (RN*(1-e**2)+h)*np.sin(lat_rad)

    return x,y,z
def cart2ell(a,e,x,y,z):
    '''
    Fonction permettant de convertir coordonnées cartésiennes géocentriques en des longitude, latitude et hauteur ellispoidales.

    Parameters
    ----------
        a : float
            Demi grand axe de l'ellipse [m].
        e : float
            Excentricité de l'ellipse [-].
        x : float
            X géocentrique [m].
        y : TYPE
            Y géocentrique [m].
        z : float
            Z géocentrique [m].

    Returns
    -------
        lon_deg : float
            Longitude [deg].
        lat_deg : TYPE
            Latitude [deg].
        h : TYPE
            Hauteur ellispoidale [m].

    '''
    lon_rad = np.arctan2(y,x)
#    if lon_rad<0:
#        lon_rad += 2*np.pi
    lon_deg = lon_rad*180.0/np.pi

    hi = 0
    him1=1
    RNi = 1

    while abs(hi-him1)>0.000001:

        him1=hi

        lat_radi = np.arctan2(z,np.sqrt(x**2+y**2)*(1-(RNi/(RNi+hi))*e**2))
        RNi = a/np.sqrt(1-e**2*np.sin(lat_radi)**2)
        hi = np.sqrt(x**2+y**2)/np.cos(lat_radi)-RNi


    lat_deg = lat_radi*180.0/np.pi
    h = hi

    return lon_deg,lat_deg,h

def ell2EN(proj,lon_deg,lat_deg,a,e):
    '''
    Fonction permettant de convertir des coordonnées ellipsoidales géocentriques en coordonnées planimétriques topocentriques projetées "Swiss MN95" ou "Swiss MN03".

    Parameters
    ----------
        proj : str
            Projection choisie.
                "swiss_MN95" : coordonnées dans le système de référence MN95
                "swiss_MN03" : coordonnées dans le système de référence MN03
        lon_deg : float
            Longitude [deg].
        lat_deg : TYPE
            Latitude [deg].
        a : float
            Demi grand axe de l'ellipse [m].
        e : float
            Excentricité de l'ellipse [-].

    Returns
    -------
        E : float
            Coordonnée Est projetée [m].
        N : float
            Coordonnée Nord projetée [m].

    '''

    if proj=='swiss_MN95' or proj=='swiss_MN03':

        lon = lon_deg*np.pi/180.0
        lat = lat_deg*np.pi/180.0

        a=6377397.155
        f=1/299.1528128
        e,b = ell_param(a,f)
        lon0= (7.0 + 26.0/60.0 + 22.50/3600.0)*np.pi/180.0
        alpha=1.0007291384304
        k=1.0030714396280
        lat_sph_0 = (46.0 + 54.0/60.0 + 27.83324846/3600.0)*np.pi/180.0
        R_sph = 6378815.90365

        if proj=='swiss_MN95':
            E0 = 2600000.000
            N0 = 1200000.000
        elif proj=='swiss_MN03':
            E0 = 600000.000
            N0 = 200000.000


        #ellipsoide->sphere normale
        lon_sph = alpha*(lon-lon0)
        lat_sph = 2*np.arctan(  k*np.tan(np.pi/4+lat/2.0)**alpha  *  ((1-e*np.sin(lat))/(1+e*np.sin(lat)))**(alpha*e/2.0)  )-np.pi/2.0

        #sphere normale->sphere oblique
        lon_sph_t = np.arctan(np.sin(lon_sph)/(np.sin(lat_sph_0)*np.tan(lat_sph) + np.cos(lat_sph_0)*np.cos(lon_sph)))
        lat_sph_t = np.arcsin(np.cos(lat_sph_0)*np.sin(lat_sph)-np.sin(lat_sph_0)*np.cos(lat_sph)*np.cos(lon_sph))

        #sphere oblique->plan
        E = E0 + R_sph*lon_sph_t
        N = N0 + R_sph*np.log(np.tan(np.pi/4.0+lat_sph_t/2.0))

    return E,N

def WGSell2cart(lon_deg,lat_deg,h):
    '''
    Fonction permettant de convertir des longitude, latitude et hauteur ellispoidales WGS84 en coordonnées cartésiennes géocentriques.

    Parameters
    ----------
        lon_deg : float
            Longitude [deg].
        lat_deg : TYPE
            Latitude [deg].
        h : TYPE
            Hauteur ellispoidale [m].

    Returns
    -------
        x : float
            X géocentrique WGS84 [m].
        y : TYPE
            Y géocentrique WGS84 [m].
        z : float
            Z géocentrique WGS84 [m].

    '''
    a_GRS80,b_GRS80,e_GRS80,f_GRS80 = get_ell_param('GRS80')
    return ell2cart(a_GRS80,e_GRS80,lon_deg,lat_deg,h)



def CHTRS952MN95(x_CHTRS95,y_CHTRS95,z_CHTRS95):
    '''
    Fonction permettant de convertir des coordonnées cartésiennes géocentriques CHTRS95 en coordonnées planimétriques topocentriques projetées "Swiss MN95" ou "Swiss MN03".

    Parameters
    ----------
        x_CHTRS95 : float
            X cartésien géocentrique CHTRS95 [m].
        y_CHTRS95 : float
            Y cartésien géocentrique CHTRS95 [m].
        z_CHTRS95 : float
            Z cartésien géocentrique CHTRS95 [m].

    Returns
    -------
        E : float
            Coordonnée Est projetée MN95 [m].
        N : float
            Coordonnée Nord projetée MN95 [m].
        h_CH1903p : float
            Hauteur ellipsoidale CH1903 [m].

    '''
    x_CH1903p = x_CHTRS95-674.374
    y_CH1903p = y_CHTRS95-15.056
    z_CH1903p = z_CHTRS95-405.346

    a_Bessel,b_Bessel,e_Bessel,f_Bessel = get_ell_param('Bessel')
    lon_CH1903p_deg,lat_CH1903p_deg,h_CH1903p = cart2ell(a_Bessel,e_Bessel,x_CH1903p,y_CH1903p,z_CH1903p)

    E_MN95,N_MN95 = ell2EN('swiss_MN95',lon_CH1903p_deg,lat_CH1903p_deg,a_Bessel,e_Bessel)

    return E_MN95,N_MN95,h_CH1903p

def read_agr2mat(agr_path):
    '''
    Fonction permettant de lire un fichier ".agr" de cote du géoide et de le convertir en liste.

    Parameters
    ----------
        agr_path : str
            Chemin complet du fichier agr.

    Returns
    -------
        geoid_RAN95 : list
            Liste de cote du géoide.

    '''
    geoid_RAN95 = []
    with open(agr_path,'r') as f:
        line = f.readline();line = f.readline();line = f.readline();line = f.readline();line = f.readline();line = f.readline()
        while line:
            line = f.readline()
            if line != '':
                geoid_RAN95.append(line.split())
    return geoid_RAN95
def hell2RAN95(E,N,h,geoid):
    '''
    Fonction permettant de convertir une hauteur ellipsoidale CH1903 en altitude orthométrique RAN95.

    Parameters
    ----------
        E : float
            Coordonnée Est projetée MN95 [m].
        N : float
            Coordonnée Nord projetée MN95 [m].
        h : float
            Hauteur ellipsoidale CH1903 [m].
        geoid : list
            Liste de cote du géoide.

    Returns
    -------
        H : float
            Altitude orthométrique RAN95 [m].

    '''
    #NOTATIONS pour interpolation bilinéaire:
    #      |
    #    __|   2 +             4 +
    #  1-y |
    #    __|           M +
    #      |
    #   y  |
    #    __|   1 +             3 +
    #      |_______________________
    #            |   x   |  1-x  |
    # Nm = (1-x)(1-y)N1+(1-x)yN2+x(1-y)N3+xyN4
    # x = (Em-E1)/(E3-E1)
    # y = (Nm-N1)/(N2-N1)
    x_start = 2480000#2479500
    y_start = 1060000#1059500
    nb_lig = int((N-y_start)/1000)
    nb_col = int((E-x_start)/1000)
    #print(nb_lig,nb_col)
    E1 = x_start+1000*nb_col
    N1 = y_start+1000*(nb_lig)
    #print(E1,N1)
    x = (E-E1)/1000; y = (N-N1)/1000
    N_RAN95_1 = float(geoid[-(nb_lig+1)][nb_col])
    N_RAN95_2 = float(geoid[-(nb_lig+2)][nb_col])
    N_RAN95_3 = float(geoid[-(nb_lig+1)][nb_col+1])
    N_RAN95_4 = float(geoid[-(nb_lig+2)][nb_col+1])
    N_RAN95 = (1-x)*(1-y)*N_RAN95_1+(1-x)*y*N_RAN95_2+x*(1-y)*N_RAN95_3+x*y*N_RAN95_4
    return h-N_RAN95


def coord_from_raw_data(lh,lv,di,facteur_echelle):
    '''
    Conversion de mesures brutes GeoCom en coordonnées locales.

    Parameters
    ----------
        lh : str
            Direction horizontale [rad].
        lv : str
            Angle zénithal [rad].
        di : str
            Distance inclinée [m].
        facteur_echelle : float
            Facteur d'échelle sur les distances [-].

    Returns
    -------
        X : list
            Vecteur tridimensionnel de coordonnées locales.
            x : str
                x local [m]
            y : str
                y local [m]
            z : str
                z local [m]
    '''
    lh = float(lh); lv = float(lv); di = float(di)
    dh = facteur_echelle * di * abs(math.sin(lv))
    x = 1000 + dh * math.sin(lh)
    y = 2000 + dh * math.cos(lh)
    z = 3000 + dh/math.tan(lv) #di * math.cos(lv)
    return [format(x,".4f"),format(y,".4f"),format(z,".4f")]
def readTimeFromGGA(trame):
    '''
    Lecture du temps GNSS de la trame GGA.

    Parameters
    ----------
        trame : str
            trame GGA.

    Returns
    -------
        t_GNSS : str
            Temps GNSS [s].

    '''
    time = trame.split(',')[1] #str
    SecondUTC = int(time[0:2])*3600 + int(time[2:4])*60 + int(time[4:6]) + int(time[7:9])/100
    return format(float(SecondUTC),".3f")
def isGGA(trame):
    '''
    Vérification du retour GNSS ou non.

    Parameters
    ----------
        trame : str
            Trame GNSS.

    Returns
    -------
        isGGA : bool
            GNSS en GGA ou non.
                True : Trame GGA
                False : Pas de trame GGA

    '''
    if trame.split(',')[0] == '$GNGGA':
        return True
    else:
        return False
def readPositionFromGGA(trame):
    '''
    Lecture de la position GNSS WGS84 ellipsoidale de la trame GGA.

    Parameters
    ----------
        trame : str
            Trame GNSS.

    Returns
    -------
        p : list
            Vecteur coordonnées ellipsoidales WGS84.
            phi : str
                Latitude ellipsoidale WGS84 [deg]
            lambda : str
                Longitude ellipsoidale WGS84 [deg]
            h : str
                Hauteur ellispoidale WGS84 [m]

    '''
    lat = trame.split(',')[2] #str
    lat = float(lat[0:2]) + float(lat[2:])/60 #degres décimal float
    lon = trame.split(',')[4] #str
    lon = float(lon[0:3]) + float(lon[3:])/60 #degres décimal float
    alt = float(trame.split(',')[9])+float(trame.split(',')[11]) #float on ellipsoide wgs84 (correction du faux geoide)
    return [format(lat,".8f"), format(lon,".8f"), format(alt,".3f")]
def isRTK(trame):
    '''
    Vérification du mode de mesure GNSS RTK ou non.

    Parameters
    ----------
        trame : str
            Trame GNSS.

    Returns
    -------
        isRTK : bool
            GNSS en RTK ou non.
                True : Position RTK
                False : Pas de position RTK

    '''
    code = trame.split(',')[6]
    if code == '4':#code positionnement RTK
        return True
    else:
        return False
def openXbee(port,baudrate):
    '''
    Ouverture du port série radio en communication avec le récepteur GNSS Xbee

    Parameters
    ----------
        port : str
            Port série du Raspberry Pi.
        baudrate : int
            Vitesse de transmission via le port série.

    Returns
    -------
        ser_GPS : bool or object
            Retourne l'objet port série si la connexion est réussie ou False si la connexion a échoué.

    '''
    try:
        ser_GPS = serial.Serial(
            port = port,#'/dev/ttyS0'
            baudrate =baudrate,#115200
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = 0.05
        )
        if not ser_GPS.isOpen():
            ser_GPS.open()
        if not '$' in ser_GPS.readline().decode('utf-8'):
            return False
        else:
            print(f'{bcolors.OKGREEN}[INFO] Xbee is open and connected: {bcolors.ENDC}',ser_GPS.isOpen())
        return ser_GPS
    except:
        print(f'{bcolors.WARNING}[Warning] Xbee is not connected or wrong port name or activate ttyS0 port on rpi.{bcolors.ENDC}')
        return False

def openTacheo(TS):
    '''
    Ouverture du port série RS232 en communication avec la station totale Leica

    Parameters
    ----------
        TS : object
            Objet GeoCom

    Returns
    -------
        Test : bool
            Station totale connectée au port série ou non
                True : Station connectée au port série
                False : Echec de la connexion de la station totale au port série

    See Also
    --------
        "GeocomLibrary.py" pour la documentation sur l'objet GeoCom"

    '''
    Open = TS.connectSerial()
    if Open :
        print(f'{bcolors.OKGREEN}[INFO] Tacheo is open: {bcolors.ENDC}',TS.checkOpen())
    else :
        print(f'{bcolors.WARNING}[Warning] Total station is not connected or wrong port name or change the communication canal on GSI by RS232 or give right to ttyUSB0.{bcolors.ENDC}')
        return False
    try:
        #initialize Tacheo
        print(f'{bcolors.OKGREEN}[INFO] Start initializing total station.{bcolors.ENDC}')
        bool_test = initializeTacheo(TS)
        if not bool_test:
            print(f'{bcolors.WARNING}[Warning] Total station is not initialized, Try again :{bcolors.ENDC}')
            return False
        print(f'{bcolors.OKGREEN}[INFO] Tacheo is open and initialized: {bcolors.ENDC}',TS.checkOpen())
        return True
    except:
        print(f'{bcolors.WARNING}[Warning] Total station is not connected or wrong port name or change the communication canal GSI by RS232 or give right to ttyUSB0.{bcolors.ENDC}')


def initializeTacheo(TS):
    '''
    Initialisation des mesures station totale

    Parameters
    ----------
        TS : object
            Objet GeoCom.

    Returns
    -------
        Test : bool
            Lancement du tracking prisme et des mesures effectués avec succès ou pas.
                True : Tracking et mesures OK
                False : Tracking et mesures pas OK

    See Also
    --------
        "GeocomLibrary.py" pour la documentation sur l'objet GeoCom"
    '''
    #choix type de prisme / Mini-prisme 360
    out = TS.BAP_SetPrismType(7)
    # EDM mode
    out = TS.TMC_SetEdmMode(7)
    # Cherche
    out = TS.AUT_FineAdjust(5.0,5.0)
    #ATR On
    out = TS.AUS_SetUserAtrState(1)
    #Lock On
    out = TS.AUS_SetUserLockState(1)
    # TRACK ON (lock le prisme et le suit)
    out = TS.AUT_LockIn()
    #Do measure
    out = TS.TMC_DoMeasure(1, 0)
    if out['RC'] == '0':
        return True
    else:
         return False

def init_txtFile(filename):
    '''
    Création et ouverture du fichier texte des observations brutes

    Parameters
    ----------
        filename : str
            Nom du fichier d'enregistrement des données brutes ".txt".

    Returns
    -------
        None
    '''
    with open(filename,'w') as txt:
        #txt.write('Station libre par drone - Projet HEIG-VD\n')
        #txt.write('MESURES BRUTES\n')
        #txt.write('===============================================\n')
        txt.write('$Tacheo; Xlocal (m); Ylocal (m); Zlocal (m); Hz (rad); V (rad); di (m); False T_GNSS (s)'+'\n')
        txt.write('OR'+'\n')
        txt.write('$GNSS; lat (degrees); lon (degrees); alt_ellips (m); True T_GNSS (s)\n')
        #txt.write('===============================================\n')

#Si il retourne False c'est que le prisme est perdu
def ask4observationTacheo(TS, ser_GPS):
    '''
    Récupération des mesures brutes de la station totale

    Parameters
    ----------
        TS : object
            Objet GeoCom.
        ser_GPS : object
            Objet port série.

    Returns
    -------
        Data : list or bool
            Retourne les données brutes ou False si erreur dans la récupération des données
                Hz : str
                    Direction horizontale [rad]
                V : str
                    Angle zénithal [rad]
    '''
    out = TS.AUS_GetUserLockState()
    if out['ONOFF'] == '1' :
        val = TS.TMC_GetSimpleMea(0, 0)
        if val['RC'] != '0' :
            out = TS.AUS_SetUserLockState(0)
            print(f'{bcolors.WARNING}[Warning] Bad return from the total station, prism lost.{bcolors.ENDC}')
            return False
        else :
            return [format(val['Hz']*np.pi/200.0,".6f"),format(val['V']*np.pi/200.0,".6f"),format(val['SlopeDistance'],".4f"),time.time()]
    else :
        out = TS.AUT_FineAdjust(0.5,0.5)
        out = TS.AUS_SetUserLockState(1)
        Lock = TS.AUT_LockIn()
        out = TS.TMC_DoMeasure(1, 0)
        if Lock['RC'] != '0' or out['RC'] != '0' :
            out = TS.AUS_SetUserLockState(0)
        ser_GPS.reset_input_buffer()
        print(f'{bcolors.WARNING}[Warning] Bad return from the total station, prism lost.{bcolors.ENDC}')
        return False


def ask4observationGPS(ser):
    '''
    Récupération des observations GNSS

    Parameters
    ----------
        ser : object
            Objet port série.

    Returns
    -------
        DATA : list or str or bool
            Si RTK : list
                POS : list
                    Liste position GNSS WGS84
                t_GNSS : str
                    Temps GNSS [s]
                t_COMPUTER : float
                    Temps ordinateur [s]
            Si GGA : str
                'GGA'
            Si Aucun : bool
                False

    See Also
    --------
        readPositionFromGGA() : Liste position GNSS WGS84
        readTimeFromGGA() : Temps GNSS [s]
        isGGA() : Check de la trame GGA
        isRTK() : Check de la position RTK

    '''
    gnss_bin_data = ser.readline()
    gnss_asc_data = gnss_bin_data.decode('utf-8')
    if isGGA(gnss_asc_data):
        if isRTK(gnss_asc_data):
            return [readPositionFromGGA(gnss_asc_data),readTimeFromGGA(gnss_asc_data),time.time()]
        else:
            return 'GGA'
    else:
        return False
def projection_for_f(lat,lon, h):
    '''
    Conversion des coordonnées GNSS WGS84 en coordonnées projetées topocentrique MN95 et altitude ellipsoidale

    Parameters
    ----------
        lat : float
            Latitude ellipsoidale WGS84 [deg].
        lon : float
            Longitude ellispoidale WGS84 [deg].
        h : float
            Hauteur ellispoidale WGS84 [deg].

    Returns
    -------
        E_MN95 : str
            Coordonnée Est projetée MN95 [m].
        N_MN95 : str
            Coordonnée Nord projetée MN95 [m].
        h_CH1903p : str
            Hauteur ellipsoidale CH1903 [m].

    See Also
    --------
        WGSell2cart()
        CHTRS952MN95()

    '''
    X_WGS84,Y_WGS84,Z_WGS84 = WGSell2cart(lon,lat,h)
    E_MN95,N_MN95,h_CH1903p = CHTRS952MN95(X_WGS84,Y_WGS84,Z_WGS84)
    return format(E_MN95,".3f"),format(N_MN95,".3f"),format(h_CH1903p,".3f")
def projection(lat,lon,h,geoid):
    '''
    Conversion des coordonnées GNSS WGS84 en coordonnées projetées topocentrique MN95 et altitude orthométrique RAN95

    Parameters
    ----------
        lat : float
            Latitude ellipsoidale WGS84 [deg].
        lon : float
            Longitude ellispoidale WGS84 [deg].
        h : float
            Hauteur ellispoidale WGS84 [deg].
        geoid : list
            Liste de cote du géoide.

    Returns
    -------
        X_MN95 : list
            Vecteur tridimensionnel de coordonnées topocentriques MN95 et altitude RAN95.
            E_MN95 : str
                Coordonnée Est projetée MN95 [m].
            N_MN95 : str
                Coordonnée Nord projetée MN95 [m].
            H_RAN95 : str
                Altitude orthométrique RAN95 [m].

    See Also
    --------
        WGSell2cart()
        CHTRS952MN95()
        hell2RAN95()
    '''
    X_WGS84,Y_WGS84,Z_WGS84 = WGSell2cart(lon,lat,h)
    E_MN95,N_MN95,h_CH1903p = CHTRS952MN95(X_WGS84,Y_WGS84,Z_WGS84)
    H_RAN95 = hell2RAN95(E_MN95,N_MN95,h_CH1903p,geoid)
    return [format(E_MN95,".3f"),format(N_MN95,".3f"),format(H_RAN95,".3f")]
def writeDataGps(filename,data):
    '''
    Ecriture dans le fichier de mesures brutes les données GNSS\n
    Ecriture sous forme "append" car le fichier est déjà créé

    Parameters
    ----------
        filename : str
            Nom du fichier de mesures brutes.
        data : array
            Coordonnées brutes GNSS WGS84.

    Returns
    -------
        None.

    '''
    with open(filename,'a') as f:
        f.write('$GNSS;'+str(data[0][0])+';'+str(data[0][1])+';'+str(data[0][2])+';'+str(data[1])+'\n')
def writeDataTacheo(filename,data,time_gps,facteur_echelle):
    '''
    Ecriture dans le fichier de mesures brutes les données de la station totale\n
    Ecriture sous forme "append" car le fichier est déjà créé

    Parameters
    ----------
        filename : str
            Nom du fichier de mesures brutes.
        data : list
            Mesures brutes issues de la station totale
        time_gps : float
            Temps GNSS [s]
        facteur_echelle : float
            Facteur d'échelle calculé sur les distances [m]

    Returns
    -------
        None.

    '''
    coordLocale = coord_from_raw_data(data[0],data[1],data[2],facteur_echelle)
    with open(filename,'a') as f:
        f.write('$Tacheo;'+str(coordLocale[0])+';'+str(coordLocale[1])+';'+str(coordLocale[2])+';'+str(data[0])+';'+str(data[1])+';'+str(data[2])+';'+str(time_gps)+'\n')

def finish_txt_file(filename_brut,filename_fini,geoid):
    f_res = open(filename_fini,'w')
    n = 0
    with open(filename_brut,'r') as f:
        for line in f:
            n += 1
    f_res.write('$Tacheo; Xlocal (m); Ylocal (m); Zlocal (m); Hz (rad); V (rad); di (m); False T_GNSS (s)'+'\n')
    f_res.write('OR'+'\n')
    f_res.write('$GNSS; lat (degrees); lon (degrees); alt_ellips (m); True T_GNSS (s); Xmn95; Ymn95; H RAN95 \n')
    with open(filename_brut,'r') as f:
        line = f.readline(); line = f.readline(); line = f.readline()
        while line:
            line = f.readline()
            if '$GNSS' in line:
                data = line.split(';')
                coord = projection(float(data[1]),float(data[2]),float(data[3]),geoid)
                f_res.write(line[:-1]+';'+coord[0]+';'+coord[1]+';'+coord[2]+'\n')
            if 'Tacheo' in line:
                f_res.write(line)
def gisement2(x1,y1,x2,y2):
    '''
    Calcul de gisement en fonction de deux points en coordonnées projetées 2D


    Parameters
    ----------
        x1 : float
            Coordonnées Est du point de base [m].
        y1 : float
            Coordonnées Nord du point de base [m].
        x2 : float
            Coordonnées Est du point visé [m].
        y2 : float
            Coordonnées Nord du point visé [m].

    Returns
    -------
        gis : float
            Gisement [rad]

    '''
    delta_x=x2-x1
    delta_y=y2-y1
    gis = math.atan2(delta_x,delta_y)
    if gis<0:
        gis+=(2*math.pi)
    return gis

def distance(point1,point2):
    '''
    Calcul de la distance entre deux points dans un système de coordonnées projeté en 2D.

    Parameters
    ----------
        point1 : list
            Vecteur 2D de coordonnées projetées du point 1.
                E : float
                    Coordonnée Est [m]
                N : float
                    Coordonnée Nord [m]
        point2 : list
            Vecteur 2D de coordonnées projetées du point 2.
                E : float
                    Coordonnée Est [m]
                N : float
                    Coordonnée Nord [m]

    Returns
    -------
        d : float
            Distance [m].

    '''
    d=math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
    return d

def make_P2(res_filename):
    '''
    Création des matrices des oservations brutes.

    Parameters
    ----------
        res_filename : str
            Nom du fichier résultat.

    Returns
    -------
        P_global : array
            Matrice des observations brutes GNSS.
        P_local : array
            Matrice des observations brutes station totale.
    '''
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
                P_local = np.append(P_local,[float(data[7]),float(data[1]),float(data[2]),float(data[3]),float(data[4]),float(data[5]),float(data[6])])
    return P_global, P_local

def filtre_detect_pause(P_global):
    '''
    En mode statique, détection des zones ou le drone était statique durant le levé pour les points GNSS.

    Parameters
    ----------
        P_global : array
            Matrice des observations brutes GNSS.

    Returns
    -------
        P_lglobal : array
            Matrice des observations brutes GNSS filtrée.

    '''
    point_avant = P_global[0]
    idx_to_remove =[]
    for i in range(1,len(P_global)):
        point = P_global[i]
        if distance(point[1:3], point_avant[1:3])>0.05:#seuil à 50cm/sec pour que ce soit considéré comme une pause
            idx_to_remove.append(i)
        point_avant = P_global[i]
    P_lglobal = np.delete(P_global,idx_to_remove,axis=0)
    return P_lglobal
def filtre_data_tacheo(P_global,P_local):
    '''
    En mode statique, détection des zones ou le drone était statique durant le levé pour les points station totale.

    Parameters
    ----------
        P_global : array
            Matrice des observations brutes GNSS filtrée.
        P_local : array
            Matrice des observations brutes station totale.

    Returns
    -------
        new_P_local : array
            Matrice des observations brutes station totale filtrée.

    '''
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

def thread_function_acquisition(name, obs_filename):
    '''
    Thread d'acquisition\n
    Fonction regroupant tout le processus d'acquisition GNSS et station totale\n
    Lancée au moment où le bouton acquisition est pressé\n
    1e étape :\n
    Connexion au ports série du GNSS et de la station totale, initialisation de l'acquistion\n
    Test de la connexion aux deux instruments (boucle)\n
    2e étape :\n
    Acquisition des positions et enregistrement des données brutes\n
    Voir les commentaires dans le code source pour plus de détail sur le fonctionnement de la fonction\n

    Parameters
    ----------
        name : TYPE
            DESCRIPTION.
        obs_filename : str
            DESCRIPTION.

    See Also
    --------
        ask4observationGPS() : fct
            Fonction
    '''
    with open('temp_info.json','w') as file_info:
        info_acquis_dict =  { "GPS_connect" : "NO", "Tacheo_connect" : "NO", "RTK" : "NO", "Prisme_lock_and_compensateur" : "NO", "Enregistrement_des_points" : "NO", "Nombre_GPS" : '0', "Echelle" : "0"}
        file_info.seek(0)  # rewind
        json.dump(info_acquis_dict,file_info)
        file_info.truncate()
        logging.info("Thread %s: starting", name)
        init_txtFile(obs_filename)
        portXbee = '/dev/ttyS0'
        portTacheo = '/dev/ttyUSB0'
        facteur_echelle = 0
        start_time = 0
        end_time = 0
        time_GPS = 0
        block = True
        nb_points_GPS = 0
        count_without_GPS = 0#pour éviter de tout bloquer quand il n'y a pas de retour GPS ie quand le timeout est plus petit que la fréquence du GPS
        ser_GPS = False; bool_tacheo = False
        TS = com.GeoCom(portTacheo,115200,1.0,1.0)
        print(f'{bcolors.WARNING}[Warning] RTK position is not yet available.{bcolors.ENDC}')
        while not ser_GPS or not bool_tacheo:
            if not ser_GPS:
                ser_GPS = openXbee(portXbee,115200)
            if ser_GPS:
                info_acquis_dict['GPS_connect'] = 'OK'
                file_info.seek(0)
                json.dump(info_acquis_dict,file_info)
                file_info.truncate()
            if not bool_tacheo :
                bool_tacheo = openTacheo(TS)
            if bool_tacheo:
                info_acquis_dict['Tacheo_connect'] = 'OK'
                file_info.seek(0)
                json.dump(info_acquis_dict,file_info)
                file_info.truncate()
        ser_GPS.reset_input_buffer()
        print(f'{bcolors.OKGREEN}[INFO] Total station and Xbee GPS are connected and open.{bcolors.ENDC}')
        while 1 :
            while facteur_echelle == 0 :#calcul de la correction de distance (au niveau 0 et facteur d'échelle)
                data_GPS = ask4observationGPS(ser_GPS)
                if data_GPS and data_GPS != 'GGA':
                    info_acquis_dict['RTK'] = 'OK'
                    E, N, h = projection_for_f(float(data_GPS[0][0]),float(data_GPS[0][1]),float(data_GPS[0][2]))
                    facteur_echelle =((float(N)-1200000)**2/(2*(6378800**2)))-(float(h)/6378800)+1
                    print(f'{bcolors.OKGREEN}[INFO] Facteur d\'échelle calculé : {bcolors.ENDC}',facteur_echelle)
                    info_acquis_dict['Echelle'] = format(facteur_echelle,'.6f')
                else:
                    print(f'{bcolors.OKGREEN}[INFO] Calcul du facteur d\'échelle, si trop long, problème de RTK.{bcolors.ENDC}')
                    info_acquis_dict['RTK'] = 'NO'
                file_info.seek(0)
                json.dump(info_acquis_dict,file_info)
                file_info.truncate()
            data_GPS = ask4observationGPS(ser_GPS)
            data_tacheo = ask4observationTacheo(TS, ser_GPS)
            if data_GPS and data_GPS != 'GGA':
                count_without_GPS = 0
                print(f'{bcolors.OKGREEN}[INFO] GPS RTK OK.{bcolors.ENDC}')
                info_acquis_dict['RTK'] = 'OK'
                time_GPS = float(data_GPS[1])
                start_time = float(data_GPS[2])
                if not block:
                    writeDataGps(obs_filename,data_GPS)
                    nb_points_GPS +=1
                block = False
            elif data_GPS == 'GGA': #veut dire que le GPS n'est pas encore en RTK, on évite d'écrire les données tacheo
                #ecrire count not gps ok et continue pour pas ecrire les données tacheo si c'est du GGA
                block = True
                print(f'{bcolors.OKGREEN}[INFO] GPS non RTK.{bcolors.ENDC}')
                info_acquis_dict['RTK'] = 'FLOAT'#en orange sur l'UI
            elif not data_GPS:
                count_without_GPS+=1
                if count_without_GPS>20:
                    print(f'{bcolors.WARNING}[Warning] No GPS data, verify GPS.{bcolors.ENDC}')
                    info_acquis_dict['GPS_connect'] = 'NO'
                    info_acquis_dict['RTK'] = 'NO'
                    block = True
            if data_tacheo != False :
                end_time = float(data_tacheo[3])
                time = time_GPS + end_time - start_time #time gps + delay
                if not block:
                    writeDataTacheo(obs_filename,data_tacheo,format(time,".3f"),facteur_echelle)#la il faudrait pas prendre le temps de data_tacheo
                    print(f'{bcolors.OKGREEN}[INFO] Prisme locké, enregistrement des points actif.{bcolors.ENDC}')
                    info_acquis_dict["Enregistrement_des_points"] = 'OK'
                info_acquis_dict["Prisme_lock_and_compensateur"] = 'OK'
                block = False
            else:
                block = True
                print(f'{bcolors.WARNING}[Warning] Prisme non locké ou compensateur hors tolérance, enregistrement des points inactif.{bcolors.ENDC}')
                info_acquis_dict["Prisme_lock_and_compensateur"] = 'NO'
                info_acquis_dict["Enregistrement_des_points"] = 'NO'
            file_info.seek(0)  # rewind
            info_acquis_dict['Nombre_GPS'] = str(nb_points_GPS)
            json.dump(info_acquis_dict,file_info)
            file_info.truncate()
    logging.info("Thread %s: finishing", name)


def start_acquisition():
    '''
    Initialisation du thread de l'acquisition des observations.
    Création du fichier txt des observations.

    Parameters
    ----------
        None.

    Returns
    -------
        None.

    '''
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")

    logging.info("Main    : before creating thread")
    global obs_filename
    obs_filename = 'observations_brutes.txt'
    print(f'{bcolors.HEADER}[INFO] press calcul at the end of acquisition to start the calcul.{bcolors.ENDC}')
    global x
    x = multiprocessing.Process(target=thread_function_acquisition, args=(1,obs_filename))
    logging.info("Main    : before running thread")
    x.start()
    logging.info("Main    : wait for the thread to finish")
    logging.info("Main    : all done")

def calcul_z_from_corres(corres, P_local, P_global, ecart_antenne_prisme):#np array
    '''
    Calcul de l'altitude de la station selon la méthode des moyennes avec.
    Les correspondances des points sont déterminées dans l'ICP.

    Parameters
    ----------
        corres : list
            Correspondance entre les points globaux et locaux.
        P_local : numpy array
            Observations brutes de la station totale.
        P_global : numpy array
            Observations brutes GNSS transformée en MN95/RAN95.
        ecart_antenne_prisme : float
            Ecart vertical entre l'antenne GNSS et le prisme.

    Returns
    -------
        z : float
            Coordonnée H_RAN95 de la station [m].

    '''

    sum_local_z = 0
    sum_global_z = 0
    for corr in corres:
        id_global = corr[1]
        id_local = corr[0]
        sum_local_z += P_local[id_local][3]
        sum_global_z += P_global[id_global][3]
    moyenne_z_tache = sum_local_z/len(corres)
    moyenne_z_GPS = sum_global_z/len(corres)
    trans_z = moyenne_z_tache-moyenne_z_GPS+ecart_antenne_prisme #+ecart_antenne_prisme
    z = 3000 - trans_z
    return z
def start_calcul(ecart_antenne_prisme, value_radio):
    '''
    Calcul des paramètres de la station libre et des indicateurs statistiques liés au calcul ICP

    Parameters
    ----------
        ecart_antenne_prisme : float
            Ecart vertical entre l'antenne GNSS et le prisme.
        value_radio : str
            Etat de la radio connectee à l'antenne GNSS.

    Returns
    -------
        x_res : str
            Coordonnée E_MN95 de la station [m].
        y_res : str
            Coordonnée N_MN95 de la station [m].
        z_res : str
            Coordonnée H_RAN95 de la station [m].
        g0 : str
            Inconnue d'orientation de la station [gon].
        fitness : str
            Nombre de correspondences trouvées divisé par le nombre de points dans le set de points cible [%].
        inlier_rmse : str
            Précision calculée sur les points qui ont été appairés [mm].
        len_correspondence : str
            Nombre de couples de points correctement appairés [-].
        P_global : numpy array
            liste de la source transformée :
                X : Coordonnée Est transformée [m].
                Y : Coordonnée Nord transformée [m].
                Z : 0, l'altitude n'est pas transformée sur cette liste.
        source_trans : str
            id_source : int
                id du point source correspondant au point cible avec l'id_target [-].
            id_target : int
                id du point cible correspondant au point source avec l'id_source [-].

    '''
    print(f'{bcolors.HEADER}[INFO] Création du fichier résultat.{bcolors.ENDC}')
    stop_threads = True
    x.kill()
    time_temp = time.time()
    res_filename = 'file_resultat_'+str(time_temp)+'.txt'#ToDo ecrire une barre d'avancement, ou un pourcentage
    #lire le fichier txt et faire la projection directement ici, sinon ca prend trop de temps dans le code
    geoid = read_agr2mat('/home/antoine/Desktop/drone_RTK/code/dronertk/versionHTML/app_dynamique_statique/geoid/chgeo2004_LV95.agr')
    finish_txt_file(obs_filename,res_filename,geoid)
    print(f'{bcolors.HEADER}[INFO] Fichier résultat terminé.{bcolors.ENDC}')
    print(f'{bcolors.HEADER}[INFO] Début du calcul de la station libre par Ransac et ICP.{bcolors.ENDC}')
    P_global, P_local = make_P2(res_filename)
    P_global= P_global.reshape(int(len(P_global)/4),4)
    P_local = P_local.reshape(int(len(P_local)/7),7)
    threshold = 0.05

    if value_radio == 'STAT':
        P_global = filtre_detect_pause(P_global)
        P_local = filtre_data_tacheo(P_global,P_local)
        threshold = 0.02

    #import pdb; pdb.set_trace()
    x_res, y_res, z_res, g0, fitness, inlier_rmse, len_correspondence, source_trans, correspondence = Ransac.ICP(P_global, P_local, threshold) #[m], [m], [m], [gon], [%], [mm], nombre de données tacheo utilisées, pts tacheo trasnformée
    z_res = format(float(z_res)-ecart_antenne_prisme,'.3f')

    #update des points utilisés
    f_info = open('temp_res.json','w')
    info_res_dict = { "Nombre_points_GNSS" : str(len(P_global)), "Nombre_points_Tacheo" : str(len(P_local))}
    f_info.seek(0)  # rewind
    json.dump(info_res_dict,f_info)
    f_info.truncate()

    print(f'{bcolors.HEADER}[INFO] Calcul terminé.{bcolors.ENDC}')
    return x_res, y_res, z_res, g0, fitness, inlier_rmse, len_correspondence, P_global, source_trans

def SetNewStation(E,N,H,g0_gons):
    '''
    Mise en station de la station totale et orientation du zéro du cercle horizontal

    Parameters
    ----------
        E : str
            Coordonnée Est projetée MN95 [m].
        N : str
            Coordonnée Nord projetée MN95 [m].
        H : str
            Altitude orthométrique RAN95 [m]
        g0_gons : str
            Inconnue d'orientation [gon].

    Returns
    -------
        Test : bool
            booléen du test si la mise en station est effectuée avec succès ou pas

    '''
    E = float(E); N = float(N); H = float(H); g0_gons = float(g0_gons)
    portTacheo = '/dev/ttyUSB0'
    TS = com.GeoCom(portTacheo, 115200, 1.0,1.0)
    Open = TS.connectSerial()
    out = TS.AUS_SetUserLockState(0)
    while out['RC'] != '0':
        out = TS.AUS_SetUserLockState(0)

    #Rotation de l'instrument vers le 0 du cercle
    out = TS.AUT_MakePositioning(0.0000, 100.0,1,0)
    while out['RC'] != '0':
        out = TS.AUT_MakePositioning(0.0000, 100.0,0,0)

    g0rad = g0_gons
    out = TS.TMC_SetStation(E,N,H,0.000)
    out2 = TS.TMC_SetOrientation(g0rad)

    #Rotation de l'instrument vers le Nord
    out = TS.AUT_MakePositioning(0.0000, 100.0,1,0)
    while out['RC'] != '0':
        out = TS.AUT_MakePositioning(0.0000, 100.0,0,0)

    TS.disconnectSerial()
    if out['RC'] == '0' and out2['RC'] == '0':
        return True
    else :
        return False
