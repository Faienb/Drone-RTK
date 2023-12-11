import serial
import numpy as np
import time
import datetime as dt
import julian
from colorama import Fore

'''
Librairie de fonctions pour la communication en GeoCom avec instruments Leica Geosystems de type TS, TM, MS, DNA, LS.

Librairie orientee objet permettant de créer un objet port série de communication avec un instrument.

Permet l'enregistrement dans un fichier texte de toutes les comandes envoyees a l'instrument ainsi que le retour de l'instrument avec message d'erreur
'''


#Objet Geocom
class GeoCom :
    '''
    Initialisation de l'objet et du dictionnaire des erreurs instrument'
    '''
    #Initialisation des variables de l'objet, creation du fichier log et creation du dictionnaire des erreurs
    def __init__(self, serName, baudRate, stopBits, timeOut):#rajouter un "filename" si décommenté tout ce qui correspond au log
    # ---------------------------------------------
    # -- Initialisation des variables de l'objet --
    # ---------------------------------------------

        self.serName = serName
        self.baudRate = baudRate
        self.stopBits = stopBits
        self.timeOut = timeOut
        self.msg_ascii = False
        self.timeStop = 5.0

        # #Ouverture du fichier log
        # date = dt.datetime.now()
        # jour = date.strftime('%d')
        # mois = date.strftime('%m')
        # annee = date.strftime('%Y')
        # heure = date.strftime('%X')
        # dateNom = annee[-2:]+mois+jour
        # dateComplete = jour + '.' + mois + '.' + annee

        # filename = dateNom + '_' + logFileName + '.log'
        # self.log = open(filename, 'w', 1)
        # self.log.write('## Fichier ecrit le {} a {}\n'.format(dateComplete,heure))

        #Creation du dictionnaires des erreurs
        self.DictError = {}
        self.DictError.update({'0':'Function successfully completed.'})
        self.DictError.update({'2':'Invalid parameter detected. Result unspecified.'})
        self.DictError.update({'3':'Invalid result.'})
        self.DictError.update({'4':'Fatal error.'})
        self.DictError.update({'5':'Not implemented yet.'})
        self.DictError.update({'8':'Function execution has been aborted.'})
        self.DictError.update({'12':'Subsystem is down.'})
        self.DictError.update({'27':'GeoCOM Robotic license key not available.'})
        self.DictError.update({'517':'No targets detected.'})
        self.DictError.update({'524':'Not the spot of the own target illuminator.'})
        self.DictError.update({'1283':'Warning: measurement without full correction.'})
        self.DictError.update({'1284':'Info: accuracy can not be guarantee.'})
        self.DictError.update({'1285':'Warning: only angle measurement valid.'})
        self.DictError.update({'1288':'Warning: only angle measurement valid but without full correction.'})
        self.DictError.update({'1289':'Info: only angle measurement valid but accuracy can not be guarantee.'})
        self.DictError.update({'1290':'Error: no angle measurement.'})
        self.DictError.update({'1291':'Error: wrong setting of PPM or MM on EDM.'})
        self.DictError.update({'1292':'Error: distance measurement not done (no aim, etc.).'})
        self.DictError.update({'1293':'Error: system is busy (no measurement done).'})
        self.DictError.update({'3077':'Request timed out.'})
        self.DictError.update({'3081':'Unknown RPC, procedure ID invalid.'})
        self.DictError.update({'8704':'Position not reached.'})
        self.DictError.update({'8707':'Motorisation error.'})
        self.DictError.update({'8708':'Position not exactly reached'})
        self.DictError.update({'8709':'Deviation measurement error.'})
        self.DictError.update({'8710':'No target detected.'})
        self.DictError.update({'8711':'Multiple target detected.'})
        self.DictError.update({'8712':'Bad environment conditions.'})
        self.DictError.update({'8713':'Error in target acquisition'})
        self.DictError.update({'8714':'Target acquisition not enabled.'})
        self.DictError.update({'8716':'Target position not exactly reached.'})
        self.DictError.update({'9999':'Data received incomplete.'})

    # ------------------------------------------------------------------------------------
    # -- Fonctions de base pour la communication GeoCom et l'enregistrement du temps PC --
    # ------------------------------------------------------------------------------------

    def getJulianDate(self):
        '''
        Date julienne exprimee en jours

        Returns
        -------
            JD : float
                Date julienne exprimee en jours

        '''
        date1 = dt.datetime.now()
        JD = julian.to_jd(date1, fmt='jd')
        return JD

    def connectSerial(self):
        '''
        Connection au port serie

        Returns
        -------
            bool : bool
                booleen connexion effectuee avec succes ou pas

        '''

        self.ser = serial.Serial()
        self.ser.port = self.serName
        self.ser.baudrate = self.baudRate
        self.ser.timeout = self.timeOut
        self.ser.stopbits = self.stopBits
        #Ouverture du port serie
        try :
            self.ser.open()
            # self.log.write('Port {:s} ouvert\n'.format(self.serName))
            print(Fore.GREEN + 'Port {:s} ouvert'.format(self.serName) + Fore.RESET)
            return True
        except serial.SerialException :
            # self.log.write('Impossible d\'ouvrir le port {:s}\n'.format(self.serName))
            print(Fore.RED + 'Impossible d\'ouvrir le port {:s}'.format(self.serName) + Fore.RESET)
            return False

    def checkOpen(self):
        '''
        Controle de la bonne ouverture du port serie

        Returns
        -------
            TYPE : bool
                True : le port serie est ouvert\n
                False : le port serie n'est pas ouvert

        '''
        return self.ser.isOpen()

    def disconnectSerial(self):
        '''
        Fermeture du port serie
        '''

        self.ser.close()

        #Ecriture dans le fichier log
        # self.log.write('Port {:s} ferme\n'.format(self.serName))
        print(Fore.GREEN + 'Port {:s} ferme\n'.format(self.serName) + Fore.RESET)

    def SendToInstru(self):
        '''
        Fonction generale d'envoi de commande a l'instrument.
        Cette fonction ne peut pas etre appelee par un code externe.
        Ce sont les fonctions internes a l'objet qui l'appellent

        Returns
        -------
            RC : str
                Message d'erreur en reponse
        '''

        #Conversion du message ASCII  en bytes
        msg_bytes = self.msg_ascii.encode()

        #Ecriture du message sur le port série
        self.ser.write(msg_bytes)

        #Lecture du port série
        cont = True
        t0 = time.time()
        while cont :
            dt = time.time() - t0

            msg_recv_bytes = self.ser.readline()
            try :
                msg_recv_ascii = msg_recv_bytes.decode('utf-8').strip()
            except UnicodeDecodeError :
                msg_recv_ascii = ''

            if msg_recv_ascii != '' or dt >= self.timeStop :
                cont = False

        if msg_recv_ascii != '' :
            #RC
            RC = msg_recv_ascii.split(':')[1]
        else :
            RC = '4'
            print(Fore.RED + 'No response from instrument on port : {}'.format(self.serName) + Fore.RESET)

        return  RC
    # ------------------------------------------------------------------------------------------------
    # -- AUS The subsystem ‘Alt User’ mainly contains functions behind the “SHIFT” + ”USER” button. --
    # ------------------------------------------------------------------------------------------------

    def AUS_SetUserAtrState(self,OnOff):
        '''
        Active, respectivement désactive l'ATR

        Parameters
        ----------
            OnOff : int
                Active ou non l'ATR
                    0 = OFF\n
                    1 = ON

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        self.msg_ascii = '%R1Q,18005:{:d}\r\n'.format(OnOff)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUS_SetUserAtrState:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : AUS_SetUserAtrState:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUS_SetUserAtrState:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def AUS_SetUserLockState(self,OnOff):
        '''
        Active ou desactive le lOCK

        Parameters
        ----------
            OnOff : int
                Active ou non LOCK
                    0 = OFF\n
                    1 = ON

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        See Also
        --------
            AUT_LockIn() : Active le tracking
        '''

        self.msg_ascii = '%R1Q,18007:{:d}\r\n'.format(OnOff)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUS_SetUserLockState:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0':
            print(Fore.GREEN + 'Port {} : AUS_SetUserLockState:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUS_SetUserLockState:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out


    def AUS_GetUserLockState(self):
        '''
        Retourne le statut actuel du mode LOCK

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'ONOFF' : str
                    Statut du mode LOCK 0=OFF, 1=ON
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        self.msg_ascii = '%R1Q,18008:\r\n'
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'ONOFF':RC[1]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUS_GetUserLockState:{}, Lock {}\n'.format(out['JulianDate'],self.serName,RC[0], RC[1]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0':
            print(Fore.GREEN + 'Port {} : AUS_GetUserLockState:{}, Lock {}'.format(self.serName,RC[0],RC[1]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUS_GetUserLockState:{}, Lock {}'.format(self.serName,RC[0],RC[1]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------
    # -- AUT Automatisation; a module which provides functions like the control of the Automatic Target Recognition, Change Face function or Positioning functions. --
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------

    def AUT_ChangeFace(self,AUT_POSMODE, ATRMode):
        '''
        Cette fonction permet de changer la position de l'instrument (POS I ou II)

        Parameters
        ----------
            AUT_POSMODE : int
                0 : positionnement rapide (pas de mesure du compensateur)\n
                1 : positionement exact (demande plus de temps)
            ATRMode : int
                0 : positionnement en Hz et V\n
                1 : positionnement et recherche du prisme proche du Hz et V  (possible seulement si l'ATR est actif)

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        '''

        #Changement du temps d'attente
        self.timeStop = 20.0
        self.msg_ascii = '%R1Q,9028:{:d},{:d},0\r\n'.format(AUT_POSMODE, ATRMode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})
        self.timeStop = 5.0

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUT_ChangeFace:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : AUT_ChangeFace:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUT_ChangeFace:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def AUT_FineAdjust(self,dSrchHz,dSrchV):
        '''
        Cette fonction permet la recherche de prisme dans un intervalle angulaire donne en entree.

        Parameters
        ----------
            dSrchHz : float
                Pas angulaire sur le cercle horizontal [gon]
            dSrchV : float
                Pas angulaire sur le cercle vertical [gon]

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        '''
        #Changement du temps d'attente
        self.timeStop = 20.0
        dSrchHz = dSrchHz*np.pi/200.0
        dSrchV = dSrchV*np.pi/200.0
        self.msg_ascii = '%R1Q,9037:{:0.6f},{:0.6f},0\r\n'.format(dSrchHz,dSrchV)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})
        self.timeStop = 5.0

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUT_FineAdjust:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : AUT_FineAdjust:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUT_FineAdjust:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def AUT_LockIn(self) :
        '''
        Si le mode LOCK est active (AUT_SetUserLockState()), cette fonction active le suivi de prisme.

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        See Also
        --------
            AUT_SetUserLockState() : Le mode LOCK doit etre active avec cette fonction afin de pouvoir activer le suivi de prisme.
        '''
        self.msg_ascii = '%R1Q,9013:\r\n'
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUT_LockIn:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0':
            print(Fore.GREEN + 'Port {} : AUT_LockIn:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUT_LockIn:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def AUT_MakePositioning(self,Hz,V,AUT_POSMODE,ATRMode):
        '''
        Cette fonction positionne la lunette de l'instrument a des valeurs Hz et V donnees en entree.

        Parameters
        ----------
            Hz : float
                Direction horizontale [gon]
            V : float
                Angle zenithal [gon]
            AUT_POSMODE : int
                0 : positionnement rapide (pas de mesure du compensateur)\n
                1 : positionement exact (demande plus de temps)
            ATRMode : int
                0 : positionnement en Hz et V\n
                1 : positionnement et recherche du prisme proche du Hz et V (possible seulement si l'ATR est actif)

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        '''
        Hz = Hz*np.pi/200.0
        V = V*np.pi/200.0
        self.msg_ascii = '%R1Q,9027:{:f},{:f},{:d},{:d}\r\n'.format(Hz,V,AUT_POSMODE,ATRMode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : AUT_MakePositioning:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : AUT_MakePositioning:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : AUT_MakePositioning:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    # ---------------------------------------------------------------------------------------------
    # -- BAP Basic Applications; some functions, which can easily be used to get measuring data. --
    # ---------------------------------------------------------------------------------------------

    def BAP_SetPrismType(self, ePrismType):
        '''
        Definit le type de prisme pour une mesure IR sur prisme

        Parameters
        ----------
            ePrismType : int
                Type de prisme
                    0 : Leica Circular Prism\n
                    1 : Leica Mini Prism\n
                    2 : Leica Reflector Tape\n
                    3 : Leica 360° Prism\n
                    7 : Leica Mini 360° Prism\n
                    8 : Leica Mini Zero Prism\n
                    9 : User Defined Prism\n
                    10 : Leica HDS Target

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        self.msg_ascii = '%R1Q,17008:{:d}\r\n'.format(ePrismType)
        RC = '4'
        while RC == '4':
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : BAP_SetPrismType:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : BAP_SetPrismType:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : BAP_SetPrismType:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    # --------------------------------------------------------------------------------------------------------------------------------
    # -- CSV Central Services; this module provides functions to get or set central/basic information about the TPS1200 instrument. --
    # --------------------------------------------------------------------------------------------------------------------------------

    def CSV_GetInstrumentName(self):
        '''
        Retourne le nom de l'instrument, par exemple : TCRP1201 R300

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'Name' : str
                    Nom de l'instrument
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        self.msg_ascii = '%R1Q,5004:\r\n'
        RC = '4'
        while RC == '4':
            RC = self.SendToInstru()

        Name = RC.split(',')[1]
        RC = RC.split(',')[0]
        out = {}
        out.update({'RC':RC})
        out.update({'Name':Name})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : CSV_GetInstrumentName:{},{}\n'.format(out['JulianDate'],self.serName,RC,Name))
        # self.log.write('{}\n'.format(self.DictError[RC]))
        if RC == '0' :
            print(Fore.GREEN + 'Port {} : CSV_GetInstrumentName:{},{}'.format(self.serName,RC,Name) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : CSV_GetInstrumentName:{},{}'.format(self.serName,RC,Name) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC]) + Fore.RESET)

        return out

    # -----------------------------------------------------------------------------------------------
    # -- TMC Theodolite Measurement and Calculation; the core module for getting measurement data. --
    # -----------------------------------------------------------------------------------------------

    def TMC_GetFace(self):
        '''
        Cette fonction retourne la position de la lunette.

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'POS' : str
                    Position de l'instrument
                        '0' : Premiere position, postion I\n
                        '1' : Deuxieme position, position II
                'JulianDate' : float
                    Date julienne exprimee en jours

        '''
        self.msg_ascii = '%R1Q,2026:\r\n'
        RC = '4'
        while RC == '4':
            RC = self.SendToInstru()

        POS = RC.split(',')[1]
        RC = RC.split(',')[0]
        out = {}
        out.update({'RC':RC})
        out.update({'POS':POS})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : TMC_GetFace:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC]))
        if RC == '0' :
            print(Fore.GREEN + 'Port {} : TMC_GetFace:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : TMC_GetFace:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC]) + Fore.RESET)

        return out

    def TMC_SetAtmCorr(self, Lambda, Pressure, DryTemperature, WetTemperature):
        '''
        Cette fonction permet de definir les parametres de correction atmospherique.

        Parameters
        ----------
            Lambda : float
                Longueur d'onde de la mesure de distance [m]
            Pressure : float
                Pression absolue [hPa]
            DryTemperature : float
                Temperature de l'air [°C]
            WetTemperature : float
                Temperature du point de rosee [°C]

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        self.msg_ascii = '%R1Q,2028:{:.10f},{:.2f},{:.2f},{:.2f}\r\n'.format(Lambda, Pressure, DryTemperature, WetTemperature)
        RC = '4'
        while RC == '4':
            RC = self.SendToInstru()

        RC = RC.split(',')[0]
        out = {}
        out.update({'RC':RC})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : TMC_GetFace:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC]))
        if RC == '0' :
            print(Fore.GREEN + 'Port {} : TMC_SetAtmCorr:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : TMC_SetAtmCorr:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC]) + Fore.RESET)

        return out

    def TMC_SetEdmMode(self,Mode):
        '''
        Cette fonction permet de definir le mode de mesure electronique de distance.

        Parameters
        ----------
            Mode : int
                Mode de mesure
                    0 : Init value\n
                    1 : IR Standard Reflector Tape\n
                    2 : IR Standard\n
                    3 : IR Fast\n
                    4 : LO Standard\n
                    5 : RL Standard\n
                    6 : Standard repeated measurement\n
                    7 : IR Tacking\n
                    8 : RL Tracking\n
                    9 : Fast repeated measurement\n
                    10 : IR Average\n
                    11 : RL Average\n
                    12 : LO Average

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        self.msg_ascii = '%R1Q,2020:{:d}\r\n'.format(Mode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : TMC_SetEdmMode:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0':
            print(Fore.GREEN + 'Port {} : TMC_SetEdmMode:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : TMC_SetEdmMode:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def TMC_SetStation(self,E0,N0,H0,Hi):
        '''
        Cette fonction permet de definir les coordonnees de la station (Mise en station).

        Parameters
        ----------
            E0 : float
                Coordonnee Est de la station [m]
            N0 : float
                Coordonnee Nord de la station [m]
            H0 : float
                Altitude de la station [m]
            Hi : float
                Hauteur d'instrument [m]

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        '''
        self.msg_ascii = '%R1Q,2010:{:.4f},{:.4f},{:.4f},{:.4f}\r\n'.format(E0,N0,H0,Hi)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : TMC_SetStation:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : TMC_SetStation:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : TMC_SetStation:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def TMC_SetOrientation(self,HzOrientation):
        '''
        Cette fonction definit l'inconnue d'orientation de la station (omega0)

        Parameters
        ----------
            HzOrientation : float
                Inconnue d'orientation de la station [gon]

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours
        '''
        HzOrientation = HzOrientation*np.pi/200.0
        self.msg_ascii = '%R1Q,2113:{:.8f}\r\n'.format(HzOrientation)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : TMC_SetOrientation:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0':
            print(Fore.GREEN + 'Port {} : TMC_SetOrientation:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : TMC_SetOrientation:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    def TMC_DoMeasure(self,Command,Mode):
        '''
        Cette fonction effectue une mesure de distance en fonction du mode de mesure TMC (mesure unique de distance, tracking, etc.).

        Parameters
        ----------
            Command : int
                0 : Stop measurement program\n
                1 : Default DIST-measurement // program\n
                3 : TMC_STOP and clear data\n
                4 : Signal measurement (test // function)\n
                6 : (Re)start measurement task\n
                8 : Distance-TRK measurement // program\n
                10 : Reflectorless tracking\n
                11 : Frequency measurement (test)
            Mode : int
                Mesure du compensateur
                    0 : Use sensor (apriori sigma)\n
                    1 : Automatic mode (sensor/plane)\n
                    2 : Use plane (apriori sigma)

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        '''
        self.msg_ascii = '%R1Q,2008:{:d},{:d},0\r\n'.format(Command,Mode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : TMC_DoMeasure:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : TMC_DoMeasure:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : TMC_DoMeasure:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        #Attente de la fin de mesure
        time.sleep(5.0)

        return out

    #Fonction permettant de récupérer les valeurs de distance et d'angle de l'instrument
    def TMC_GetSimpleMea(self,WaitTime,Mode):
        '''
        Cette fonction retourne les mesures d'angle et de distance. Attention, cette commande n'effectue pas de nouvelle mesure.
        Elle recupere les valeurs actuelles enregistrees dans l'instrument. La fonction TMC_DoMEasure() doit etre lancee avant.

        Parameters
        ----------
            WaitTime : int
                Temps d'attente pour la fin de la mesure de distance.
            Mode : int
                Mesure du compensateur
                    0 : Use sensor (apriori sigma)\n
                    1 : Automatic mode (sensor/plane)\n
                    2 : Use plane (apriori sigma)

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'Hz' : float
                    direction horizontale [gon]
                'V' : float
                    Angle zénithal [gon]
                'SlopeDistance' : float
                    Distane inclinee [m]
                'JulianDate' : float
                    Date julienne exprimee en jours

        Raises
        ------
            IndexError
                Si la reponse sur le port serie en retour est tronquee.\n
                Dans ce cas : out['RC'] = NoData

        See Also
        --------
            TMC_DoMeasure() : Effectuer une mesure avant de lancer celle-ci.

        '''
        self.msg_ascii = '%R1Q,2108:{:d},{:d},0\r\n'.format(WaitTime,Mode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        try :
            out.update({'RC':RC[0]})
            out.update({'Hz':float(RC[1])*200/np.pi})
            out.update({'V':float(RC[2])*200/np.pi})
            out.update({'SlopeDistance':float(RC[3])})
            out.update({'JulianDate':self.getJulianDate()})

            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetSimpleMea:{},{:.4f},{:.4f},{:.4f}\n'.format(out['JulianDate'],self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi,float(RC[3])))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            if RC[0] == '0' :
                print(Fore.GREEN + 'Port {} : TMC_GetSimpleMea:{},{:.4f},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi,float(RC[3])) + Fore.RESET)
                print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
            else :
                print(Fore.RED + 'Port {} : TMC_GetSimpleMea:{},{:.4f},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi,float(RC[3])) + Fore.RESET)
                print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        except IndexError :
            out.update({'RC':'NoData'})
            out.update({'JulianDate':self.getJulianDate()})
            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetSimpleMea:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            print(Fore.RED + 'Port {} : TMC_GetSimpleMea:{}'.format(self.serName,out['RC']) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError['9999']) + Fore.RESET)

        return out

    def TMC_GetAngle5(self,Mode):
        '''
        Cette fonction effectue une mesure angulaire et renvoie le resultat.

        Parameters
        ----------
            Mode : int
                Mesure du compensateur
                    0 : Use sensor (apriori sigma)\n
                    1 : Automatic mode (sensor/plane)\n
                    2 : Use plane (apriori sigma)

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'Hz' : float
                    direction horizontale [gon]
                'V' : float
                    Angle zénithal [gon]
                'JulianDate' : float
                    Date julienne exprimee en jours

        Raises
        ------
            IndexError
                Si la reponse sur le port serie en retour est tronquee.\n
                Dans ce cas : out['RC'] = NoData

        '''
        self.msg_ascii = '%R1Q,2107:{:d}\r\n'.format(Mode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        try :
            out.update({'RC':RC[0]})
            out.update({'Hz':float(RC[1])*200/np.pi})
            out.update({'V':float(RC[2])*200/np.pi})
            out.update({'JulianDate':self.getJulianDate()})

            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetAngle5:{},{:.4f},{:.4f}\n'.format(out['JulianDate'],self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            if RC[0] == '0' :
                print(Fore.GREEN + 'Port {} : TMC_GetAngle5:{},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi) + Fore.RESET)
                print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
            else :
                print(Fore.RED + 'Port {} : TMC_GetAngle5:{},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi) + Fore.RESET)
                print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        except IndexError :
            out.update({'RC':'NoData'})
            out.update({'JulianDate':self.getJulianDate()})
            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetAngle5:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            print(Fore.RED + 'Port {} : TMC_GetAngle5:{}'.format(self.serName,out['RC']) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError['9999']) + Fore.RESET)

        return out

    #Fonction permettant de récupérer les valeurs angulaires
    def TMC_GetAngle1(self,Mode):
        '''
        Cette fonction effectue une mesure angulaire et selon le mode une mesure du compensateur et renvoie la reponse.

        Parameters
        ----------
            Mode : int
                Mesure du compensateur
                    0 : Use sensor (apriori sigma)\n
                    1 : Automatic mode (sensor/plane)\n
                    2 : Use plane (apriori sigma)

        Returns
        -------
            out : dict
                'RC' : str
                    Message d'erreur en reponse
                'Hz' : float
                    direction horizontale [gon]
                'V' : float
                    Angle zénithal [gon]
                'AngleAccuracy' : float
                    Precision angulaire de l'instrument [gon]
                'AngleTime' : float
                    Moment de la mesure angulaire [ms]
                'CrossIncline' : float
                    Mesure d'inclinaison de l'axe principale dans le plan perpendiculaire a l'axe de visee [gon]
                'lengthIncline' : float
                    Mesure d'inclinaison de l'axe principale dans le plan de l'axe de visee [gon]
                AcurracyIncline : float
                    Précision de la mesure du compensateur [gon]
                InclineTime : float
                    Moment de la mesure du compensateur [ms]
                FaceDef : str
                    Position de la lunette
                'JulianDate' : float
                    Date julienne exprimee en jours

        Raises
        ------
            IndexError
                Si la reponse sur le port serie en retour est tronquee.\n
                Dans ce cas : out['RC'] = NoData

        '''
        self.msg_ascii = '%R1Q,2107:{:d}\r\n'.format(Mode)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        try :
            out.update({'RC':RC[0]})
            out.update({'Hz':float(RC[1])*200/np.pi})
            out.update({'V':float(RC[2])*200/np.pi})
            out.update({'AngleAccuracy':float(RC[3])*200/np.pi})
            out.update({'AngleTime':float(RC[4])})
            out.update({'CrossIncline':float(RC[5])*200/np.pi})
            out.update({'LengthIncline':float(RC[6])*200/np.pi})
            out.update({'AccuracyIncline':float(RC[7])*200/np.pi})
            out.update({'InclineTime':float(RC[8])})
            out.update({'FaceDef': RC[9]})
            out.update({'JulianDate':self.getJulianDate()})

            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetAngle1:{},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n'.format(out['JulianDate'],self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi,float(RC[3])*200/np.pi,float(RC[4]),float(RC[5])*200/np.pi,float(RC[6])*200/np.pi,float(RC[7])*200/np.pi,float(RC[8]),float(RC[9])))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            if RC[0] == '0':
                print(Fore.GREEN + 'Port {} : TMC_GetAngle1:{},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi,float(RC[3])*200/np.pi,float(RC[4]),float(RC[5])*200/np.pi,float(RC[6])*200/np.pi,float(RC[7])*200/np.pi,float(RC[8]),float(RC[9])) + Fore.RESET)
                print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
            else :
                print(Fore.RED + 'Port {} : TMC_GetAngle1:{},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1])*200/np.pi,float(RC[2])*200/np.pi,float(RC[3])*200/np.pi,float(RC[4]),float(RC[5])*200/np.pi,float(RC[6])*200/np.pi,float(RC[7])*200/np.pi,float(RC[8]),float(RC[9])) + Fore.RESET)
                print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        except IndexError :
            out.update({'RC':'NoData'})
            out.update({'JulianDate':self.getJulianDate()})
            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetAngle1:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            print(Fore.RED + 'Port {} : TMC_GetAngle1:{}'.format(self.serName,out['RC']) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError['9999']) + Fore.RESET)

        return out

    #Fonction permettant de récupérer les coordonnées du point levé
    def TMC_GetSimpleCoord(self,WaitTime,eProg):
        '''
        Cette fonction retourne les coordonnees cartesiennes du point mesure (seulement valide si une mesure a ete effectue voir TMC_DoMeasure())

        Parameters
        ----------
            WaitTime : int
                Temps d'attente pour la fin de la mesure de distance.
            eProg : int
                Mesure du compensateur
                    0 : Use sensor (apriori sigma)\n
                    1 : Automatic mode (sensor/plane)\n
                    2 : Use plane (apriori sigma)

        Returns
        -------
            out : dict
                RC' : str
                    Message d'erreur en reponse
                'dCoordE' : float
                    Coordonnees Est [m]
                'dCoordN' : float
                    Coordonnees Nord [m]
                'dCoordH' : float
                    Altitude [m]
                'JulianDate' : float
                    Date julienne exprimee en jours

        See Also
        --------
            TMC_DoMeasure() : Effectuer une mesure avant de lancer celle-ci.

        Raises
        ------
            IndexError
                Si la reponse sur le port serie en retour est tronquee.\n
                Dans ce cas : out['RC'] = NoData

        '''
        self.msg_ascii = '%R1Q,2116:{:d},{:d},0\r\n'.format(WaitTime,eProg)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        try :
            out.update({'RC':RC[0]})
            out.update({'dCoordE':float(RC[1])})
            out.update({'dCoordN':float(RC[2])})
            out.update({'dCoordH':float(RC[3])})
            out.update({'JulianDate':self.getJulianDate()})

            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetSimpleCoord:{},{:.4f},{:.4f},{:.4f}\n'.format(out['JulianDate'],self.serName,RC[0],float(RC[1]),float(RC[2]),float(RC[3])))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            if RC[0] == '0' :
                print(Fore.GREEN + 'Port {} : TMC_GetSimpleCoord:{},{:.4f},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1]),float(RC[2]),float(RC[3])) + Fore.RESET)
                print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
            else :
                print(Fore.RED + 'Port {} : TMC_GetSimpleCoord:{},{:.4f},{:.4f},{:.4f}'.format(self.serName,RC[0],float(RC[1]),float(RC[2]),float(RC[3])) + Fore.RESET)
                print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        except IndexError :
            out.update({'RC':'NoData'})
            out.update({'JulianDate':self.getJulianDate()})
            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : TMC_GetSimpleCoord:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            print(Fore.RED + 'Port {} : TMC_GetSimpleCoord:{}'.format(self.serName,out['RC']) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError['9999']) + Fore.RESET)

        return out

    # --------------------------------------------------------------
    # -- DNA Groupe de fonctions exécutables sur niveau numérique --
    # --------------------------------------------------------------

    #Fonction permettant de commencer les mesures de nivellement
    def DNA_StartMeasurement(self):
        '''
        Cette fonction permet d'effectuer une mesure avec un niveau type DNA ou LS.
        Pour recuperer les valeurs, utiliser DNA_GetMeasResult().

        Returns
        -------
            out : dict
                RC' : str
                    Message d'erreur en reponse
                'JulianDate' : float
                    Date julienne exprimee en jours

        See Also
        --------
            Pour recuperer les valeurs, utiliser DNA_GetMeasResult().

        '''
        self.msg_ascii = '%R1Q,29036:\r\n'
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        out.update({'RC':RC[0]})
        out.update({'JulianDate':self.getJulianDate()})

        #Ecriture dans le fichier log
        # self.log.write('JD : {:.9f}, Port {} : DNA_StartMeasurement:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
        # self.log.write('{}\n'.format(self.DictError[RC[0]]))
        if RC[0] == '0' :
            print(Fore.GREEN + 'Port {} : DNA_StartMeasurement:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
        else :
            print(Fore.RED + 'Port {} : DNA_StartMeasurement:{}'.format(self.serName,RC[0]) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        return out

    #Fonction permettant de récupérer les mesures d'un niveau
    def DNA_GetMeasResult(self,WaitTime):
        '''
        Cette fonction permet de recuperer la mesure d'un niveau type DNA ou LS.
        N'est possible que si une mesure a ete faite DNA_StartMeasurement().

        Parameters
        ----------
            WaitTime : int
                Temps d'attente pour la fin de la mesure.

        Returns
        -------
            out : dict
                RC' : str
                    Message d'erreur en reponse
                'heightReading' : float
                    Mesure de hauteur sur latte a code barre [m]
                'distReading' : float
                    Mesure de distance [m]
                'JulianDate' : float
                    Date julienne exprimee en jours

        See Also
        --------
            Pour effectuer une mesure utiliser DNA_StartMeasurement()

        Raises
        ------
            IndexError
                Si la reponse sur le port serie en retour est tronquee.\n
                Dans ce cas : out['RC'] = NoData

        '''
        self.msg_ascii = '%R1Q,29005:{:d}\r\n'.format(WaitTime)
        RC = '4'
        while RC == '4' :
            RC = self.SendToInstru()

        RC = RC.split(',')
        out = {}
        try :
            out.update({'RC':RC[0]})
            out.update({'heightReading':float(RC[1])})
            out.update({'distReading':float(RC[2])})
            out.update({'JulianDate':self.getJulianDate()})

            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : DNA_GetMeasResult:{},{:.6f},{:.3f}\n'.format(out['JulianDate'],self.serName,RC[0],float(RC[1]),float(RC[2])))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            if RC[0] == '0' :
                print(Fore.GREEN + 'Port {} : DNA_GetMeasResult:{},{:.6f},{:.3f}'.format(self.serName,RC[0],float(RC[1]),float(RC[2])) + Fore.RESET)
                print(Fore.GREEN + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)
            else :
                print(Fore.RED + 'Port {} : DNA_GetMeasResult:{},{:.6f},{:.3f}'.format(self.serName,RC[0],float(RC[1]),float(RC[2])) + Fore.RESET)
                print(Fore.RED + '{}\n'.format(self.DictError[RC[0]]) + Fore.RESET)

        except IndexError :
            out.update({'RC':'NoData'})
            out.update({'JulianDate':self.getJulianDate()})
            #Ecriture dans le fichier log
            # self.log.write('JD : {:.9f}, Port {} : DNA_GetMeasResult:{}\n'.format(out['JulianDate'],self.serName,RC[0]))
            # self.log.write('{}\n'.format(self.DictError[RC[0]]))
            print(Fore.RED + 'Port {} : DNA_GetMeasResult:{}'.format(self.serName,out['RC']) + Fore.RESET)
            print(Fore.RED + '{}\n'.format(self.DictError['9999']) + Fore.RESET)

        return out
