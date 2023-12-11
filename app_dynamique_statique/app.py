#Flask Librairies
from flask import Flask, render_template, Response, request, redirect, url_for, send_file
from flask_socketio import SocketIO, emit
#Commons librairies
import json
import io
import base64
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
#Personal Librairies
import Drone_RTK_RPI_thread as drone
import test_write_pdf as pdf
import os

#Permet de nettoyer les fichiers info et résultat
def init_file():
    '''
    Fonction permettant de nettoyer les fichiers temporaires infos et résultats.
    '''
    with open('temp_info.json','w') as file_info:
        info_acquis_dict=  { "GPS_connect" : "NO", "Tacheo_connect" : "NO", "RTK" : "NO", "Prisme_lock_and_compensateur" : "NO", "Enregistrement_des_points" : "NO", "Nombre_GPS" : '0', "Echelle" : "0"}
        file_info.seek(0)
        json.dump(info_acquis_dict,file_info)
        file_info.truncate()
    with open('temp_res.json','w') as f:
        info_res_dict = {"Nombre_points_GNSS" : "0", "Nombre_points_Tacheo" : "0"}
        f.seek(0)  # rewind
        json.dump(info_res_dict,f)
        f.truncate()

def create_plot(x_st, y_st, P_GPS, P_tacheo):
    '''
    Fonction permettant de créer une figure du résultat de l'ICP, affichable dans l'interface graphique Web de Flask.

    Parameters
    ----------
        x_st : Float
            Coordonnée Est de la Station calculée [m].
        y_st : Float
            Coordonnée Nord de la Station calculée [m].
        P_GPS : list of Vector
            Time : float
                Temps GPS du point [s].
            X : float
                Coordonnée Est du point [m].
            Y : float
                Coordonnée Nord du point [m].
            Z : float
                altitude RAN95 du point [m].
        P_Tacheo : list of Vector
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
        img : BytesIO
            Image lisible directement par l'interface graphique Web de Flask
    '''
    fig = Figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.set_title("Résultat ICP")
    axis.set_xlabel("Coordonnée E [m]", rotation = 0)
    axis.set_ylabel("Coordonnée N [m]", rotation = 90)
    axis.grid()
    axis.plot(P_GPS[:,1], P_GPS[:,2], ".r", label='GNSS')
    axis.plot(P_tacheo[:,0], P_tacheo[:,1], ".g", label='Station totale')
    axis.plot(x_st, y_st, "Db", label='Stationnement')
    axis.legend(loc='best')#doc : https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html
    #Formater les valeurs des axes x et y
    current_values = fig.gca().get_yticks()
    fig.gca().set_yticklabels(['{:.0f}'.format(x) for x in current_values])
    current_values = fig.gca().get_xticks()
    fig.gca().set_xticklabels(['{:.0f}'.format(x) for x in current_values], rotation=45.0)
    #ajouter un padding afin de ne pas cropper l'image
    fig.subplots_adjust(left=0.16)
    fig.subplots_adjust(bottom=0.2)
    fig.savefig('ICP.png')#for pdf creation
    # Convert plot to PNG image
    pngImage = io.BytesIO()
    FigureCanvas(fig).print_png(pngImage)
    # Encode PNG image to base64 string
    pngImageB64String = "data:image/png;base64,"
    pngImageB64String += base64.b64encode(pngImage.getvalue()).decode('utf8')
    return pngImageB64String



init_file()
try:
    os.remove('ICP.png')
except:
    print('no ICP graph to remove')
try:
    os.remove('retour.pdf')
except:
    print('no pdf to remove')
app = Flask(__name__)
socketio = SocketIO(app)



@app.route("/")
def index():
    '''
    Fonction créant la page web d'accueil du serveur Flask.
    '''
    init_file()
    return render_template('index.html')

@app.route("/acquisition/", methods=['POST'])
def acquisition():
    '''
    Fonction appelée lors du clic sur le bouton acquisition, lance l'acquisition et le retour des informations sur la page web du serveur Flask.
    '''
    try:
        os.remove('ICP.png')
    except:
        print('no ICP graph to remove')
    try:
        os.remove('retour.pdf')
    except:
        print('no pdf to remove')
    init_file()
    drone.start_acquisition()
    return render_template('index.html', info = 'Acquisition en cours voir les boutons pour le statut.');


@app.route("/calcul/", methods=['POST'])
def calcul():
    '''
    Fonction appelée lors du clic sur le bouton calcul, lance le calcul et le retour des résultats sur la page web du serveur Flask.
    '''
    #Get the radiobutton status
    global AcquiType
    AcquiType = request.form['AcquisType']#'STAT' ou 'DYN'
    print(AcquiType)
    ecart_antenne_prisme = int(request.form['ecart'])/1000
    global x_res, y_res, z_res, g0, fitness, inlier_rmse, len_correspondence, img
    x_res,y_res,z_res,g0, fitness, inlier_rmse, len_correspondence, P_global, P_local = drone.start_calcul(ecart_antenne_prisme,AcquiType)
    img = create_plot(float(x_res), float(y_res), P_global, P_local)
    pdf.write_pdf(AcquiType, x_res, y_res, z_res, g0, fitness, inlier_rmse, len_correspondence)
    return render_template('index.html', res_E = str(x_res), res_N = str(y_res), res_H = str(z_res), res_g0 = str(g0), res_fitness = fitness, res_rmse = inlier_rmse, res_corres = len_correspondence, image = img);

@app.route("/redo/", methods=['POST'])
def redo():
    '''
    Fonction appelée lors du clic sur le bouton Mise en station, lance la mise en station de l'instrument.
    '''
    #init_file()
    bool_test_st = drone.SetNewStation(x_res,y_res,z_res,g0)
    if bool_test_st == True :
        pass
    else :
        while bool_test_st == False :
            bool_test_st = drone.SetNewStation(x_res,y_res,z_res,g0)
    return render_template('index.html', res_E = str(x_res), res_N = str(y_res), res_H = str(z_res), res_g0 = str(g0), res_fitness = fitness, res_rmse = inlier_rmse, res_corres = len_correspondence, image = img);

@app.route('/download/')
def download():
    path = 'retour.pdf'
    return send_file(path, as_attachment=True)
############################Configuration des sockets##########################
@socketio.on('connect', namespace='/getInfo')
def connect_get_info():
    print('Client connected')
    return

# Disconnect Client Info
@socketio.on('disconnect', namespace='/getInfo')
def test_disconnect():
    print('Client disconnected')

@socketio.on('ctrl', namespace='/getInfo')
def handle_crtl_evt(info_json):
    #Read file info and file result
    with open('temp_info.json','r') as f:
        dictio_info = json.load(f)
    with open('temp_res.json','r') as f:
        dictio_res_info = json.load(f)
    GPS_connect = dictio_info['GPS_connect'] #ok ou no
    Tacheo_connect = dictio_info['Tacheo_connect'] #ok ou no
    RTK = dictio_info['RTK'] #ok float ou no
    Prisme_lock_and_compensateur = dictio_info['Prisme_lock_and_compensateur'] #ok ou no
    Enregistrement_des_points = dictio_info['Enregistrement_des_points'] #ok ou no
    n_GPS = dictio_info["Nombre_GPS"] #nombre de points GPS RTK
    echelle = dictio_info["Echelle"]
    n_GNSS = dictio_res_info['Nombre_points_GNSS']
    n_Tacheo = dictio_res_info['Nombre_points_Tacheo']
    msg = {'GPS_connect': GPS_connect, 'Tacheo_connect' : Tacheo_connect, "RTK" : RTK, "Prisme_lock_and_compensateur" : Prisme_lock_and_compensateur, "Enregistrement_des_points" : Enregistrement_des_points, "Nombre_GPS" : n_GPS, "Echelle" : echelle, 'Nombre_points_GNSS' : n_GNSS, 'Nombre_points_Tacheo' : n_Tacheo}
    socketio.emit('info', msg, namespace='/getInfo')

if __name__ == '__main__':
    socketio.run(app, host="0.0.0.0")#accessible à l'adresse ip du rpi sur un autre appareil
