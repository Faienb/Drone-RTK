#Test ecrire pdf with Python
from fpdf import FPDF  # fpdf class
import io

class PDF(FPDF):
    def header(self):
        # Logos
        self.image('logo_suisse.png', 10, 12, 50)#x,y,width
        self.image('logo_insit.JPG', 150, 8, 50)#x,y,width
        # Arial bold 15
        self.set_font('Arial', 'B', 15)
        # Move to the right
        self.cell(80)
        # Title
        self.cell(30, 40, 'Résultat de la mise en station', 0, 0, 'C')
        # Line break
        self.ln(30)

    # Page footer
    def footer(self):
        # Position at 1.5 cm from bottom
        self.set_y(-15)
        # Arial italic 8
        self.set_font('Arial', 'I', 8)
        # Page number
        self.cell(0, 10, 'Page ' + str(self.page_no()) + '/{nb}', 0, 0, 'C')

    def load_resource(self, reason, filename):
        if reason == "image":
            if filename.startswith("http://") or filename.startswith("https://"):
                f = io.BytesIO(urlopen(filename).read())
            elif filename.startswith("data"):
                f = filename.split('base64,')[1]
                f = base64.b64decode(f)
                f = io.BytesIO(f)
            else:
                f = open(filename, "rb")
            return f
        else:
            self.error("Unknown resource loading reason \"%s\"" % reason)

def write_pdf(type_acquis, est, nord, alt, g0, fitness, rmse, nb_pt_utilise):
    pdf = PDF()
    pdf.alias_nb_pages()
    pdf.add_page()
    pdf.set_font('Times', '', 12)
    pdf.multi_cell(0, 7, 'La méthode utilisée pour mettre en station votre appareil a été développée par l\'HEIG-VD, l\'utilisation de cette méthode \
    n\'affranchit pas l\'utilisation de mesures de contrôles qui garantissent le travail du géomètre. Le calcul de cette mise en station a été faite \
    avec la méthode statique, plusieurs analyses et critères de précision sont fourni ci-dessous. Si la mise en station n\'est pas bonne, recommencer avec une meilleure configuration. \
    Si vous avez des retours sur cette méthode, merci de nous les communiquer à cette adresse mail : antoine.carreaud@heig-vd.ch.' , 0, 'J')
    if type_acquis == 'STAT':
        pdf.multi_cell(0, 7, 'Vous avez choisi une mise en station avec la méthode stop and go.' , 0, 'J')
    elif type_acquis == 'DYN':
        pdf.multi_cell(0, 7, 'Vous avez choisi une mise en station avec la méthode dynamique.' , 0, 'J')
    pdf.ln(5)
    pdf.set_font('Times','B',13)
    pdf.cell(0,0,'Résultat de la mise en station : ',0,2)
    pdf.ln(8)
    pdf.set_font('Times', '', 12)
    pdf.cell(0,0,'EST (MN95) : '+str(est)+' [m]',0,1)
    pdf.ln(8)
    pdf.cell(0,0,'NORD (MN95) : '+str(nord)+' [m]',0,1)
    pdf.ln(8)
    pdf.cell(0,0,'Altitude (RAN95) : '+str(alt)+' [m]',0,1)#voir pour ajouter NF02
    pdf.ln(8)
    pdf.cell(0,0,'G0 de la station : '+str(g0)+' [gon]',0,1)
    pdf.ln(8)
    if float(fitness)>50:
        pdf.set_fill_color(0,255,0)
    else:
        pdf.set_fill_color(255,0,0)#r,g,b
    pdf.cell(80,8,'Fitness : '+str(fitness)+' %',1,1,fill=True)
    pdf.cell(80,8,'Nombre de couples de points appairés : '+str(nb_pt_utilise),1,1,fill=True)
    if float(rmse)<30 and float(rmse)>0.0:
        pdf.set_fill_color(0,255,0)
    else:
        pdf.set_fill_color(255,0,0)#r,g,b
    pdf.cell(80,8,'RMS of inlier : '+str(rmse)+' mm',1,1,fill=True)
    pdf.image('ICP.png',15,None,190)
    pdf.output('retour.pdf', 'F')
