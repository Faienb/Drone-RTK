import serial
import ntplib
import time
from datetime import datetime, timezone
from pyubx2 import UBXReader
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
def readTimeFromGGA(trame):
	time = trame.split(',')[1] #str
	SecondUTC = int(time[0:2])*3600 + int(time[2:4])*60 + int(time[4:6]) + int(time[7:9])/100
	return format(float(SecondUTC),".3f")
def isGGA(trame):
	if trame.split(',')[0] == '$GNGGA':
		return True
	else:
		return False
def readPositionFromGGA(trame):
    lat = trame.split(',')[2] #str
    lat = float(lat[0:2]) + float(lat[2:])/60 #degres décimal float
    lon = trame.split(',')[4] #str
    lon = float(lon[0:3]) + float(lon[3:])/60 #degres décimal float
    alt = float(trame.split(',')[9]) #float
    return [format(lat,".8f"), format(lon,".8f"), format(alt,".3f")]
def isRTK(trame):
    code = trame.split(',')[6]
    if code == '4':#code positionnement RTK
        return True
    else:
        return False
def RequestTimefromNtp(addr='0.de.pool.ntp.org'):
    REF_TIME_1970 = 2208988800  # Reference time
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = b'\x1b' + 47 * b'\0'
    client.sendto(data, (addr, 123))
    data, address = client.recvfrom(1024)
    if data:
        t = struct.unpack('!12I', data)[10]
        t -= REF_TIME_1970
    return time.ctime(t), t
def openXbee(port,baudrate,count_GPS_disconnected):
	try:
		ser_GPS = serial.Serial(
			port = port,#'/dev/ttyS0'
			baudrate =baudrate,#115200
			parity = serial.PARITY_NONE,
			stopbits = serial.STOPBITS_ONE,
			bytesize = serial.EIGHTBITS,
			timeout = 0.01
		)
		if not ser_GPS.isOpen():
			ser_GPS.open()
		if not '$' in ser_GPS.readline().decode('utf-8'):
			if count_GPS_disconnected == 1:
				print(f'{bcolors.WARNING}[Warning] Xbee is not connected or wrong port name or activate ttyS0 port on rpi.{bcolors.ENDC}')
			return False
		else:
			print(f'{bcolors.OKGREEN}[INFO] Xbee is open and connected: {bcolors.ENDC}',ser_GPS.isOpen())
		return ser_GPS
	except:
		print(f'{bcolors.WARNING}[Warning] Xbee is not connected or wrong port name or activate ttyS0 port on rpi.{bcolors.ENDC}')
		return False
def ask4observationGPS(ser,count_GPS_is_ok,count_GPS_not_ok):
	gnss_bin_data = ser.readline()
	gnss_asc_data = gnss_bin_data.decode('utf-8')
	if count_GPS_not_ok == 1:
		print(f'{bcolors.WARNING}[Warning] RTK position is not yet available.{bcolors.ENDC}')
	if isGGA(gnss_asc_data):
		if isRTK(gnss_asc_data):
			if count_GPS_is_ok == 1:
				print(f'{bcolors.OKGREEN}[INFO] RTK position is now available.{bcolors.ENDC}')
				#print(f'{bcolors.OKGREEN}[INFO] Start recording points.{bcolors.ENDC}')
			return [readPositionFromGGA(gnss_asc_data),readTimeFromGGA(gnss_asc_data),time.time()]
		else:
			return 'GGA'
	else:
		if 'ZDA' in gnss_asc_data:
			print('time ZDA '+gnss_asc_data+' '+str(time.time()))
		return False
c = ntplib.NTPClient()
ser_GPS = False
while not ser_GPS:
    ser_GPS = openXbee('/dev/ttyS0',115200,0)
    ser_GPS2 = serial.Serial('/dev/ttyACM0',115200, timeout = 0.1)
# Provide the respective ntp server ip in below function
while 1:
    response = c.request('ch.pool.ntp.org', version=3)
    response.offset
    msg = ser_GPS2.readline()
    print('time gps 2 : '+str(UBXReader.parse(msg)))
    ntp_time = datetime.fromtimestamp(response.tx_time)
    data_GPS = ask4observationGPS(ser_GPS,10,0)
    if data_GPS:
        print(ntp_time,data_GPS[1:])
