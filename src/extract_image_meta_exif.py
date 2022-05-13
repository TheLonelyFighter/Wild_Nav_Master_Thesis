import subprocess
import re
import os


csv_filename = "matrice_wide_session_1.txt"

photo_folder = '/home/marius/Desktop/4_may_dataset/matrice_300/matrice_wide_session_1/'

def convert_gnss_coord(lat_or_lon):
    deg, deg_string, minutes,discard_1,  seconds, discard_2, direction =  re.split('[\s°\'"]', lat_or_lon)
    converted = (float(deg) + float(minutes)/60 + float(seconds)/(60*60)) * (-1 if direction in ['W', 'S'] else 1)
    return str(converted)

def load_images_from_folder(folder):
    images_list = []
    for filename in os.listdir(folder):
        images_list.append(filename)
    return images_list
    

converted = convert_gnss_coord('''60 deg 24' 8.13" N''')
print(converted)


images_list = load_images_from_folder(photo_folder)
print(images_list)

f = open(csv_filename, "a")

filesize = os.path.getsize(csv_filename)
if filesize == 0:
    f.write("Filename,Latitude,Longitude,Altitude,Gimball_Roll,Gimball_Yaw,Gimball_Pitch,Flight_Roll,Flight_Yaw,Flight_Pitch")

for image in images_list:
    infoDict = {} #Creating the dict to get the metadata tags
    exifToolPath = 'exiftool'
    imgPath = photo_folder + image
    process = subprocess.Popen([exifToolPath,imgPath],stdout=subprocess.PIPE, stderr=subprocess.STDOUT,universal_newlines=True) 
    ''' get the tags in dict '''
    for tag in process.stdout:
        line = tag.strip().split(':')
        infoDict[line[0].strip()] = line[-1].strip()

    filename= infoDict['File Name']
    lat = infoDict['GPS Latitude']
    lon = infoDict['GPS Longitude']
    altitude = infoDict['Relative Altitude']
    gimball_roll = infoDict['Gimbal Roll Degree']
    gimball_yaw = infoDict['Gimbal Yaw Degree']
    gimball_pitch = infoDict['Gimbal Roll Degree']
    flight_roll = infoDict['Flight Roll Degree']
    flight_yaw = infoDict['Flight Yaw Degree']
    flight_pitch = infoDict['Flight Pitch Degree']

    lat = convert_gnss_coord(lat)
    lon = convert_gnss_coord(lon)

    f.write('\n' + filename + ',' + lat + ',' + lon + ',' + altitude + ',' + gimball_roll + ',' + gimball_yaw + ',' + gimball_pitch + ',' + flight_roll + ','  + flight_yaw + ','+ flight_pitch)

f.close()


