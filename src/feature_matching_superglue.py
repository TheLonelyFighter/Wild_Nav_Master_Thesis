
import numpy as np
import cv2 
import matplotlib.pyplot as plt
import csv
import superglue_utils
import time
import color_mask
import imutils



class GeoPhotoDrone:
    """Stores a drone photo together with GNSS location and camera rotation parameters
    """
    def __init__(self, filename, photo = 0, latitude = 0, longitude = 0 , altitude = 0 ,gimball_roll = 0, gimball_yaw = 0, gimball_pitch = 0, flight_roll = 0, flight_yaw = 0, flight_pitch = 0):
        self.filename = filename
        self.photo = photo
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.gimball_roll = gimball_roll
        self.gimball_yaw = gimball_yaw
        self.gimball_pitch = gimball_pitch
        self.flight_roll = flight_roll
        self.flight_yaw = flight_yaw
        self.flight_pitch = flight_pitch

    def __str__(self):
        return "%s; \nlatitude: %f \nlongitude: %f \naltitude: %f \ngimball_roll: %f \ngimball_yaw: %f \ngimball_pitch: %f \nflight_roll: %f \nflight_yaw: %f \nflight_pitch: %f" % (self.filename, self.latitude, self.longitude, self.altitude, self.gimball_roll, self.gimball_yaw, self.gimball_pitch, self.flight_roll, self.flight_yaw, self.flight_pitch )
        
class GeoPhoto:
    """Stores a satellite photo together with (latitude, longitude) for top_left and bottom_right_corner
    """
    def __init__(self, filename, photo, geo_top_left, geo_bottom_right):
        self.filename = filename
        self.photo = photo
        self.top_left_coord = geo_top_left
        self.bottom_right_coord = geo_bottom_right

    def __str__(self):
        return "%s; \n\ttop_left_latitude: %f \n\ttop_left_lon: %f \n\tbottom_right_lat: %f \n\tbottom_right_lon %f " % (self.filename, self.top_left_coord[0], self.top_left_coord[1], self.bottom_right_coord[0], self.bottom_right_coord[1])

def csv_read_drone_images(filename):
    """Builds a list with drone geo tagged photos by reading a csv file with this format:
    Filename, Top_left_lat,Top_left_lon,Bottom_right_lat,Bottom_right_long
    "big_photo_2.png",60.506787,22.311631,60.501037,22.324467
    """
    geo_list_drone = []
    photo_path = "../photos/query/real_dataset_1/matrice_300_session_1/"
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:                
                #img = cv2.imread(photo_path + row[0],0)
                geo_photo = GeoPhotoDrone(photo_path + row[0], 0, float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8]), float(row[9]))
                geo_list_drone.append(geo_photo)
                line_count += 1

        print(f'Processed {line_count} lines.')
        return geo_list_drone

def csv_read_sat_map(filename):
    """Builds a list with geo tagged photos by reading a csv file with this format:
    Filename, Top_left_lat,Top_left_lon,Bottom_right_lat,Bottom_right_long
    "big_photo_2.png",60.506787,22.311631,60.501037,22.324467
    """
    geo_list = []
    photo_path = "../photos/map/"
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:                
                img = cv2.imread(photo_path + row[0],0)
                geo_photo = GeoPhoto(photo_path + row[0],img,(float(row[1]),float(row[2])), (float(row[3]), float(row[4])))
                geo_list.append(geo_photo)
                line_count += 1

        print(f'Processed {line_count} lines.')
        return geo_list

def calculate_geo_pose(geo_photo, center):
    latitude = geo_photo.top_left_coord[0] + (center[1] / geo_photo.photo.shape[0]) * ( geo_photo.bottom_right_coord[0] - geo_photo.top_left_coord[0])
    longitude = geo_photo.top_left_coord[1] + (center[0] / geo_photo.photo.shape[1]) * ( geo_photo.bottom_right_coord[1] - geo_photo.top_left_coord[1])
    return latitude, longitude





#Read all the geo tagged images that make up the sattelite map used for reference
geo_images_list = csv_read_sat_map("../photos/map/real_dataset/map.csv")
drone_images_list = csv_read_drone_images("../photos/query/real_dataset_1/matrice_300_session_1/drone_image_test.csv")
print(drone_images_list[7])
drone_image = cv2.imread("../photos/query/real_dataset_1/matrice_300_session_1/photo_2.JPG", 0)

#write the query image to the map folder
#the query will be matched to every sattelite map image
drone_image = imutils.rotate(drone_image, 0)

for drone_image in drone_images_list:
    photo =  cv2.imread(drone_image.filename)
    cv2.imwrite("../photos/map/real_dataset/1_query_image.jpg", photo)
    satellite_map_index, center = superglue_utils.match_image()
    if center != None:
        print("Located", center, satellite_map_index)
        print("Length of geo_images_list", len(geo_images_list))
        current_location = calculate_geo_pose(geo_images_list[satellite_map_index], center)
        print("Geographical location: ", current_location)
        # cv2.imshow("Sat map", geo_images_list[satellite_map_index].photo)
        # cv2.waitKey()

