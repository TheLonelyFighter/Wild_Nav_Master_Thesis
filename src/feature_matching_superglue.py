
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
        self.latitude_calculated = -1
        self.longitude_calculated = -1
        self.altitude = altitude
        self.gimball_roll = gimball_roll
        self.gimball_yaw = gimball_yaw
        self.gimball_pitch = gimball_pitch
        self.flight_roll = flight_roll
        self.flight_yaw = flight_yaw
        self.flight_pitch = flight_pitch
        self.matched = False

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

    def __lt__(self, other):
         return self.filename < other.filename

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
    photo_path = "../photos/map/real_dataset/"
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        print(csv_reader)
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:   
                print(row[0])             
                img = cv2.imread(photo_path + row[0],0)
                geo_photo = GeoPhoto(photo_path + row[0],img,(float(row[1]),float(row[2])), (float(row[3]), float(row[4])))
                geo_list.append(geo_photo)
                line_count += 1

        print(f'Processed {line_count} lines.')
        geo_list.sort() # sort alphabetically by filename to ensure that the feature matcher return the right index of the matched sat image
        return geo_list
    
def csv_write_image_location(photo_list):
    header = ['Filename', 'Latitude', 'Longitude', 'Calculated_Latitude', 'Calculated_Longitude']
    with open('calculated_coordinates.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for photo in photo_list:
            photo_name = photo.filename.split("/")[-1]
            line = [photo_name, str(photo.latitude), str(photo.longitude), str(photo.latitude_calculated), str(photo.longitude_calculated)]
            writer.writerow(line)

def calculate_geo_pose(geo_photo, center, features_mean,  shape):
     #use ratio here instead of pixels because image is reshaped in superglue
    query_lat = 0.001
    query_lon = 0.00263
    latitude = geo_photo.top_left_coord[0] + center[1]  * ( geo_photo.bottom_right_coord[0] - geo_photo.top_left_coord[0])
    longitude = geo_photo.top_left_coord[1] + center[0]  * ( geo_photo.bottom_right_coord[1] - geo_photo.top_left_coord[1])
    corrected_lat = latitude -  ((query_image.shape[1] / 2  - features_mean[1]) / shape[1]) * query_lat
    corrected_lon = longitude + ((query_image.shape[0] / 2  - features_mean[0]) / shape[0])  * query_lon
    print("Old coord: ", center, latitude, longitude)
    print("New coord: ", corrected_lat, corrected_lon)
    #input("press enter ")
    return latitude, longitude





#Read all the geo tagged images that make up the sattelite map used for reference
geo_images_list = csv_read_sat_map("../photos/map/real_dataset/map.csv")
drone_images_list = csv_read_drone_images("../photos/query/real_dataset_1/matrice_300_session_1/drone_image_test.csv")
print(drone_images_list[7])
#drone_image = cv2.imread("../photos/query/real_dataset_1/matrice_300_session_1/photo_2.JPG", 0)

#write the query image to the map folder
#the query will be matched to every sattelite map image
#drone_image = imutils.rotate(drone_image, 0)
latitude_truth = []
longitude_truth = []
latitude_calculated = []
longitude_calculated = []

print(drone_images_list)
for drone_image in drone_images_list:
    latitude_truth.append(drone_image.latitude)
    longitude_truth.append(drone_image.longitude)
    photo =  cv2.imread(drone_image.filename)
    #photo = imutils.rotate(photo, drone_image.flight_yaw + drone_image.gimball_yaw )
    
    #Try different rotations
    
    photo = imutils.rotate(photo, -20 )
    cv2.imwrite("../photos/map/real_dataset/1_query_image.png", photo)
    satellite_map_index, center, located_image, features_mean, query_image = superglue_utils.match_image()
    photo_name = drone_image.filename.split("/")[-1]
    if center != None:        
        current_location = calculate_geo_pose(geo_images_list[satellite_map_index], center, features_mean, query_image.shape )
        cv2.putText(located_image, str(current_location), org = (10,625),fontFace =  cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8,  color = (0, 0, 0))
        cv2.imwrite("../results/" + photo_name + "_result.png", located_image)
        cv2.imwrite("test_image_loc.png", located_image)
        print("Located", photo_name,  center, satellite_map_index, features_mean)
        #cv2.circle(query_image, (int(features_mean[0]), int(features_mean[1])), radius = 10, color = (255, 0, 0), thickness = 2)
        # cv2.imshow("query_image", query_image)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        input("Press Enter to continue")        
        print("Geographical location: ", current_location)
        drone_image.matched = True
        print(current_location[0])
        print(current_location[1])
        #input("Press Enter to continue...")
        drone_image.latitude_calculated = current_location[0]
        drone_image.longitude_calculated = current_location[1]
        latitude_calculated.append(drone_image.latitude_calculated)
        longitude_calculated.append(drone_image.longitude_calculated)
        #cv2.imshow("Sat map", geo_images_list[satellite_map_index].photo)
        #cv2.waitKey()
    else:
        print("NOT MATCHED:", photo_name)


print(latitude_truth)
print(longitude_truth)
print(latitude_calculated)
print(longitude_calculated)

csv_write_image_location(drone_images_list)



