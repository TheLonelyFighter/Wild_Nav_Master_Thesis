
import numpy as np
import cv2 
import matplotlib.pyplot as plt
import csv
import superglue_utils
import time
import color_mask
import imutils


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

def csv_read(filename):
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
geo_images_list = csv_read("../photos/map/map_2_castle.csv")
drone_image = cv2.imread("../photos/query/real_dataset_1/matrice_300_session_1/photo_2.JPG", 0)

#write the query image to the map folder
#the query will be matched to every sattelite map image
drone_image = imutils.rotate(drone_image, 0)

cv2.imwrite("../photos/map/real_dataset/1_query_image.jpg", drone_image)
satellite_map_index, center = superglue_utils.match_image()
if center != None:
    print("Located", center, satellite_map_index)
    current_location = calculate_geo_pose(geo_images_list[satellite_map_index], center)
    print("Geographical location: ", current_location)
    # cv2.imshow("Sat map", geo_images_list[satellite_map_index].photo)
    # cv2.waitKey()

