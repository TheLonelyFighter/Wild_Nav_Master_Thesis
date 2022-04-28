import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import imutils
import csv


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
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:                
                img = cv.imread(row[0],0)
                geo_photo = GeoPhoto(row[0],img,(float(row[1]),float(row[2])), (float(row[3]), float(row[4])))
                geo_list.append(geo_photo)
                line_count += 1

        print(f'Processed {line_count} lines.')
        return geo_list

def good_sift_matches(img1, geo_photo):
    """
    Search small img1 in big geo_photo and, if found, return the geographical coordinates of the center matched photo.
    If not found return None, None
    """
    img2 = geo_photo.photo
    localized = False
    MIN_MATCH_COUNT = 10 # this is important, is less matches are 
    # Initiate SIFT detector
    sift = cv.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        localized = True
        print("Number of good matches found: ", len(good))
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts,M)
        img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)
        center = (((dst[0][0][0] + dst[3][0][0]) / 2), (dst[0][0][1] + dst[2][0][1]) / 2)
        print(center)
        current_lat = geo_photo.top_left_coord[0] + (center[1] / geo_photo.photo.shape[0]) * ( geo_photo.bottom_right_coord[0] - geo_photo.top_left_coord[0])
        current_lon = geo_photo.top_left_coord[1] + (center[0] / geo_photo.photo.shape[1]) * ( geo_photo.bottom_right_coord[1] - geo_photo.top_left_coord[1])
    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
    img3 = cv.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
    plt.imshow(img3, 'gray')
    plt.show()

    if localized:
        return (current_lat,current_lon)
    else:
         return None,None

geo_images_list = csv_read("photo_coordinates.csv")
print(geo_images_list[0])

for photo in geo_images_list:
    print(photo)

img1 = cv.imread('big_photo_2.png',0)          # queryImage
img2 = cv.imread('small_photo_6.png',0) # trainImage


#Geographical coordinates of the big sattelite patch
geo_top_left = (60.506787,   22.311631)
geo_bottom_right = (60.501037,  22.324467)


geo_photo = GeoPhoto("example",img1, geo_top_left, geo_bottom_right)



current_position = good_sift_matches(img2,geo_images_list[3])
# print("Matches: ",len(good_matches))

print(current_position)


#Rotated images also work, but it's better to use images with the same orientation as
#the sattelite view; this way you get more good matches
# rotation_list = np.zeros(100)#np.arange(0,360,10)
# for rot in rotation_list:
#     rotated = imutils.rotate_bound(img2, rot)
#     good_matches = good_sift_matches(img1,rotated)
#     print("Matches: ",len(good_matches), "    Rotation:", rot)
    



# big_photo_2.png
#     top_left: 60.506787,   22.311631
#     bottom_right:  60.501037,  22.324467

