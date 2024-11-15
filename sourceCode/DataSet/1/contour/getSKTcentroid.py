# env : conda activate wearMask

from cmath import sqrt
import cv2
import numpy as np
import math
import time
import sys
from skimage.morphology import skeletonize
from shapely.geometry import LineString, Point, LinearRing, Polygon

image = cv2.imread('003.png', cv2.IMREAD_UNCHANGED)
image = cv2.copyMakeBorder(image, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=[255,255,255, 0])
image = cv2.resize(image, (500, 500))

# binarize
row, col, dim = image.shape
for i in range(row):
    for j in range(col):
        if image[i,j,3] == 0:
            image[i,j] = 0
        else:
            image[i,j] = 255

binary_image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
cv2.imshow("bi_image", binary_image)
cv2.waitKey(0)

boundarys, _ = cv2.findContours(binary_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
# im = cv2.drawContours(image, boundarys, 1, (255, 0, 0), 5)
# cv2.imshow("contour", im)
# cv2.waitKey(0)

# if boundarys.size > 1, remove this and check the max boundary.
# try:
#     if len(boundarys) != 1:
#         raise 
# except:
#     print("boundary error !")
#     sys.exit(0)

boundary = boundarys[0]
boundary = np.array(boundary)

boundary_coords = [(p[0][0], p[0][1]) for p in boundary]
boundary_coords.append(boundary_coords[0])

boundary_line = LinearRing(boundary_coords)


skeleton = skeletonize(binary_image)
# get coordinate of skeleton
skeleton_coords = []
for i in range(skeleton.shape[0]):
    for j in range(skeleton.shape[1]):
        if skeleton[i, j]:
            skeleton_coords.append((j, i))
            cv2.circle(binary_image, (j, i), 2, (0, 0, 0), -1)

cv2.imshow("skt", binary_image)
cv2.waitKey(0)

maxLen = 0
maxPos = skeleton_coords[0]
dic = {}
for pt_coord in skeleton_coords:
    # distance to polygon.
    distances = [abs(cv2.pointPolygonTest(boundary_p, pt_coord, True)) for boundary_p in boundary]
    
    dist_index = np.argsort(distances)
    
    one = dist_index[0]
    two = 0
    p_one = boundary_coords[one]
    p_two = p_one
    thres = distances[one]
    # find the second point. If didn't do it, the one two points will too close
    for id in dist_index:
        p = boundary_coords[id]
        if ((p_one[0] - p[0])**2 + (p_one[1] - p[1])**2)**0.5 > thres:
            p_two = p
            two = id
            break
    
    length = 0
    if one < two:
        d1 = two - one
        d2 = one + len(boundary_coords) - two
        length = min(d1, d2)
    else:
        d1 = one - two
        d2 = two + len(boundary_coords) - one
        length = min(d1, d2)

    dic[pt_coord] = length

    # save the max-scared point
    if length > maxLen:
        maxLen = length
        maxPos = pt_coord


poly = Polygon(boundary_coords)
centroid = poly.centroid

    

print("Pos : ", maxPos)

cv2.circle(image, (int(centroid.x), int(centroid.y)), 2, (0, 0, 255), -1)
cv2.circle(image, maxPos, 2, (0, 255, 0), -1)
cv2.imshow("img", image)
cv2.waitKey(0)

output_file_path = './skeleton.txt'
with open(output_file_path, 'w') as f:
    for pt_coord in skeleton_coords:
         f.write(str(pt_coord[0]) + ' ' + str(pt_coord[1]) + ' ' + str(dic[pt_coord]) + '\n')