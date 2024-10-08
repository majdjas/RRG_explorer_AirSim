import airsim
import cv2
import os
import time
import sys
import math
import numpy as np

outputFile = "cloud.asc" 
projectionMatrix = np.array([[-0.501202762, 0.000000000, 0.000000000, 0.000000000],
                              [0.000000000, -0.501202762, 0.000000000, 0.000000000],
                              [0.000000000, 0.000000000, 10.00000000, 100.00000000],
                              [0.000000000, 0.000000000, -10.0000000, 0.000000000]])


def printUsage():
   print("Usage: python point_cloud.py [cloud.txt]")
   
def savePointCloud(depth_image, photo_image, fileName):
   f = open(fileName, "w")
   for x in range(depth_image.shape[0]):
     for y in range(depth_image.shape[1]):
        position_pt = depth_image[x,y]
        rgb_pt = photo_image[x,y]
        if (math.isinf(position_pt[0]) or math.isnan(position_pt[0])):
          # skip it
          None
        else: 
          f.write("%f %f %f %d %d %d\n" % (position_pt[0], position_pt[1], position_pt[2]-1, rgb_pt[0], rgb_pt[1], rgb_pt[2]))
   f.close()

for arg in sys.argv[1:]:
  cloud.txt = arg

client = airsim.VehicleClient()
client.confirmConnection()

while True:
    depthImage = client.simGetImage("0", airsim.ImageType.DepthPerspective)
    photoImage = client.simGetImage("0", airsim.ImageType.Scene)

    if (depthImage is None):
        print("Camera is not returning image, please check airsim for error messages")
        airsim.wait_key("Press any key to exit")
        sys.exit(0)
    else:
        png_depth = cv2.imdecode(np.frombuffer(depthImage, np.uint8) , cv2.IMREAD_UNCHANGED)
        png_photo = cv2.imdecode(np.frombuffer(photoImage, np.uint8) , cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png_depth, cv2.COLOR_BGR2GRAY)
        Image3D = cv2.reprojectImageTo3D(gray, projectionMatrix)
        savePointCloud(Image3D, png_photo, outputFile)
        print("saved " + outputFile)

        airsim.write_file(os.path.normpath('cloud_scene.png'), photoImage)

        airsim.wait_key("Press any key to exit")
        sys.exit(0)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
