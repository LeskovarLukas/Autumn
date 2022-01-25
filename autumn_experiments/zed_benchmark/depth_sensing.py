########################################################################
#
# Copyright (c) 2021, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import math
import numpy as np
import sys
import cv2
import cv2.aruco as aruco

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD2K  # Use HD2K resolution 

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error")
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    #image_size.width = image_size.width / 2
    #image_size.height = image_size.height / 2

    # Capture 150 images and depth, then stop
    i = 0
    image = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m


    output = np.array([], dtype=(float, 8))
    current_frame = 0

    maxProbeCount = 500
    startChecking = False

    cameraDistance = input("Enter the distance between the camera and the Scale: ")
    cameraDistance = float(cameraDistance)

    tagDistance = input("Enter the distance between the Aruco tags: ")
    tagDistance = float(tagDistance)

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    arucoParams = cv2.aruco.DetectorParameters_create()

    key = ' '
    probeCount = 0
    while key != 113:  # for 'q' key
        points = []

        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU, image_size)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)

            # Draw window
            image_ocv = image.get_data()
            image_ocv_rgb = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2RGB)
            
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image_ocv_rgb, arucoDict, parameters=arucoParams)

            # verify *at least* one ArUco marker was detected
            if len(corners) > 1:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # draw the bounding box of the ArUCo detection
                    cv2.line(image_ocv, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image_ocv, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image_ocv, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image_ocv, bottomLeft, topLeft, (0, 255, 0), 2)
                    # compute and draw the center (x, y)-coordinates of the ArUco
                    # marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image_ocv, (cX, cY), 4, (0, 0, 255), -1)

                    points.append((cX, cY))

                    # draw marker position
                    cv2.putText(image_ocv, str(cX) + "," + str(cY), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)   
                    # draw the ArUco marker ID on the image
                    # cv2.putText(image_ocv, str(markerID),
                    #     (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    #     0.5, (0, 255, 0), 2)

                cv2.line(image_ocv, points[0], points[1], (255, 0, 0), 2)

                # Get and print distance value in mm at the center of the image
                # We measure the distance camera - object using Euclidean distance
                err, point_cloud_value1 = point_cloud.get_value(points[0][0], points[0][1])
                err, point_cloud_value2 = point_cloud.get_value(points[1][0], points[1][1])

                distance = math.sqrt((point_cloud_value1[0] - point_cloud_value2[0])**2 + (point_cloud_value1[1] - point_cloud_value2[1])**2 + (point_cloud_value1[2] - point_cloud_value2[2])**2)
                if (startChecking and probeCount < maxProbeCount):
                    probeCount += 1

                    tag1X = points[0][0]
                    tag1Y = points[0][1]
                    tag2X = points[1][0]
                    tag2Y = points[1][1]
                    output = np.append(output, [
                        [current_frame, 
                        cameraDistance, tagDistance, distance,
                        tag1X, tag1Y, tag2X, tag2Y, 
                        ]], axis=0)

                    
                    cv2.putText(image_ocv, "Recording: {:0.2f}".format(probeCount), (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    if (probeCount >= maxProbeCount):
                        startChecking = False

                cv2.putText(image_ocv, "Distance: {:.2f} m".format(distance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow("Image", image_ocv)
            if (startChecking):
               current_frame = current_frame + 1

            key = cv2.waitKey(10)
            if key == ord('r'):
                startChecking = not startChecking
                if (startChecking):
                    print("Started recording")
                else:
                    print("Stopped recording")
            

    if (output.size > 0):
        # Save the distances to a csv file
        with open('distancesZED1.csv', 'a') as csvfile:
            csvfile.write("frame, cameraDistance, tagDistance, distance, tag1X, tag1Y, tag2X, tag2Y\n")
            np.savetxt(csvfile, output, delimiter=", ", fmt="%1.3f")
    # Close the camera
    zed.close()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
