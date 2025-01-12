import cv2
import time
import imutils
import lib.elegoolib as elib
from ultralytics import YOLO
import numpy as np

def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [base_colors[color_index][i] + increments[color_index][i] * 
    (cls_num // len(base_colors)) % 256 for i in range(3)]
    return tuple(color)

def detectYoloObj(frame,objtodetect):
        # Load the model
        #cv2.imwrite('robotcapture2.png', frame)
        yolo = YOLO('yolov8l-worldv2.pt')
        origheight, origwidth, channels = frame.shape
        print("Orig Width ", origwidth)
        print("Orig Height ", origheight)
        frame = imutils.resize(frame, width=640, height=480)
        listfoundobj = []
        # results = yolo.track(frame, stream=False)
        results = yolo(frame)
        cv2.line(frame, (320,0), (320,480), (0,0,255), 1)           # center line
        for result in results:
        # get the classes names
            classes_names = result.names

            # iterate over each box
            for box in result.boxes:
                # check if confidence is greater than 40 percent
                if box.conf[0] > 0.4:
                    # get coordinates
                    [x1, y1, x2, y2] = box.xyxy[0]
                    # convert to int
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    xh, yh = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # get the class
                    cls = int(box.cls[0])

                    # get the class name
                    class_name = classes_names[cls]
                    print("Found object ",class_name, " with confidence ", box.conf[0])
                    
                    # get the respective colour
                    colour = getColours(cls)
                    # draw the rectangle
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                    if classes_names[int(box.cls[0])] == objtodetect:
                        print("Found object ",objtodetect, " with confidence ", box.conf[0])
                        print("Center of object at ", xh, yh)
                        listfoundobj.append(box)
                        cv2.circle(frame, (xh, yh), 5, (0, 0, 255), -1)
                        objdetected = True
                    # get the respective colour
                    colour = getColours(cls)

                        # draw the rectangle
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                    # put the class name and confidence on the image
                    cv2.putText(frame, f'{classes_names[int(box.cls[0])]} {box.conf[0]:.2f} ({xh},{yh})' , (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
        return frame,listfoundobj

def sortconf(e):
     print("Conf in list is ", e.conf[0].item())
     return e.conf[0].item()

def findobjectlocation(yolo_result):
        objlocs=[]
        yolo_result.sort(reverse=True,key=sortconf)
        if not yolo_result:
            print("No object detected")
        else:
            for boxres in yolo_result:
                [x1, y1, x2, y2] = boxres.xyxy[0]
                # convert to int
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                xc, yc = (x1 + x2) // 2, (y1 + y2) // 2
                print("Center of ball at ", xc, yc)

                # Now get the angle of the center of the camera to the centre point of the detected object
                # pixel size of ESP32-Camera is 2.2 uM
                # focal length of lens is 3.6mm
                # So width of sensor is 2.2uM * 1600 (Max Width of image that can be taken) = 3.5mm
                # so in portion f_pix 3.6mm = 1600 px (focal length / width of sensor) * image size
                       
                # In the pinhole camera model, we assume that the optical axis of the camera (the Z-axis) is perpendicular to the image plane. If we know the pixel coordinates 
                # (𝑥,𝑦)on the image, we can use the camera intrinsic matrix to map the 2D pixel coordinates back to normalized camera coordinates (often in a "camera-centered" or "sensor-centered" coordinate system).
                
                # Normalized Camera Coordinates: From the pinhole camera matrix, we can convert the image coordinates (x,y) back to normalized coordinates in the camera frame 
                # (𝑥′,𝑦′,𝑧′) using the inverse of the intrinsic matrix.

                # In normalized camera coordinates, the 𝑧′  coordinate will be fixed to 1 (representing a point on the image plane at depth 1).

                # Angle Calculation: The angle between the optical axis (aligned along the Z-axis in the camera frame) and the point (𝑥′,𝑦′,1) can be found using the dot product 
                # between the optical axis and the vector to the point. This gives us the cosine of the angle. From there, we can compute the angle using the inverse cosine function.
                camwidth = 640
                camheight = 480
                cx = (camwidth-1) / 2
                cy = (camheight-1) / 2 
                f_pix = (3.6/3.5) * camwidth
                # This is the pinhole matrix 
                K = np.array([[f_pix, 0, cx],
                              [0, f_pix, cy],
                              [0, 0, 1]])
                
                # Convert (x, y) to normalized coordinates (x', y')
                pixel_coords = np.array([xc, yc, 1])
                normalized_coords = np.linalg.inv(K).dot(pixel_coords)
                
                # Extract the normalized coordinates
                x_prime, y_prime, _ = normalized_coords
                
                # Calculate the cosine of the angle using the formula:
                # cos(theta) = 1 / sqrt(x'^2 + y'^2 + 1)
                cos_theta = 1 / np.sqrt(x_prime**2 + y_prime**2 + 1)
                 # Calculate the cosine of the angle with respect to the x-axis
                cos_theta_x = 1 / np.sqrt(x_prime**2 + 1)
    
                # Calculate the angle in radians
                theta_rad = np.arccos(cos_theta)
                # Calculate the angle in radians
                theta_x_rad = np.arccos(cos_theta_x)
                
                # Convert the angle to degrees
                theta_deg = np.degrees(theta_rad)
                # Convert the angle to degrees
                theta_x_deg = np.degrees(theta_x_rad)
    
                # Explanation of the Code:
                # 
                # It converts the pixel coordinates to normalized camera coordinates by multiplying the inverse of the camera matrix K−1

                #Normalized Coordinates: Once the pixel coordinates are converted to normalized coordinates 
                #(𝑥′,𝑦′) we compute the cosine of the angle between the optical axis (along the Z-axis) and the point in the image.

                #Angle Calculation: Using the inverse cosine (np.arccos), the angle 𝜃
                #between the camera's optical axis and the given point is calculated in radians and then converted to degrees.
                # print(f"The angle between the center axis and the point at ({xc}, {yc}) is {theta_deg:.2f} degrees.")
                # print(f"The x axis angle between the center axis and the point at ({xc}, {yc}) is {theta_x_deg:.2f} degrees.")
                objlocs.append([xc,yc,theta_x_deg])

        return objlocs



objdetect = 0
obj_to_detect = "sports ball"
imgcap,objsdetect = None,None
img = None
defspeed = 100
defmovetime = 0.75
dist_min = 20       # min distance to obstacle (cm)
d180 = 90           # eq rotation distance for 180 deg turn
dturn = 60          # eq rotation distance for smaller than 180 deg turns
img = cv2.imread('robotcapture.png')
# 1 sec is 53 degrees
turntime = (1/53)
print("Starting AI")

imgcap,objsdetect = detectYoloObj(img,obj_to_detect)
objlocs = findobjectlocation(objsdetect)
if objlocs is not None:
        xc,yc,ang_deg = objlocs[0]
        print("Angle to object", round(ang_deg))
        steer_ang = ang_deg
        sleeptime = ang_deg * turntime
        #sleeptime = (dturn/defspeed*abs(steer_ang)/180) * 2.5
        print("sleep time is ",str(round(sleeptime,2)))

cv2.imshow('Orig',img)
cv2.imshow('AI',imgcap)
cv2.waitKey(0)
cv2.destroyAllWindows()
