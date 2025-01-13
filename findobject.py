import cv2
import time
import lib.elegoolib as elib

## Script Variables
global objdetect,imgcap,objsdetect_list,img,robotlinedup
objsdetect_list = 0
objdetect = 0
obj_to_detect = "sports ball"  # Change to the class name of the object you wish to detect
# obj_to_detect = "bottle"
imgcap,objsdetect_list = None,None
img = None
defspeed = 150
defmovetime = 0.5
dist_min = 20       # min distance to obstacle (cm)
robotlinedup = 0    # Is the robot lined up with the object ? 0 is No, 1 is Yes, 3 is lost object as no objects detected anyore
dist_count = 0      # Numbe rof iterations of moving fwd before it checks if still lined up on object its moving towards
ang_min = 1.7       # Min distance between object and center line of the robot. Set before it thenmoves forward

robot = elib.robottank()
robot.robot_type = 'tank'
robot.mpu_type = 'MPU6050'
robot.motordriver_type = 'TB6612'
robot.ip = "192.168.4.1"
robot.port = 100
robot.connect()
robotvision = elib.robotVision()
robotvision.yolopoints_file = 'yolov8m-worldv2.pt'

def checkforobjs():
    print("Checking for object in camera")
    objdetect = 0
    print("Getting Image")
    img = robot.captureimg()
    cv2.imshow('Original Image', img)
    imgcap,objsdetect = robotvision.detectYoloObj(img,obj_to_detect)
    cv2.imshow('Object Image', imgcap)
    cv2.waitKey(1) & 0xFF == ord('0')
   
    if robotvision.objdetected:
        print(obj_to_detect + " Object detected")
        objdetect = 1
    else:
        print("No " + obj_to_detect + " object detected")
        objdetect = 0
        robot.move_left(defspeed)
        time.sleep(defmovetime)
        robot.motionstop()

    return objsdetect

def lineup_robot(detected_objects):
    isrobotlinedup = 0
    while isrobotlinedup == 0:    
    # Now fine tune the movement 
        # objdetect,objsdetect = checkforobjs()
        objlocs = robotvision.findobjectlocation(detected_objects)
        if objlocs is not None:
            for objloc in objlocs:
            # if objlocs.coun > 0:
                xc,yc,ang_deg = objloc
                print("Angle to object", round(ang_deg,1))
                if ang_deg > ang_min:
                    sleeptime = (ang_deg * robot.turntime) 
                    if xc < 320:
                        robot.move_left(defspeed)
                        time.sleep(sleeptime)
                        robot.motionstop()
                    else:
                        robot.move_right(defspeed)
                        time.sleep(sleeptime)
                        robot.motionstop()
                else:
                    isrobotlinedup = 1
        else:
            print("No objects detected")
            isrobotlinedup = 3
            break
    return isrobotlinedup
# Turn robot around in a 360 degree circle. Grab image every 10 degrees and check image to see of object is present
# if so then start driving towards it

def findobjects():
    objdet = 0
    while objdet == 0:
        objdet,objsdetect = checkforobjs()
    robotlinedup = lineup_robot(objsdetect)
    objdetect = objdet
    objsdetect_list = objsdetect

# Now we are here, that means that we have detected an object that we specified
# Now line the robot up with the object in mind (The most confident target if more then one)

# Find the objects in the camera (i.e. check for object, if none found then turn left and check again. Then repeat until object found)
findobjects()

foundobject = False
while foundobject == False:
    if robotlinedup == 1:
        distobj = robot.measure_dist()
        print("Start Distance to object ",distobj)
        while distobj > dist_min:
            print("Distance to object ",distobj)
            robot.move_fwd(defspeed)
            time.sleep(defmovetime)
            distobj = robot.measure_dist()
            # robot.motionstop()
            dist_count += 1
            print("Dist count is ",dist_count)
            if dist_count >= 10:
                dist_count = 0
                robot.motionstop()
                robotlinedup = lineup_robot()
                if robotlinedup == 3:
                    print("Lost object")
                    findobjects()
                    # lineup_robot()
                    break
        print("Robot reached object ", obj_to_detect)
        robot.motionstop()
        foundobject = True
    elif robotlinedup == 3:
        print("No objects detected to move towards")
        findobjects()
        # lineup_robot()
    else:
        print("No objects found for some reason")
        findobjects()
        # lineup_robot()

cv2.waitKey(0)
cv2.destroyAllWindows()