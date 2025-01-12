import cv2
import time
import lib.elegoolib as elib

## Script Variables
objdetect = 0
obj_to_detect = "sports ball"
imgcap,objsdetect = None,None
img = None
defspeed = 100
defmovetime = 0.75
dist_min = 20       # min distance to obstacle (cm)
d180 = 90           # eq rotation distance for 180 deg turn
dturn = 60          # eq rotation distance for smaller than 180 deg turns
robotlinedup = 0

robot = elib.robottank()
robot.robot_type = 'tank'
robot.mpu_type = 'MPU6050'
robot.motordriver_type = 'TB6612'
robot.ip = "192.168.4.1"
robot.port = 100
robot.connect()
robotvision = elib.robotVision(robot)

def checkforobjs():
    print("Checking for object in camera")
    objdet = 0
    print("Getting Image")
    img = robot.captureimg()
    cv2.imshow('Original Image', img)
    imgcap,objsdetect = robotvision.detectYoloObj(img,obj_to_detect)
    cv2.imshow('Object Image', imgcap)
    cv2.waitKey(1) & 0xFF == ord('0')
   
    if robotvision.objdetected:
        print(obj_to_detect + " Object detected")
        objdet = 1
    else:
        print("No " + obj_to_detect + " object detected")
        objdet = 0
        robot.move_left(defspeed)
        time.sleep(defmovetime)
        robot.motionstop()

    return objdet,objsdetect

# Turn robot around in a 360 degree circle. Grab image every 10 degrees and check image to see of object is present
# if so then start driving towards it
while objdetect == 0:
    objdet,objsdetect = checkforobjs()
    objdetect = objdet
    # img = imgcap

# Now we are here, that means that we have detected an object that we specified
# Now line the robot up with the object in mind (The most confident target if more then one)
while robotlinedup == 0:    
    # Now fine tune the movement 
    objdetect,objsdetect = checkforobjs()
    objlocs = robotvision.findobjectlocation(objsdetect)
    if objlocs is not None:
    # print("Object Location", objlocs)
        # for objloc in objlocs:
        if objlocs.count == 0:
            xc,yc,ang_deg = objlocs[0]
            print("Angle to object", round(ang_deg,1))
            if ang_deg > 2:
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
                robotlinedup = 1

distobj = robot.measure_dist()
print("Start Distance to object ",distobj)
while distobj > dist_min:
    print("Distance to object ",distobj)
    robot.move_fwd(defspeed)
    time.sleep(defmovetime/2)
    distobj = robot.measure_dist()
robot.motionstop()

cv2.waitKey(0)
# cv2.waitKey(1) & 0xFF == ord('0')
cv2.destroyAllWindows()