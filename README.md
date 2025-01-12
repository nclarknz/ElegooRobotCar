# ElegooRobotCarV4

Install the requirements in Python scripts folder by running pip install -r requirements.txt in the folder

The library is in teh lib folder. This allows you to control with the tank or teh car (Need to setup the type when you init the library)

Init like this

robot = elib.robottank()
robot.robot_type = 'tank'
robot.mpu_type = 'MPU6050'
robot.motordriver_type = 'TB6612'
robot.ip = "192.168.4.1"
robot.port = 100
robot.connect()

Can then call a robot vision class that can run object detection. This can use different trained models. 


The findobject.py when run will find an object class (That you specify within the script) and the robot will find it and move to it


