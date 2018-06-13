import os
print os.getcwd()

from Communication.Communication import FlightControl
#from Camera.CameraController import CameraController
import dronekit_sitl
import cv2



def missionForKey(key , copter):

    pitch = 0; roll = 0 ;yaw = 0; throttle = 0.5
    #pitch is going forward or backward
    if key == 0: #up
        pitch = 1
    elif key == 2: #down
      pitch = -1
    #roll is going right or left
    elif key == 1: #right
        roll = 1
    elif key == 3: #left
        roll = -1
    #yaw is turning the vehicle face
    elif chr(key) == 'a':
        yaw = -1
    elif chr(key) == 'z':
        yaw = -1

    # yaw is turning the vehicle face
    elif chr(key) == 'u':
        print "Go Up!"
        throttle = 5
    elif chr(key) == 'd':
        throttle = 0

    if chr(key) == 'g':
        print "Going for mission!"
        copter.simple_goto1()
        return


    #copter.Rotate(pitch , roll , yaw)
    copter.set_attitude(roll , pitch, yaw , throttle , 3)







def DemoMission():

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    copter = FlightControl(connection_string)
    copter.arm_and_takeoff(10)

    lena = '/Users/natouda/Desktop/xcodeTechnion/Proc_Stat_Data/MatlabIm/lena512.bmp'
    img = cv2.imread(lena, 0)

    stop = False
    copter.addObserver()
    copter.ChangeSpeed(5)

    # camera = CameraController()
    #/home/quad/Desktop/lena512.bmp

    print ("Start Demo...")
    while not stop:

        # pics = camera.getPicture()


        cv2.imshow('Realsense' , img)
        key = cv2.waitKey(1)

        if key != -1:
            print (key , ' char converssion: ', chr(key))
            missionForKey(key,copter)
            if chr(key) == 's':
                stop = True


    copter.Finish()




DemoMission()
