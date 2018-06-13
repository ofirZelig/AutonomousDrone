from Communication.Communication import FlightControl
from Camera.CameraController import CameraController
import dronekit_sitl
import cv2



def missionForKey(key , copter):

    pitch = 0; roll = 0 ;yaw = 0; throttle = 0.5
    #pitch is going forward or backward
    if chr(key) == 'w': #up
        pitch = 1
    elif chr(key) == 'x': #down
      pitch = -1
    #roll is going right or left
    elif chr(key) == 'd': #right
        roll = 1
    elif chr(key) == 'a': #left
        roll = -1
    #yaw is turning the vehicle face
    elif chr(key) == 'q':
        yaw = -1
    elif chr(key) == 'e':
        yaw = -1

    # yaw is turning the vehicle face
    elif chr(key) == 'u':
        print "Go Up!"
        throttle = 1
    elif chr(key) == 'j':
        throttle = 0.3

    elif chr(key) == 'g':
        print "Going for mission!"
        copter.simple_goto1()
        return


    #copter.Rotate(pitch , roll , yaw)
    copter.set_attitude(roll , pitch, yaw , throttle , 3)






def DemoMission():

    sitl = dronekit_sitl.start_default()
#    connection_string = '/dev/ttyACM0'
    connection_string = sitl.connection_string()
    copter = FlightControl(connection_string)
    copter.arm_and_takeoff(10)

    #lena = '/Users/natouda/Desktop/xcodeTechnion/Proc_Stat_Data/MatlabIm/lena512.bmp'
    #img = cv2.imread(lena, 0)

    stop = False
    #copter.addObserver()
    copter.ChangeSpeed(5)

    camera = CameraController()

    print ("Start Demo...")
    while not stop:

        pics = camera.getPicture()


        cv2.imshow('Realsense' , pics)
        key = cv2.waitKey(1)

        if key != -1:
            print (key , ' char converssion: ', chr(key))
            missionForKey(key,copter)
            if chr(key) == 's':
                stop = True

    copter.Finish()




DemoMission()
