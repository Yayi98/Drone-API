#!/usr/bin/env python
import roslib
import rospy
import time
from ctypes import *
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Char,Int16
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import sys, select, termios, tty
import numpy as np 
import matplotlib.pyplot as plt
import thread
import multiprocessing

#graphing variables
xaxis = [0] * 160000
thry  = [0] * 160000
erry  = [0] * 160000
intiy = [0] * 160000
diffy = [0] * 160000


thrx  = [0] * 160000
errx  = [0] * 160000
intix = [0] * 160000
diffx = [0] * 160000

thrz  = [0] * 160000
errz  = [0] * 160000
intiz = [0] * 160000
diffz = [0] * 160000

#global variables intialised
e_prevx  = 0
ui_prevx = 0
e_prevy  = 0
ui_prevy = 0
ui_prevz  = 0
e_prevz  = 0
currx  = 0
curry  = 0
currz  = 0
currax = 0
curray = 0
curraz = 0
currx1  = 0
curry1  = 0
currz1  = 0
count  = 4


#input required functions
def fmtFloat(n):
    return '{:6.3f}'.format(n)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#pid controllers for x, y, z

def pid_controllery(ya, yc, h=0.1, Ti=1.78, Td=61.11, Kp=13.5, u0=0, e0=0, ki=1):                                                   #PID CONTROLLER  kp 11
	

	
	global count
        global thry 
        global erry 
        global intiy 
        global diffy 
	global e_prevy
        global ui_prevy
	k = 0
        
        while 1:
                e = yc - ya
                ui = ui_prevy + 1/Ti * h*e
		ud = Td * (e - erry[count-4])/(4*h)
                e_prevy = e
		ui_prevy = ui
                u = (Kp *e) + ki*ui + ud
                 
                print "errorY",round(e,2)," integerator ",round(ki*ui,2)," deffren ",round(ud,2)
		k += 1
                
                erry[count]=e
                intiy[count]=ki*ui
                diffy[count]=ud
                return u

def pid_controllerx(ya, yc, h=0.1, Ti=1.78, Td=60.88, Kp=14, u0=0, e0=0, ki=1):                                                           #PID CONTROLLER kp 11
	
	
	global e_prevx
        global ui_prevx 
        global thrx 
        global errx 
        global intix 
        global diffx 
	k = 0
        
        while 1:
                e = yc - ya
                ui = ui_prevx + 1/Ti * h*e
		ud = Td * (e - errx[count-4])/(4*h)
                e_prevx = e
		ui_prevx = ui
                u = (Kp *e) + ki*ui + ud
                
		print "errorX",round(e,2)," integerator ",round(ki*ui,2)," deffren ",round(ud,2)
		k += 1
                errx[count]=e
                intix[count]=ki*ui
                diffx[count]=ud
                return u

def pid_controllerz(ya, yc, h=0.1, Ti=1.21, Td=29, Kp=55, u0=0, e0=0, ki=1):                                                          #PID CONTROLLER
	
        
	global thrz 
        global errz 
        global intiz 
        global diffz 
	global e_prevz
        global ui_prevz
	k = 0
        
        while 1:
                e = yc - ya
                ui = ui_prevz + 1/Ti * h*e
		ud = Td * (e - errz[count-6])/(6*h)
                e_prevz = e
		ui_prevz = ui
                u = (Kp*e) + ki*ui + ud
                
		print "errorZ",round(e,2)," integerator ",round(ki*ui,2)," deffren ",round(ud,2)
		k += 1
                errz[count]=e
                intiz[count]=ki*ui
                diffz[count]=ud
                return u 



# publisher for /drone_command
class send_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_server')
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		self.currax = PlutoMsg().rcRoll
                self.curray = PlutoMsg().rcPitch
                self.curraz = PlutoMsg().rcYaw 
		self.cmd = PlutoMsg()
		self.cmd.rcRoll     =1500
		self.cmd.rcPitch    =1500
		self.cmd.rcYaw      =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1     =1500
		self.cmd.rcAUX2     =1500
		self.cmd.rcAUX3     =1500
		self.cmd.rcAUX4     =1000
		
	def arm(self):
		self.cmd.rcRoll     =1500
		self.cmd.rcYaw      =1500
		self.cmd.rcPitch    =1500
		self.cmd.rcThrottle =1000
                self.cmd.rcAUX1     =1500
		self.cmd.rcAUX2     =1500
		self.cmd.rcAUX3     =1500
		self.cmd.rcAUX4     =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)
	def disarm(self):
                self.cmd.rcRoll     =1500
		self.cmd.rcYaw      =1500
		self.cmd.rcPitch    =1500
                self.cmd.rcAUX1     =1500
		self.cmd.rcAUX2     =1500
		self.cmd.rcAUX3     =1500
                self.cmd.rcThrottle =1000
		self.cmd.rcAUX4     =1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.5)
	
	
	def reset(self):
		self.cmd.rcRoll     =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcPitch    =1500
		self.cmd.rcYaw      =1500
		self.command_pub.publish(self.cmd)
	

	def move(self,currax,curray,curraz):
		        self.cmd.rcRoll     =currax
			self.cmd.rcPitch    =curray
		        self.cmd.rcThrottle =curraz
                        self.cmd.rcYaw      =1500
                        self.cmd.rcAUX1     =1500
		        self.cmd.rcAUX2     =1500
		        self.cmd.rcAUX3     =1500
		        self.cmd.rcAUX4     =1500
			self.command_pub.publish(self.cmd)             
                        rospy.sleep(0.028)          
                         
       
#callback function for whycon
def callback(data):
    global xaxis
    global currx
    global curry 
    global currz 
    global currax 
    global curray 
    global curraz   
    global currx1
    global curry1
    global currz1
    
    
    #tempz = 23.243310-(0.0902551*int((currx+20)*10))-(0.1130619*int((curry+20)*10))+(0.00024389254*int((currx+20)*10)*int((currx+20)*10))+(0.000311211*int((curry+20)*10)*int((curry+20)*10))-(0.000027951*int((currx+20)*10)*int((curry+20)*10))+data.poses[0].position.z
    
    
    #tempz1 = 23.243310-(0.0902551*int((currx1+20)*10))-(0.1130619*int((curry1+20)*10))+(0.00024389254*int((currx1+20)*10)*int((currx1+20)*10))+(0.000311211*int((curry1+20)*10)*int((curry1+20)*10))-(0.000027951*int((currx1+20)*10)*int((curry1+20)*10))+data.poses[1].position.z
     
    tempz = 15.47919754423-(0.04569349161*int((currx+20)*10))-(0.0310939534*int((curry+20)*10))+(0.0000794588956*int((currx+20)*10)*int((currx+20)*10))-(0.0000890845797*int((curry+20)*10)*int((curry+20)*10))+(0.000000125473*int((currx+20)*10)*int((currx+20)*10)*int((currx+20)*10))+(0.0000007430721*int((curry+20)*10)*int((curry+20)*10)*int((curry+20)*10))-(0.000108668831*int((curry+20)*10)*int((currx+20)*10))+(0.000000376315*int((currx+20)*10)*int((currx+20)*10)*int((curry+20)*10))-(0.00000020113742*int((currx+20)*10)*int((curry+20)*10)*int((curry+20)*10))+data.poses[0].position.z
    
    #tempz1 = 15.47919754423-(0.04569349161*int((currx1+20)*10))-(0.0310939534*int((curry1+20)*10))+(0.0000794588956*int((currx1+20)*10)*int((currx1+20)*10))-(0.0000890845797*int((curry1+20)*10)*int((curry1+20)*10))+(0.000000125473*int((currx1+20)*10)*int((currx1+20)*10)*int((currx1+20)*10))+(0.0000007430721*int((curry1+20)*10)*int((curry1+20)*10)*int((curry1+20)*10))-(0.000108668831*int((curry1+20)*10)*int((currx1+20)*10))+(0.000000376315*int((currx1+20)*10)*int((currx1+20)*10)*int((curry1+20)*10))-(0.00000020113742*int((currx1+20)*10)*int((curry1+20)*10)*int((curry1+20)*10))+data.poses[1].position.z
    

    '''if tempz1<24:
        currx = data.poses[1].position.x
        curry = data.poses[1].position.y
        currz = tempz1
        currx1 = data.poses[0].position.x
        curry1 = data.poses[0].position.y
        currz1 = tempz
    else:
        if tempz1<tempz:
           currx = data.poses[1].position.x
           curry = data.poses[1].position.y
           currz = tempz1
           currx1 = data.poses[0].position.x
           curry1 = data.poses[0].position.y
           currz1 = tempz
        else:
           currx = data.poses[0].position.x
           curry = data.poses[0].position.y
           currx1 = data.poses[1].position.x
           curry1 = data.poses[1].position.y
           currz = tempz
           currz1 = tempz1 '''

    currx = data.poses[0].position.x
    curry = data.poses[0].position.y
    currz = tempz 
    #currz = data.poses[0].position.z
       
    currx = round(currx,2)                
    curry = round(curry,2)
    currz = round(currz,2)
    currx1 = round(currx1,2)                
    curry1 = round(curry1,2)
    currz1 = round(currz1,2)



def mydrone(threadName, delay):
        print "Thread started"
        global count
        count =4  
        land=False
        start=False
        
        
        
        
        while(1): 
                 count=count+1
                 if q.full():
                   droneD=q.get()
                   start=True
                   if droneD['e']:
                     land=False
                     print "Disarmed"   
                     drone.disarm()
                     print "                       "
                     print "Disarmed"
                     rospy.sleep(0.5)
                     drone.disarm()
                     droneD['e']=False
                     start=False
                   elif droneD['t']:
                       land=False
                       print "Drone in TAKING OFF & HOLD"
                       drone.disarm()
        	       rospy.sleep(1)
                       print "Arming"
                       drone.arm()
                       rospy.sleep(1)
                       count=9
                       droneD['t']=False
                       xcor=currx
                       ycor=curry
                       zcor=23
                       
                   elif droneD['g']:
                        land=False
                        print "Drone in GOTO state"
                        count=501
                        droneD['g']=False
                        xcor=droneD['xcod']
                        ycor=droneD['ycod']
                        zcor=droneD['zcod']
                   elif droneD['l']:
                          print "Drone going to LAND"
                          land=True 
                          droneD['l']=False
               
                 
                 
                 if start:
                    if count<15:
                       uy=pid_controllerx(currx,xcor)
                       ux=pid_controllery(curry,ycor)
                       uz=pid_controllerz(currz,zcor)
                       ay = 1500+(-ux)
                       ax = 1500+(uy)
                       az = 1500+(-uz)
                       print "Warming up Controller and rejecting false initial values"
                       
                       print "value X",int(ax),"coordinates",currx
                       print "value Y",int(ay),"coordinates",curry
                       print "value Z",int(az),"coordinates",currz
                       print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
                       
                       
                    
                    else:
                            if land:
                               zcor=zcor-0.01
 
                             
                            print "-----------------------------------------------------------------------------"
                            '''if (currx<-9.00 or currx>11.00 or curry<-9.50 or curry>7.00): 
                            print "ERROR Whycon too far to be true, EXITING"
                            rospy.sleep(0.2)
                            if count > 1550 :
                               drone.disarm()
                               print "Disarmed, Limit reached in ERROR"
                               break 
                            else:
                                '''
                               
                            print "LETS GO TO X ",xcor,"Y ",ycor,"Z ",zcor  
                            uy=pid_controllerx(currx,xcor)
                            ux=pid_controllery(curry,ycor)
                            uz=pid_controllerz(currz,zcor) 
                            if (count>500) and (count <506):
                                
                                uy=uy-diffx[count]
                                ux=ux-diffy[count]
                                print "----------------------I M ACTIVE-------------------NEGATING:",diffx[count]," ",uy
                    
                            ay = 1500+(-ux)
                            ax = 1500+(uy)
                            az = 1500+(-uz)
                            
                            
                            print "value X",int(ax),"coordinates",currx
                            print "value Y",int(ay),"coordinates",curry
                            print "value Z",int(az),"coordinates",currz
                            
                            drone.move(int(ax),int(ay),int(az))          
                            thrx[count]=uy
                            thry[count]=-ux
                            thrz[count]=-uz
                            print count
                            '''           #Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary     
                            key=getKey() 
                            if key == " ":
                               print "Please input Kp then kd"
                               zkp = input()
                               ztd = input()
                               print "updated with ",zkp," ",ztd 
                            #Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary Temporary  '''
                                        
                 
                            #except KeyboardInterrupt:
        plt.figure(1)
        plt.plot(thry, color="black")
        plt.plot(erry, color="red")
        plt.plot(intiy, color="green")
        plt.plot(diffy, color="blue")
        plt.plot(xaxis, color="purple")
        plt.show()
        
        plt.figure(2)
        plt.plot(thrx, color="black")
        plt.plot(errx, color="red")
        plt.plot(intix, color="green")
        plt.plot(diffx, color="blue")
        plt.plot(xaxis, color="purple")
        plt.show()

        plt.figure(3)
        plt.plot(thrz, color="black")
        plt.plot(errz, color="red")
        plt.plot(intiz, color="green")
        plt.plot(diffz, color="blue") 
        plt.plot(xaxis, color="purple")
        plt.show() 



if __name__=="__main__": 
        settings = termios.tcgetattr(sys.stdin)
        drone = send_data(); 
        cord=rospy.Subscriber('/whycon/poses', PoseArray, callback)                 #Passing Topic name and msg type
        q=multiprocessing.Queue(maxsize=1)       
        mainD={'t':False,'g':False,'h':False,'l':False,'e':False,'xcod':0.00,'ycod':0.00,'zcod':28.00}
        thread.start_new_thread( mydrone,("Thread-1", 2, ))          
        mainD['g']=True    #remove
        q.put(mainD,True,0.1)
        rospy.sleep(100) 
         
                
                
        
                
                
        

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
























