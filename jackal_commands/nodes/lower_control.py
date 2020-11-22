#!/usr/bin/env python
# license removed for brevity
import rospy
from jackal_commands.msg import tupla
from geometry_msgs.msg import Twist

import threading
#working height and weight
w_depth=320
h_depth=240

#max angular velocity:
max_Ang_Vel=0.5   #ang and linear are different! check it out! check also the max values
max_Lin_Vel=0.6

class Initialization():
  def __init__(self):
      rospy.init_node('jackal_vineyard_node')
      
      global f #flag of the controller
      global dc#data_controller 
      f=0
      dc=0
      #rate = rospy.Rate(10) # 10hz


class Reading(threading.Thread):

  def __init__(self):
    threading.Thread.__init__(self)
    print 'init'
    
  def run(self):
    self.subControl=rospy.Subscriber('/control_value', tupla, self.LowLevController)
    rospy.spin()


  def LowLevController(self,data): 
    global f, dc
    f=data.flag
    dc=data.control
    #print f, dc


class Writing():

  def __init__(self):
    self.pub_cmd = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    self.velocity = Twist()

  def Move(self,linear_x, angular_theta):
    '''
    Publish the components of velocity
    :param linear_x: 
    :param angular_theta:
    :return: none
    '''
    self.velocity.linear.x=linear_x
    self.velocity.angular.z=angular_theta
    self.pub_cmd.publish(self.velocity)



  #function to set the controller
  def controllerDepth(self,delta):
    #parabolic control 
    if delta > 0:                  #  LEFT SIDE
      ang_vel_command = float( - max_Ang_Vel*delta*delta)/((w_depth/2)*(w_depth/2))

    else:                       # RIGTH SIDE
      ang_vel_command = float(max_Ang_Vel*delta*delta)/((w_depth/2)*(w_depth/2))


    #lin_vel_command = float(max_Lin_Vel*delta*delta)/((w_depth/2)*(w_depth/2))WRONG FUNCTION!!

    #print("ang_vel_command",ang_vel_command)
    #return (((max_Ang_Vel-0.2)-abs(ang_vel_command)),ang_vel_command)

    #return (lin_vel_command,ang_vel_command)
    return (0,ang_vel_command) #prova fatta al PIC




if __name__ == '__main__':

  init = Initialization()

  IN2 = Reading()

  IN2.start()

  OUT = Writing()


  while not rospy.is_shutdown():
    try:
        #f=1
        if not f:  #depth is sending msg
          (cmd_lin2, cmd_ang2) = OUT.controllerDepth(dc)  # controller based on the center of the longitudinal (x-axis)
          OUT.Move(cmd_lin2, cmd_ang2)  #give the cmd
        else:  #ML is sending msg
          if dc == 0:     #turn right
            OUT.Move(0,-0.1)
          elif dc == 1:   #center
            OUT.Move(0, 0) 
          elif dc == 2:    #turn left
            OUT.Move(0,0.1)
          else:
            OUT.Move(0,0)




    except rospy.ROSInterruptException:
        pass


