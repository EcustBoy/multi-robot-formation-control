import numpy as np
import math
import time
import rospy
import tf
import roslaunch
import rospkg

control_space=np.ones([1,2])
action=list()
t=0

def execute(action):
        #action:[v,w]-linear/angular velocity
        #rospy.wait_for_service('/gazebo/unpause_physics')
        #try:
         #   self.unpause
        #except rospy.ServiceException:
         #   print("/gazebo/unpause_physics service call failed")
        rospy.init_node('env_node')
        vel_pub_1 = rospy.Publisher('/mbot_1/cmd_vel', Twist, queue_size=5)
        vel_pub_2 = rospy.Publisher('/mbot_2/cmd_vel', Twist, queue_size=5)
        vel_cmd = Twist()
        vel_cmd.linear.x=action['linear_vel']
        vel_cmd.angular.z=action['angular_vel']
        #vel=[vel_cmd.linear.x,vel_cmd.angular.z]
        #print(vel)
        vel_pub_1.publish(vel_cmd)
        vel_pub_2.publish(vel_cmd)
        time.sleep(0.05)
        
while True:
    t=t+1
    #compute consensus-based control input
    
        #control_space[i,:]=np.array([np.sin(state_space[i,0]),np.sin(state_space[i,2])])
    control_space=np.array([np.sin(t),np.cos(t)])
    action.append({'linear_vel':control_space[0],'angular_vel':control_space[1]})
    execute(action)        