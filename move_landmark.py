#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import SetModelState,SpawnModel
from gazebo_msgs.msg import LinkStates, ModelState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix, translation_matrix,quaternion_from_matrix,euler_matrix
from numpy import matrix, array

def Homogeneous(pose):
    R = matrix(quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
    t = matrix(translation_matrix([pose.position.x,pose.position.y,pose.position.z]))
    return t*R

def toPoseMsg(M):
    pose = Pose()
    pose.position.x,pose.position.y,pose.position.z = array(M[:3,3]).flatten()    
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = quaternion_from_matrix(M).flatten()
    return pose
    
class Listener:
    def __init__(self, dt):
        self.dt = dt
        self.cam_link = ''
        self.sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_callback)
        self.pose = Pose()
        self.pose.orientation.w = 1
        self.pose.orientation.y = -1
        self.pose.position.z = 0
        self.msg_ok = False  
        self.vel = [0 for i in range(6)]
        
    def link_callback(self, msg):
        self.msg_ok = True
        # try to find a link called camera or something
        for i,name in enumerate(msg.name):
            if 'camera' in name.split('::')[1]:
                # register this link as camera
                self.cam_link = name
                self.pose = msg.pose[i]
                
                # start listening to velocities
                self.sub.unregister()
                self.sub = rospy.Subscriber('joint_states', JointState, self.vel_callback)
                break 
    
    def vel_callback(self, msg):
        self.vel = [dt*v for v in msg.position]

if __name__ == '__main__':
    
    rospy.init_node('calib_bridge')
    dt = 0.1
    rate = rospy.Rate(1/dt)

    # try to get camera link to spawn landmark
    listener = Listener(dt)    
    
    count = 0
    while not rospy.is_shutdown() and not listener.msg_ok and count < 100:
        print count
        print listener.msg_ok 
        count += 1    
        rate.sleep()
        
    if listener.cam_link != '':        
        print('Found camera link at %s' % listener.cam_link)
    else:
        print('Could not find camera link, spawning landmark at (0,0,0.5)')
        
    # spawn model aligned at 0.5m in front of camera
    landmark_pose = Pose()
    landmark_pose.position.x = 0.5
    landmark_pose.orientation.x = landmark_pose.orientation.y = landmark_pose.orientation.z = landmark_pose.orientation.w = 1
    M = Homogeneous(listener.pose) * Homogeneous(landmark_pose)
    
    urdf = rospy.get_param('landmark_description')
    spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    spawner.call('landmark', urdf, 'landmark', toPoseMsg(M), '')
    
    # apply velocity to landmark
    
    pose_update = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState()
    state.model_name = 'landmark'
    
    
    while not rospy.is_shutdown():
        # homogeneous matrix from velocity
        R = matrix(euler_matrix(listener.vel[3],listener.vel[4],listener.vel[5]))
        t = matrix(translation_matrix(listener.vel[:3]))
                
        M = M * t * R
        
        # update in Gazebo
        state.pose = toPoseMsg(M)
        pose_update.call(state)
        
        rate.sleep()        
        
        #print M
        

    
        
        
        
    
    
