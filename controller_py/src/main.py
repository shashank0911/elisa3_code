from nodes import Nodes
import numpy as np
import rospy
import json
from configuration import *
import math

from std_msgs.msg import String
# import string

if __name__ == "__main__":
    # Load data of robots 
    with open('mapper.json') as json_file:
        mapper = json.load(json_file)
    active_robots = list(mapper.keys())

    # Init phase
    print("start")   
    robots = Nodes(active_robots)
    # print(robots.camera_makers.measurement_list)
    # Set Leds
    robots.set_leds(green=0, blue=0, red=10)
    
    publisher_tag = rospy.Publisher("elisa3_tag", String, queue_size=10)
    
    # Reset the robot odom in the beginning    
    i = 0
    while(i < 3):
        print("wait for odom response")
        robots.move('still', step_size= 0.0, theta=0.)
        robots.reset('theor')
        print("reseting")
        error = 0
        for tag in robots.nodes:
            error += math.sqrt((robots.nodes[tag].odom_x - robots.nodes[tag].estimation[0])**2)
            print("est: ", [robots.nodes[tag].estimation[0], robots.nodes[tag].estimation[1]])
            
        if(error < 1e-6):
            print("reseted")
            break
        rospy.sleep(0.1)
        i += 1
        
    print("start reset")
    while(len(robots.camera_makers.measurement_list) == 0):
        rospy.sleep(0.05)
    print("awake from sleep")
    
    # Move Robots
    last_saved_time = 0
    step_size = 1.0
    theta = 0.0
    for t in range(last_saved_time, 50):
        print('\n')
        print("t: ", t)
        print('\n')
        publisher_tag.publish("run")
        robots.store_data(t)
        robots.loop_fuc('move')
        if(t%3 == 0):
            robots.reset('theor')
            
        # robots.move('still', step_size= 0.0, theta = 0.)       
        # robots.plot_data(t)
        
        # robots.test_cam()
        
        # rospy.sleep(0.01)

    robots.save_data(0)
    
    # Stop engine
    for i in range(10):
        robots.move('still', step_size= 0.0, theta=0.)
        rospy.sleep(0.05)
    
    publisher_tag.publish("stop")
    
    print('done')


