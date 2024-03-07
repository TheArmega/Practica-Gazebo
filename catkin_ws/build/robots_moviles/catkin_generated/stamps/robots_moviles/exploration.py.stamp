#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint
import actionlib
import math
from os import system
import sys

visitedNodes = []
aux_x = 0
aux_y = 0

def truncate_float(float_number, decimal_places):
    multiplier = 10 ** decimal_places
    return int(float_number * multiplier) / multiplier

def callback_feedback(feedback, goal_client):
    #rospy.loginfo("Feedback:%s" % str(feedback.base_position.pose.position.x))

    global aux_x, aux_y

    if truncate_float(aux_x, 4) == truncate_float(float(feedback.base_position.pose.position.x), 4) and truncate_float(aux_y, 4) == truncate_float(float(feedback.base_position.pose.position.y), 4):
        rospy.loginfo("Deteniendo el objetivo...")
        goal_client.cancel_goal()

    aux_x = float(feedback.base_position.pose.position.x)
    aux_y = float(feedback.base_position.pose.position.y)

def map_callback(map_msg):
    print('Mapa recibido')
    map_data = map_msg
    select_and_publish_goal(map_data)

def select_and_publish_goal(map_data):
    global visitedNodes, aux_x, aux_y

    if map_data is not None:
        print('Eligiendo destino...')
        width = map_data.info.width
        height = map_data.info.height

        unknown_cells = find_unknown_cells(map_data)
        
        if unknown_cells:
            random_x = unknown_cells[1]
            random_y = unknown_cells[0]

            print("Voy: ", random_x, random_y)
            print(visitedNodes)
            print(len(visitedNodes))

            print('Destino elegido. Navegando hasta el punto...')

            goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            goal_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = random_x * map_data.info.resolution + map_data.info.origin.position.x
            goal.target_pose.pose.position.y = random_y * map_data.info.resolution + map_data.info.origin.position.y
            goal.target_pose.pose.orientation.w = 1.0

            goal_client.send_goal(goal, active_cb=None, feedback_cb=lambda feedback: callback_feedback(feedback, goal_client), done_cb=None)
            wait = goal_client.wait_for_result()
            print('Punto alcanzado!')
        else:
            system("rosrun map_server map_saver -f nombreDelMapa1")
            print("MAPA MAPEADO :))))")
            exit(1)

def find_unknown_cells(map_data):
    global visitedNodes

    unknown_cells = [0, 0]
    if map_data is not None and map_data.info is not None:
        for y in range(map_data.info.height):
            for x in range(map_data.info.width):
                index = y * map_data.info.width + x
                if map_data.data[index] == 0 and (x, y) not in visitedNodes:
                    if map_data.data[(y - 2) * map_data.info.width + x] == -2 or map_data.data[(y + 2) * map_data.info.width + x] or map_data.data[y * map_data.info.width + x - 2] or map_data.data[y * map_data.info.width + x + 2]:
                        unknown_cells = [x, y]
                        visitedNodes.append((x, y))
                        if len(visitedNodes) > int(((map_data.info.width * map_data.info.height) * 0.05)/100):
                            return None
                        return unknown_cells
        return None

    return unknown_cells

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)
        
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
