import pygame
import numpy as np
import csv

import config
from fire import Fire
import vehicle
from vehicle import Vehicle
from planners.hotspot_planner import HotspotPlanner
from planners.base_planner import BasePlanner
from flame import config

import rospy
from mavros.msg import *
from mavros.srv import *
import time
home_offset = 0
home_offset_flag = 0

def waypoints_cb(data):
    global home_offset, home_offset_flag
    print data
    if home_offset_flag == 0:
        home_offset = Waypoint(frame=Waypoint.FRAME_GLOBAL_REL_ALT,
                            command=Waypoint.NAV_WAYPOINT,
                            is_current=True,
                            x_lat=44.5654, y_long= -123.2725, z_alt=0) #data.waypoints[0]
    rospy.loginfo("PULLED GPS LOCATION:  %s", data.waypoints)

def handle_waypoints(data):
    rospy.loginfo("Got waypoints: %s", data)

def run(fires_toa, fires_fli, trial_directory):
    #Start GPS Node

    gps_file = open('gps.csv', 'wb')
    a = csv.writer(gps_file)
    header = [['latitude', 'longitude']]
    a.writerows(header)

    global home_offset, home_offset_flag
    if config.USING_ROS:
        rospy.init_node('waypoint')
    #rospy.Subscriber('/mavros/mission/waypoints', WaypointList, handle_waypoints)
    pygame.init()
    pygame.display.set_caption("Flame: Fire Simulator")

    fire = Fire(fires_toa, fires_fli)
    planner = HotspotPlanner(fire, trial_directory)

    entities = [
        fire,
        Vehicle(planner)
    ]

    screen = pygame.display.set_mode(fire.shape(), pygame.HWSURFACE | pygame.DOUBLEBUF)
    clock = pygame.time.Clock()
    print planner.is_done()

    # This makes sure the simulation begins at the start of the fire
    simulation_time = np.asarray(fire.time_of_arrival)
    simulation_time = np.sort(simulation_time[simulation_time >= 0])[0]+3
    MAX_SIM_TIME = config.MAX_SIM_TIME + simulation_time

    #Waypoint set up
    old_time = 0.0
    time_difference = 0.0
    if config.USING_ROS:
        get_gps()
    #rospy.loginfo("Got home: %s", home_offset)
    test_scale_multiplier =10
    scaling_factor_x = .078534/364576.4709016314  *test_scale_multiplier  #(feet/pixel)/(feet/degree)
    scaling_factor_y = .078534/260659.33267246236 *test_scale_multiplier #(feet/pixel)/(feet/degree)

    map_offset_x = home_offset.y_long - config.STARTING_LOCATION[0]*scaling_factor_x
    map_offset_y = home_offset.x_lat + config.STARTING_LOCATION[1]*scaling_factor_y


    while simulation_time < MAX_SIM_TIME and planner.is_done() is False:
        # clock.tick(100)

        loop_start_time = time.time()

        for entity in entities:
            entity.update(simulation_time)
            entity.draw(screen)

        # Draw frontier on alpha surface and blit it to the screen
        frontier_surf = pygame.Surface(fire.shape(), pygame.SRCALPHA)
        frontier = np.asarray(fire.frontier)
        for (x, y) in frontier.T:
            frontier_surf.set_at((x, y), (0, 255, 0, 20))
        screen.blit(frontier_surf, (0, 0))

        # Draw clusters on alpha surface and blit it to the screen
        cluster_surf = pygame.Surface(fire.shape(), pygame.SRCALPHA)
        clusters = fire.clusters.astype(np.uint)
        if clusters.any():
            for (x, y) in clusters:
                pygame.draw.circle(cluster_surf, (255, 0, 0, 128), (x, y), 6)
        screen.blit(cluster_surf, (0, 0))
        if len(vehicle.vis_plan) >0:
            vis_plan_surf = pygame.Surface(fire.shape(), pygame.SRCALPHA)
            vis_plan = np.array(vehicle.vis_plan)

            cur_x, cur_y = entities[1].location[0], entities[1].location[1]
            for (dx, dy) in vis_plan:
                cur_x += dx
                cur_y += dy

                vis_plan_surf.set_at((int(cur_x), int(cur_y)), (255, 0, 0, 255))
            screen.blit(vis_plan_surf, (0, 0))

        time_since = time.time() - old_time
        print "time_since: ", time_since
        if (time_since > config.UPDATE_RATE) & config.USING_ROS:

            #conversion to GPS
            print "FIRE SHAPE:"
            print fire.shape()
            print "UAV LOCATION:"
            print entities[1].location
            long_cor = entities[1].location[0]*scaling_factor_x + map_offset_x
            lat_cor = -entities[1].location[1]*scaling_factor_y + map_offset_y 

            # b = open('gps.csv', 'ab')
            #a = csv.writer(b)
            #data = [[lat_cor, long_cor]]
            #a.writerows(data)
            #Send Waypoint
            #rospy.loginfo("waiting for MAVROS PUSH service")

            rospy.wait_for_service('/mavros/set_mode')
            mode = 220 #auto mode = 220. Guided_Mode = 216
            #mode = MAV_MODE(mode)
            mode_set = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            resp = mode_set(0,'3')
            rospy.loginfo("SETTING TO AUTO: %s", resp)

            rospy.wait_for_service('/mavros/mission/push')
            waypoints = [home_offset, Waypoint(frame=Waypoint.FRAME_GLOBAL_REL_ALT,
                            command=Waypoint.NAV_WAYPOINT,
                            is_current=True,autocontinue=True,
                            x_lat=lat_cor, y_long=long_cor, z_alt=6)
                        ]
            #waypoint = Waypoint(frame=Waypoint.FRAME_GLOBAL_REL_ALT,
            #                command=Waypoint.NAV_WAYPOINT,
            #                is_current=True,
            #                x_lat=lat_cor, y_long= long_cor, z_alt=6)

            waypoint_goto = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            resp = waypoint_goto(waypoints)
            #rospy.loginfo(resp)
            #rospy.loginfo("Sent Waypoints: %s", resp)
            old_time = time.time()


        #Advance one timestep
        simulation_time += config.TIME_STEP

        #Wait for the deadline
        loop_time = time.time() - loop_start_time
        while loop_time <= config.DEADLINE:
            loop_time = time.time() - loop_start_time
        pygame.display.flip()


def gps_init():
    rospy.Subscriber('/mavros/mission/waypoints', WaypointList, waypoints_cb)
    #rospy.loginfo("waiting for MAVROS PULL service")
    rospy.wait_for_service('/mavros/mission/pull')
    get_gps()


def get_gps():
    #Call the WaypointPull service
    #rospy.loginfo("CALLING WAYPOINTPULL SERVICE")
    waypoint_pull = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
    resp = waypoint_pull()
    #rospy.loginfo(resp)

