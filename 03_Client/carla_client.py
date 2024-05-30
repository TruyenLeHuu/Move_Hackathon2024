#!/usr/bin/env python

import time

import rospy
import carla
import json
import math
from carla_msgs.msg import CarlaWorldInfo

from vehicle_control import Vehicle_Control
from ros_connect import RosConnect
from scenario_runner import Scenario_Runner


from vehicle_motion import HUD
from vehicle_motion import World
from vehicle_motion import DualControl

import pygame

class CarlaClient():
    """
    class responsable of:
        -spawning 3 vehicles of which one ego
        -interact with ROS and carla server
        -destroy the created objects
        -execute the parking manoeuvre
    """
    def __init__(self):
        """
        construct object CarlaClient with server connection and
        ros node initiation
        """
        self.is_parking = 0
        self.state = -1
        self.release_control = 0

        self.vehicle_controller = Vehicle_Control()
        self.ros_connection = RosConnect(self.vehicle_controller)
        self.scenario_runner = Scenario_Runner()

        self._steer = 0
        self._brake = 0
        self._hand_brake = 0
        self._throttle = 0
        self._reverse = 0
        self.manual_gear_shift = False
        self.gear = 0

        rospy.loginfo('Step 0 - Set up scenarios DONE')
        time.sleep(2)
    
    def manual_control_with_func(self):
        pygame.init()
        pygame.font.init()
        world = None
        try:
            client = carla.Client('127.0.0.1', 2000)
            client.set_timeout(2.0)
            world_carla = client.get_world()
            display = pygame.display.set_mode(
            # (1840, 1080),
            (1500, 800),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.vehicle_controller.create_car(world_carla)
            hud = HUD(1500, 800, self.ros_connection)
            world = World(world_carla, hud, 'vehicle.*', self.vehicle_controller.vehicle, self.ros_connection)
            controller = DualControl(self)
            clock = pygame.time.Clock()
            self.is_traffic_1 = False
            while True:
                clock.tick_busy_loop(60)
                self.ros_connection.publish_status()
                self.ros_connection.publish_obstacle_distance()
                self.ros_connection.publish_traffic_sign_info()
                self.scenario_runner.counting_stop_point(self.vehicle_controller.vehicle, hud)
                self.scenario_runner.check_speed_limited_1(self.vehicle_controller.vehicle, hud, controller, world_carla)
                self.scenario_runner.check_pedestrian_1(self.vehicle_controller.vehicle, controller, world_carla)
                hud.long_minus_score_with_condition(1)
                # self.ros_connection.keep_topic_alive()
                if (self.scenario_runner.is_vehicle_in_weather_area_1(self.vehicle_controller.vehicle)):
                    # if (world_carla.get_weather() != carla.WeatherParameters.ClearNight):
                    self.ros_connection.set_weather("ClearNight")
                    world_carla.set_weather(carla.WeatherParameters.ClearNight)
                else:
                    # if (world_carla.get_weather() != carla.WeatherParameters.Default):
                    world_carla.set_weather(carla.WeatherParameters.Default)
                    self.ros_connection.set_weather("ClearDay")

                if controller.parse_events(world, clock, self.vehicle_controller.vehicle, world_carla):
                    return
                """ This place begin for application code """
                # self.vehicle_controller.vehicle_control_with_latency_and_error(self._throttle, self._steer, self._brake, self._reverse, self._hand_brake, self.manual_gear_shift, self.gear)
                if (not self.release_control):
                    self.ros_connection.vehicle_control_with_ros(self._throttle, self._steer, self._brake, self._reverse, self._hand_brake, self.manual_gear_shift, self.gear)
                if (self.is_traffic_1 == False and self.scenario_runner.is_vehicle_in_traffic_area_1(self.vehicle_controller.vehicle) and self.ros_connection.tfl_134_status == 0):
                    hud.notification("Cross the red traffic light => score - 1")
                    hud.short_minus_score(1)
                    hud.crossRTL+=1
                    self.is_traffic_1 = True
                elif (self.is_traffic_1 == False and self.scenario_runner.is_vehicle_in_traffic_area_1(self.vehicle_controller.vehicle) and self.ros_connection.tfl_134_status == 1):
                    hud.notification("Cross the yellow traffic light")
                    self.is_traffic_1 = True
                elif (self.is_traffic_1 == False and self.scenario_runner.is_vehicle_in_traffic_area_1(self.vehicle_controller.vehicle) and self.ros_connection.tfl_134_status == 2):
                    hud.notification("Cross the green traffic light")
                    self.is_traffic_1 = True
                """ This place end for application code """

                world.tick(clock)
                world.render(display)
                pygame.display.flip()
        finally:
            if world is not None:
                world.destroy()

            pygame.quit()

    def run(self):

        """
        main loop7
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
        rospy.loginfo("CARLA world available. Spawn ego vehicle...")

        self.manual_control_with_func()

        print("press CTRL+C to terminate the node")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def destroy(self):
        self.vehicle_controller.destroy()
