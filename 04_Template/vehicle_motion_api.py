

"""
Class for carla
"""

import rospy
import time
from std_msgs.msg import Header, String, Int32
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, NavSatFix, Image, PointCloud2, Imu
from geometry_msgs.msg import Quaternion, Vector3, Pose
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from carla_msgs.msg import (CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaWorldInfo,CarlaTrafficSignList, CarlaEgoVehicleObstacle,
                            CarlaActorList, CarlaTrafficLightStatusList,CarlaEgoVehicleDoorStatus, CarlaTrafficLightList,
                            CarlaTrafficLightInfoList, CarlaEgoVehicleControl)
from std_msgs.msg import Bool  # pylint: disable=import-error

TIMEOUT = 20

class General():
    def __init__(self):
        self.clock_msg = None
        self.vehicle_velocity = 0.0
        self.vehicle_acceleration = None
        self.vehicle_orientation = None
        self.vehicle_rotation = None
        self.vehicle_location = None
        self.vehicle_controls = None
        self.vehicle_info = None
        self.odometry = None
        self.gnss = None
        self.imu = None
        self.vehicle_obstacle = None
        self.camera_info = None
        self.image = None
        self.dvs_camera_info = None
        self.dvs_image = None
        self.dvs_camera_events = None
        self.lidar = None
        self.semantic_lidar = None
        self.radar = None
        self.ego_vehicle_objects = None
        self.objects = None
        self.marker = None
        self.map = None
        self.world_info = None
        self.actor_list = None
        self.old_distance = 0
        self.traffic_sign_info = None
        self.traffic_lights_status = None
        self.door_status = None
        self.weather = None

        # Initialize ROS node
        self.control_override_pub = rospy.Publisher('/carla/hero/vehicle_control_manual_override', Bool, queue_size=10)
        self.control_manual_pub = rospy.Publisher('/carla/hero/vehicle_control_cmd_manual', CarlaEgoVehicleControl, queue_size=10)
        # Create a publisher for the vehicle control command
        self.control_pub = rospy.Publisher('/carla/hero/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
        self.light_control = rospy.Publisher('/carla/hero/vehicle_control_light', String, queue_size=10)
        self.door_FL_control = rospy.Publisher('/carla/hero/vehicle_toggle_FL_door', Int32, queue_size=10)
        self.door_FR_control = rospy.Publisher('/carla/hero/vehicle_toggle_FR_door', Int32, queue_size=10)
        self.door_RL_control = rospy.Publisher('/carla/hero/vehicle_toggle_RL_door', Int32, queue_size=10)
        self.door_RR_control = rospy.Publisher('/carla/hero/vehicle_toggle_RR_door', Int32, queue_size=10)

        rospy.init_node('nvidia_node', anonymous=True)

        self.subcribe_carla_topics()

        # Create a flag to track whether to exit
        self.exit_flag = False

        # Set up a callback for Ctrl + C
        rospy.on_shutdown(self.shutdown)
        
    """===========================================================================================================================================
        Begin VMP API
       ==========================================================================================================================================="""
    
    """ Set override is "True" to enable vehicle_control_manual api, "False" to disable"""
    def vehicle_control_manual_override(self, override):
        # Create a publisher for the vehicle control command
        override_msg = override
        self.control_override_pub.publish(override_msg)

    """ Control manual vehicle with code"""
    def vehicle_control_manual(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False, manual_gear_shift=False, gear=0):
        # Create a message object
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        control_msg.brake = brake
        control_msg.hand_brake = hand_brake
        control_msg.reverse = reverse
        control_msg.manual_gear_shift = manual_gear_shift
        control_msg.gear = gear
        # Publish the message
        self.control_manual_pub.publish(control_msg)
    """ Control vehicle with code and wheel hardware overlap"""
    def vehicle_control(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False, manual_gear_shift=False, gear=0):
        # Create a message object
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        control_msg.brake = brake
        control_msg.hand_brake = hand_brake
        control_msg.reverse = reverse
        control_msg.manual_gear_shift = manual_gear_shift
        control_msg.gear = gear
        # Publish the message
        self.control_pub.publish(control_msg)

    def vehicle_control_throttle(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False, manual_gear_shift=False, gear=0):

        # Create a publisher for the vehicle control command
        control_pub = rospy.Publisher('/carla/hero/vehicle_control_throttle', CarlaEgoVehicleControl, queue_size=10)

        # Create a message object
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = throttle
        control_msg.steer = steer
        control_msg.brake = brake
        control_msg.hand_brake = hand_brake
        control_msg.reverse = reverse
        control_msg.manual_gear_shift = manual_gear_shift
        control_msg.gear = gear
        # Publish the message
        control_pub.publish(control_msg)
    
    def vehicle_control_throttle_switch(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False, manual_gear_shift=False, gear=0):

        # Create a publisher for the vehicle control command
        control_pub = rospy.Publisher('/carla/hero/vehicle_control_throttle_switch', CarlaEgoVehicleControl, queue_size=10)

        # Create a message object
        control_msg = CarlaEgoVehicleControl()
        # control_msg.throttle = throttle
        control_msg.steer = steer
        control_msg.brake = brake
        control_msg.hand_brake = hand_brake
        control_msg.reverse = reverse
        control_msg.manual_gear_shift = manual_gear_shift
        control_msg.gear = gear
        # Publish the message
        control_pub.publish(control_msg)

    def vehicle_control_toggle_door_FL(self):
        self.door_FL_control.publish(1)
    
    def vehicle_control_toggle_door_RL(self):
        self.door_RL_control.publish(1)
    
    def vehicle_control_toggle_door_FR(self):
        self.door_FR_control.publish(1)
    
    def vehicle_control_toggle_door_RR(self):
        self.door_RR_control.publish(1)
    
    """ Input control: "On"/"Off"
        1.2 (m/s)
    """
    def vehicle_control_light(self, control):
        self.light_control.publish(control)

    """ Velocity value return:
        1.2 (m/s)
    """
    def get_vehicle_velocity(self):
        return self.vehicle_velocity
    
    """ Acceleration value return:
    {   
        linear: 
            x: 0.0
            y: -0.0
            z: 0.0
        angular: 
            x: 0.0
            y: 0.0
            z: 0.0
    }
    """
    def get_vehicle_acceleration(self):
        return self.vehicle_acceleration
    
    """ Orientation value return:
    {   
        y: -1.6292922383290965e-06
        z: 0.42434795213668
        w: 0.905499207903972
    }
    """
    def get_vehicle_orientation(self):
        return self.vehicle_orientation
    
    """ Rotation value return:
    {   
        roll: -6.103515625e-05
        pitch: 0.00017758490866981447
        yaw: -50.21879577636719
    }
    """
    def get_vehicle_rotation(self):
        return self.vehicle_rotation
    
    """ Location value return:
    {   
        x: 150.9740447998047
        y: -13.36904525756836
        z: 0.040282171219587326
    }
    """
    def get_vehicle_location(self):
        return self.vehicle_location
    
    """ Control value return:
    {   
        stamp: 
            secs: 0
            nsecs:         0
        frame_id: ''
        throttle: 0.0
        steer: 0.0
        brake: 0.0
        hand_brake: False
        reverse: True
        gear: -1
        manual_gear_shift: True 
    }
    """
    def get_vehicle_controls(self):
        return self.vehicle_controls
    
    def get_carla_vehicle_info(self):
        """
        gets vehicle_info
        """
        return self.vehicle_info

    def get_odometry(self):
        """
        gets Odometry
        """
        return self.odometry
    
    """ Gnss value return:
        {   
            latitude: 0.0
            longitude: 0.0
            altitude: 0.0
        }
    """
    def get_gnss(self):
        """
        gets Gnss
        """
        return self.gnss
    
    """ Imu value return:
        {   
            linear_acceleration: 0.0
            angular_velocity: 0.0
            orientation: 0.0
        }
    """
    def get_imu(self):
        """
        gets IMU sensor node
        """
        return self.imu
    
    """ Obstacle value return:
    {   
        is_obstacle: True/False
        obstacle_actor: vehicle/walker
        obstacle_distance: 5.243424241
    }
    """
    def get_vehicle_obstacle(self):
        return self.vehicle_obstacle
    
    """ Vehicle door value return:
    {   
        FL: "Close"/"Open"
        FR: "Close"/"Open"
        RL: "Close"/"Open"
        RR: "Close"/"Open"
    }
    """
    def get_vehicle_door(self):
        return self.door_status
    
    """ Traffic light value return:
    {   
        [{
            id: 213,
            transform: {
            
            },
            state: 0/1/2 (RED=0 YELLOW=1 GREEN=2)
        },
        {
            id: 214,
            transform: {
            
            },
            state: 0/1/2 (RED=0 YELLOW=1 GREEN=2)
        },...
        ]
    }
    """
    def get_traffic_lights(self):
        if (not self.traffic_lights_status == None):
            return self.traffic_lights_status.traffic_lights
    
    """ Traffic sign value return:
    {   
        [{
            id: 217,
            transform: {
            
            },
            type: 1/.../10      #   Stop = 1
                                #   Speed Limited 40 = 2
                                #   Speed Limited 60 = 3
                                #   Speed Limited 90 = 4
                                #   Direct Turn Left = 5
                                #   Direct Turn Right = 6
                                #   Direct Straight = 7
                                #   Prohibiting right turn = 8
                                #   Prohibiting left turn = 9
                                #   Prohibiting straight turn = 10
        },
        {
            id: 219,
            transform: {
            
            },
            type: 1/.../10      #   Stop = 1
                                #   Speed Limited 40 = 2
                                #   Speed Limited 60 = 3
                                #   Speed Limited 90 = 4
                                #   Direct Turn Left = 5
                                #   Direct Turn Right = 6
                                #   Direct Straight = 7
                                #   Prohibiting right turn = 8
                                #   Prohibiting left turn = 9
                                #   Prohibiting straight turn = 10
        },...
        ]
    }
    """
    def get_traffic_signs(self):
        return self.traffic_sign_info
    
    """ Weather value return:
    {     
        "ClearDay"/ "ClearNight"/ "DayRaining"/ "DayAfterRain"
    }
    """
    def get_weather(self):
        return self.weather

    """===========================================================================================================================================
       Finish VMP API 
       ==========================================================================================================================================="""
    
    def get_carla_clock(self, message):
        self.clock = message

    def get_carla_vehicle_status_msg(self, msg):
        self.vehicle_velocity = msg.velocity
        self.vehicle_acceleration = msg.acceleration
        self.vehicle_orientation = msg.orientation
        self.vehicle_rotation = msg.rotation
        self.vehicle_location = msg.location
        self.vehicle_controls = msg.control

    def get_carla_vehicle_info_msg(self, msg):
        """
        gets vehicle_info
        """
        self.vehicle_info = msg
    
    def get_carla_odometry_msg(self, msg):
        """
        gets Odometry
        """
        self.odometry = msg

    def get_vehicle_gnss_msg(self, msg):
        """
        gets Gnss
        """
        self.gnss = msg

    def get_vehicle_imu_msg(self, msg):
        """
        gets IMU sensor node
        """
        self.imu = msg

    def get_vehicle_obstacle_msg(self, msg):
        """
        gets obstacle sensor
        """
        if (self.old_distance != msg.obstacle_distance):
            self.old_distance = msg.obstacle_distance
            msg.is_obstacle = True
        else:
            self.old_distance = msg.obstacle_distance
            msg.is_obstacle = False
            msg.obstacle_actor = "None"
            msg.obstacle_distance = 200
        self.vehicle_obstacle = msg
    
    def get_traffic_light_status_msg(self, msg):
        """
        get traffic lights
        """
        self.traffic_lights_status = msg
    
    def get_traffic_sign_info_msg(self, msg):
        """
        get traffic sign info
        """
        self.traffic_sign_info = msg

    def get_weather_status_msg(self, msg):
        """
        get weather status
        """
        self.weather = msg.data

    def get_door_status_msg(self, msg):
        """
        get door status
        """
        self.door_status = msg

    
    def shutdown(self):
        # Set the exit flag when Ctrl + C is detected
        print("Shutting down")
        self.exit_flag = True

    def subcribe_carla_topics(self):
        rospy.Subscriber(   "/clock", Clock, self.get_carla_clock)
        rospy.Subscriber(   "/carla/hero/vehicle_status", CarlaEgoVehicleStatus, self.get_carla_vehicle_status_msg)
        rospy.Subscriber(   "/carla/hero/vehicle_info", CarlaEgoVehicleInfo, self.get_carla_vehicle_info_msg)
        rospy.Subscriber(   "/carla/hero/odometry", Odometry, self.get_carla_odometry_msg)
        rospy.Subscriber(   "/carla/hero/gnss", NavSatFix, self.get_vehicle_gnss_msg)
        rospy.Subscriber(   "/carla/hero/imu", Imu, self.get_vehicle_imu_msg)
        rospy.Subscriber(   "/carla/hero/obstacle", CarlaEgoVehicleObstacle, self.get_vehicle_obstacle_msg)
        rospy.Subscriber(   "/carla/hero/vehicle_door_status", CarlaEgoVehicleDoorStatus, self.get_door_status_msg)
        rospy.Subscriber(   "/carla/traffic_light_status", CarlaTrafficLightList, self.get_traffic_light_status_msg)
        rospy.Subscriber(   "/carla/traffic_sign_info", CarlaTrafficSignList, self.get_traffic_sign_info_msg)
        rospy.Subscriber(   "/carla/weather_status", String, self.get_weather_status_msg)

    # def get_camera_info(self):
    #     """
    #     gets camera_info
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/rgb_front/camera_info", CameraInfo, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/rgb_front")
    #     self.assertEqual(msg.height, 600)
    #     self.assertEqual(msg.width, 800)

    # def get_camera_image(self):
    #     """
    #     gets camera_images
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/rgb_front/image", Image, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/rgb_front")
    #     self.assertEqual(msg.height, 600)
    #     self.assertEqual(msg.width, 800)
    #     self.assertEqual(msg.encoding, "bgra8")

    # def get_dvs_camera_info(self):
    #     """
    #     gets dvs camera info
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/dvs_front/camera_info", CameraInfo, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/dvs_front")
    #     self.assertEqual(msg.height, 70)
    #     self.assertEqual(msg.width, 400)

    # def get_dvs_camera_image(self):
    #     """
    #     gets dvs camera images
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/dvs_front/image", Image, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/dvs_front")
    #     self.assertEqual(msg.height, 70)
    #     self.assertEqual(msg.width, 400)
    #     self.assertEqual(msg.encoding, "bgr8")

    # def get_dvs_camera_events(self):
    #     """
    #     gets dvs camera events
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/dvs_front/events", PointCloud2, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/dvs_front")

    # def get_lidar(self):
    #     """
    #     gets Lidar sensor node
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/lidar", PointCloud2, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/lidar")

    # def get_semantic_lidar(self):
    #     """
    #     gets semantic_lidar sensor node
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/semantic_lidar", PointCloud2, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/semantic_lidar")

    # def get_radar(self):
    #     """
    #     gets Radar sensor node
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/radar_front", PointCloud2, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "hero/radar_front")

    # def get_ego_vehicle_objects(self):
    #     """
    #     gets objects node for ego_vehicle
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/hero/objects", ObjectArray, timeout=15)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "map")
    #     self.assertEqual(len(msg.objects), 0)

    # def get_objects(self):
    #     """
    #     gets carla objects
    #     """
    #     msg = rospy.wait_for_message("/carla/objects", ObjectArray, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(msg.header.frame_id, "map")
    #     self.assertEqual(len(msg.objects), 1)  # only ego vehicle exists

    # def get_marker(self):
    #     """
    #     gets marker
    #     """
    #     msg = rospy.wait_for_message("/carla/markers", MarkerArray, timeout=TIMEOUT)
    #     return msg
    #     self.assertEqual(len(msg.markers), 1)  # only ego vehicle exists

    #     ego_marker = msg.markers[0]
    #     self.assertEqual(ego_marker.header.frame_id, "map")
    #     self.assertNotEqual(ego_marker.id, 0)
    #     self.assertEqual(ego_marker.type, 1)
    #     self.assertNotEqual(ego_marker.pose, Pose())
    #     self.assertNotEqual(ego_marker.scale, Vector3())
    #     self.assertEqual(ego_marker.color.r, 0.0)
    #     self.assertEqual(ego_marker.color.g, 255.0)
    #     self.assertEqual(ego_marker.color.b, 0.0)

    # def get_map(self):
    #     """
    #     gets map
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/map", String, timeout=TIMEOUT)
    #     return msg
    #     self.assertNotEqual(len(msg.data), 0)

    # def get_world_info(self):
    #     """
    #     gets world_info
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/world_info", CarlaWorldInfo, timeout=TIMEOUT)
    #     return msg
    #     self.assertNotEqual(len(msg.map_name), 0)
    #     self.assertNotEqual(len(msg.opendrive), 0)

    # def get_actor_list(self):
    #     """
    #     gets actor_list
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/actor_list", CarlaActorList, timeout=TIMEOUT)
    #     return msg
    #     self.assertNotEqual(len(msg.actors), 0)

    # def get_traffic_lights(self):
    #     """
    #     gets traffic_lights
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/traffic_lights/status", CarlaTrafficLightStatusList, timeout=TIMEOUT)
    #     return msg
    #     self.assertNotEqual(len(msg.traffic_lights), 0)

    # def get_traffic_lights_info(self):
    #     """
    #     gets traffic_lights
    #     """
    #     msg = rospy.wait_for_message(
    #         "/carla/traffic_lights/info", CarlaTrafficLightInfoList, timeout=TIMEOUT)
    #     return msg
    #     self.assertNotEqual(len(msg.traffic_lights), 0)

class VehicleMotionAPI():

    def __init__(self):

        self.General = General()
    

