import carla
from carla_msgs.msg import (CarlaWorldInfo, CarlaTrafficSign, CarlaTrafficSignList)
from geometry_msgs.msg import Pose

class ParkingScenario():
        def __init__(self):
            self.parking_first_point = 0.0
            self.vehicle_init_position  = [
                carla.Transform(carla.Location(x=-265, y=-10, z=0.05), carla.Rotation(yaw=270))]

class RoundOneScenario():
        def __init__(self):

            stop_traffic_sign_bp = "static.prop.stop"
            speed_limit_sign_30_bp = "static.prop.speedlimit.30"
            speed_limit_sign_40_bp = "static.prop.speedlimit.40"
            speed_limit_sign_50_bp = "static.prop.speedlimit.50"
            speed_limit_sign_60_bp = "static.prop.speedlimit.60"
            speed_limit_sign_90_bp = "static.prop.speedlimit.90"
            left_sign_pb = "static.prop.arrowleft"
            right_sign_pb = "static.prop.arrowright"
            straight_sign_pb = "static.prop.arrowstraight"
            crosswalk_sign_pb = "static.prop.crosswalk"

            sign_stop = 1
            sign_speed_limited_30 = 2
            sign_speed_limited_60 = 3
            sign_speed_limited_90 = 4
            sign_direct_left_turn = 5
            sign_direct_right_turn = 6
            sign_direct_straight = 7
            sign_prohibiting_right_turn = 8
            sign_prohibiting_left_turn = 9
            sign_prohibiting_straight_turn = 10

            self.traffic_area_1 = [
                (-60, 75),  # Point 1 (x, y)
                (-60, 99),  # Point 2 (x, y)
                (-64, 99),  # Point 3 (x, y)
                (-64, 75)   # Point 4 (x, y)
            ]

            self.traffic_area_2 = [
                (16.3, 100),  # Point 1 (x, y)
                (16.3, 80),  # Point 2 (x, y)
                (22, 80),  # Point 3 (x, y)
                (22, 100)   # Point 4 (x, y)
            ]

            self.weather_area_1 = [
                (15, 82),  # Point 1 (x, y)
                (15, 99),  # Point 2 (x, y)
                (-37, 99),  # Point 3 (x, y)
                (-37, 82)   # Point 4 (x, y)
            ]

            self.weather_area_2 = [
                (-40, 76),  # Point 1 (x, y)
                (-59, 76),  # Point 2 (x, y)
                (-59, 30),  # Point 3 (x, y)
                (-40, 30)   # Point 4 (x, y)
            ]

            self.stop_area_1 = [
                (-86, 90),  # Point 1 (x, y)
                (-93, 90),  # Point 2 (x, y)
                (-93, 97),  # Point 3 (x, y)
                (-86, 97)   # Point 4 (x, y)
            ]

            self.limit_speed_start = [
                (46, 75),  # Point 1 (x, y)
                (41, 75),  # Point 2 (x, y)
                (41, 98),  # Point 3 (x, y)
                (46, 98)   # Point 4 (x, y)
            ]

            self.limit_speed_stop = [
                (92, 41),  # Point 1 (x, y)
                (92, 44.5),  # Point 2 (x, y)
                (109, 44.5),  # Point 3 (x, y)
                (109, 41)   # Point 4 (x, y)
            ]

            self.pedestrian_area_1 = [
                (80, 10),  # Point 1 (x, y)
                (90, 10),  # Point 2 (x, y)
                (90, -10),  # Point 3 (x, y)
                (80, -10)   # Point 4 (x, y)
            ]

            self.parking_first_point = 0.0
            self.vehicle_init_position  = [
                carla.Transform(carla.Location(x=-265, y=-10, z=0.05), carla.Rotation(yaw=270))]
            #   Stop = 1
            #   Speed Limited 40 = 2
            #   Speed Limited 60 = 3
            #   Speed Limited 90 = 4
            #   Direct Turn Left = 5
            #   Direct Turn Right = 6
            #   Direct Straight = 7
            #   Prohibiting right turn = 8
            #   Prohibiting left turn = 9
            #   Prohibiting straight turn = 10
            tranform = Pose()
            tranform.position.x = 265
            tranform.position.y = -10
            tranform.position.z = 0.05

            tranform1 = Pose()
            tranform1.position.x = -143
            tranform1.position.y = 73.8
            tranform1.position.z = 0.05

            tranform2 = Pose()
            tranform2.position.x = 14
            tranform2.position.y = 98.6
            tranform2.position.z = 0.05

            tranform3 = Pose()
            tranform3.position.x = 43.6
            tranform3.position.y = 99.4
            tranform3.position.z = 0.05

            tranform3 = Pose()
            tranform3.position.x = 110
            tranform3.position.y = 30.4
            tranform3.position.z = 0.05

            tranform4 = Pose()
            tranform4.position.x = 110.9
            tranform4.position.y = 11
            tranform4.position.z = 0.05


            self.stop_sign = CarlaTrafficSign()
            self.stop_sign.id = 1
            self.stop_sign.transform = tranform
            self.stop_sign.type = sign_stop

            self.sign_speed_limited_30 = CarlaTrafficSign()
            self.sign_speed_limited_30.id = 2
            self.sign_speed_limited_30.transform = tranform3
            self.sign_speed_limited_30.type = sign_speed_limited_30

            self.sign_speed_limited_90 = CarlaTrafficSign()
            self.sign_speed_limited_90.id = 3
            self.sign_speed_limited_90.transform = tranform
            self.sign_speed_limited_90.type = sign_speed_limited_90

            self.sign_direct_left_turn = CarlaTrafficSign()
            self.sign_direct_left_turn.id = 4
            self.sign_direct_left_turn.transform = tranform1
            self.sign_direct_left_turn.type = sign_direct_left_turn

            self.sign_direct_left_turn_1 = CarlaTrafficSign()
            self.sign_direct_left_turn_1.id = 5
            self.sign_direct_left_turn_1.transform = tranform4
            self.sign_direct_left_turn_1.type = sign_direct_left_turn

            self.sign_prohibiting_left_turn = CarlaTrafficSign()
            self.sign_prohibiting_left_turn.id = 6
            self.sign_prohibiting_left_turn.transform = tranform2
            self.sign_prohibiting_left_turn.type = sign_prohibiting_left_turn
            
            self.traffic_sign_list = CarlaTrafficSignList([self.stop_sign, self.sign_speed_limited_30, self.sign_speed_limited_90, self.sign_direct_left_turn, self.sign_direct_left_turn_1, self.sign_prohibiting_left_turn])
            # print(self.traffic_sign_list)

