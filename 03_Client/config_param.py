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
            sign_speed_limited_40 = 2
            sign_speed_limited_60 = 3
            sign_speed_limited_90 = 4
            sign_direct_left_turn = 5
            sign_direct_right_turn = 6
            sign_direct_straight = 7
            sign_prohibiting_right_turn = 8
            sign_prohibiting_left_turn = 9
            sign_prohibiting_straight_turn = 10

            self.traffic_area_1 = [
                (19.5, 9.2),  # Point 1 (x, y)
                (19.5, -9.2),  # Point 2 (x, y)
                (40, -9.2),  # Point 3 (x, y)
                (40, 9.2)   # Point 4 (x, y)
            ]

            self.weather_area_1 = [
                (15, 82),  # Point 1 (x, y)
                (15, 99),  # Point 2 (x, y)
                (-60, 99),  # Point 3 (x, y)
                (-60, 82)   # Point 4 (x, y)
            ]

            self.weather_area_2 = [
                (-40, 76),  # Point 1 (x, y)
                (-59, 76),  # Point 2 (x, y)
                (-59, 30),  # Point 3 (x, y)
                (-40, 30)   # Point 4 (x, y)
            ]

            self.stop_area_1 = [
                (-86, 90),  # Point 1 (x, y)
                (-91.3, 90),  # Point 2 (x, y)
                (-91.3, 97.7),  # Point 3 (x, y)
                (-86, 97.7)   # Point 4 (x, y)
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

            self.stop_sign = CarlaTrafficSign()
            self.stop_sign.id = 1
            self.stop_sign.transform = tranform
            self.stop_sign.type = sign_stop

            self.sign_speed_limited_40 = CarlaTrafficSign()
            self.sign_speed_limited_40.id = 2
            self.sign_speed_limited_40.transform = tranform
            self.sign_speed_limited_40.type = sign_speed_limited_40

            self.sign_speed_limited_90 = CarlaTrafficSign()
            self.sign_speed_limited_90.id = 3
            self.sign_speed_limited_90.transform = tranform
            self.sign_speed_limited_90.type = sign_speed_limited_90

            self.sign_direct_right_turn = CarlaTrafficSign()
            self.sign_direct_right_turn.id = 4
            self.sign_direct_right_turn.transform = tranform
            self.sign_direct_right_turn.type = sign_direct_right_turn

            self.sign_direct_straight = CarlaTrafficSign()
            self.sign_direct_straight.id = 5
            self.sign_direct_straight.transform = tranform
            self.sign_direct_straight.type = sign_direct_straight

            self.sign_prohibiting_straight_turn = CarlaTrafficSign()
            self.sign_prohibiting_straight_turn.id = 6
            self.sign_prohibiting_straight_turn.transform = tranform
            self.sign_prohibiting_straight_turn.type = sign_prohibiting_straight_turn
            
            self.traffic_sign_list = CarlaTrafficSignList([self.stop_sign, self.sign_speed_limited_40, self.sign_speed_limited_90, self.sign_direct_right_turn, self.sign_direct_straight, self.sign_prohibiting_straight_turn])
            # print(self.traffic_sign_list)

