import time
import random
import rospy
import carla
from carla_msgs.msg import CarlaWorldInfo
from config_param import ParkingScenario
from config_param import RoundOneScenario
import math

class Scenario_Runner():
    def __init__(self):
        """
        construct object CarlaVehicle with server connection and
        ros node initiation
        """
        rospy.init_node('park_vehithrottlecle', anonymous=True)
        self.parkingScenrio = ParkingScenario()
        self.roundOneScenario = RoundOneScenario()
        self.actor_list = []
        self.lights = carla.VehicleLightState.NONE
        self.door_status = 0
        self.time_stop = 0
        self.is_in_stop = False
        self.is_cross_stop = False
        self.is_successful_stop = False
        self.was_in_limit_speed_start = False
        self.was_create_pedestrian = False
        self.is_pass_traffic_1 = False
        self.is_pass_traffic_2 = False

    # def order_points_counterclockwise(self, points):
    #     # Calculate the centroid of the polygon
    #     center = tuple(sum(x) / len(points) for x in zip(*points))
    #     # Sort points by angle from the center
    #     sorted_points = sorted(points, key=lambda p: math.atan2(p[1] - center[1], p[0] - center[0]), reverse=True)
    #     return sorted_points

    def is_point_in_polygon(self, point, polygon):
        # point = self.order_points_counterclockwise(_point)
        x, y = point
        inside = False
        n = len(polygon)
        p1x, p1y = polygon[0]
        for i in range(n+1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y-p1y) * (p2x-p1x) / (p2y-p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def is_vehicle_in_traffic_area_1(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.traffic_area_1)

    def is_vehicle_in_traffic_area_2(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.traffic_area_2)

    def is_vehicle_in_weather_area_1(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.weather_area_1)

    def is_vehicle_in_weather_area_2(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.weather_area_2)
    
    def is_vehicle_in_stop_area_1(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.stop_area_1)
    
    def is_vehicle_in_limit_speed_start(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.limit_speed_start)

    def is_vehicle_in_limit_speed_stop(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.limit_speed_stop)

    def is_vehicle_in_limit_pedestrian_area_1(self, vehicle):
        vehicle_location = vehicle.get_transform().location
        vehicle_point = (vehicle_location.x, vehicle_location.y)
        return self.is_point_in_polygon(vehicle_point, self.roundOneScenario.pedestrian_area_1)

    def check_traffic_light_1(self, vehicle, hud, controller, world_carla):
        if (self.is_vehicle_in_traffic_area_1(vehicle)):
            if (not self.is_pass_traffic_1):
                self.is_pass_traffic_1 = True
                hud.short_minus_score(1)
                hud.notification("Cross the red traffic light => score - 1")
                hud.crossRTL+=1

    def check_traffic_light_2(self, vehicle, hud, controller, world_carla):
         if (self.is_vehicle_in_traffic_area_2(vehicle)):
            if (not self.is_pass_traffic_2):
                self.is_pass_traffic_2 = True
                hud.short_minus_score(1)
                hud.notification("Cross the red traffic light => score - 1")
                hud.crossRTL+=1

    def counting_stop_point(self, vehicle, hud):
        if (self.is_vehicle_in_stop_area_1(vehicle)):
            if (not self.is_in_stop):
                self.time_stop = time.time()
                self.is_in_stop = True
            if (not self.is_successful_stop and time.time() - self.time_stop > 3):
                hud.notification("Stop satified!")
                self.is_successful_stop = True
        elif (not self.is_successful_stop and self.is_in_stop and time.time() - self.time_stop > 4):
                hud.short_minus_score(1)
                hud.crossStop = hud.crossStop + 1
                self.is_successful_stop = True

    def check_speed_limited_1(self, vehicle, hud, controller, world_carla):
        if (self.is_vehicle_in_limit_speed_start(vehicle) and not self.was_in_limit_speed_start):
            self.was_in_limit_speed_start = True

        if (self.is_vehicle_in_limit_speed_stop(vehicle) and self.was_in_limit_speed_start):
            self.was_in_limit_speed_start = False
            controller.create_car_1(world_carla)
            
        if (self.was_in_limit_speed_start and hud.vehicle_speed > 30):
            hud.is_minus = True
        else:
            hud.is_minus = False

    def check_pedestrian_1(self, vehicle, controller, world_carla):
        if(self.is_vehicle_in_limit_pedestrian_area_1(vehicle) and not self.was_create_pedestrian):
            controller.create_pedestria_1(world_carla)
            self.was_create_pedestrian = True
            

    def destroy(self):
        """
        destroy all the actors
        """
        print('destroying actors')
        for actor in self.actor_list:
            if actor is not None:
                actor.destroy()
        print('done.')

    def get_actor_blueprints(self, world, filter, generation):
        bps = world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return bps
            else:
                print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                return []
        except:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
        
    def set_door(self, status):
        self.door_status = status
        if status:
            self.vehicle.open_door(carla.VehicleDoor.All)
        else:
            self.vehicle.close_door(carla.VehicleDoor.All)

    def get_door_status(self):
        return self.door_status
    
    def set_light_blink(self):
        self.lights |= carla.VehicleLightState.LeftBlinker
        self.lights |= carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState(self.lights))

    def set_light_not_blink(self):
        self.lights &= ~carla.VehicleLightState.LeftBlinker
        self.lights &= ~carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState(self.lights))

    def vehicle_control_with_latency_and_error(self, control_throttle, control_steer, control_brake, vehicle_reverse = False, vehicle_handbrake = False, vehicle_mg_shift = False, vehicle_gear = 0):
        if self.noises_enable == 1:
            if abs(control_throttle) > 0.1:
                #control_throttle = random.uniform(control_throttle - 0.001, control_throttle + 0.001)
                #temp_throttle_with_noise = np.random.normal(control_throttle, (control_throttle*0.01) ,1)
                #control_throttle = temp_throttle_with_noise[0]
                pass
            if control_steer > 0.08:
                temp_steer_with_noises = np.random.normal(control_steer, (control_steer*0.1) ,1)
                control_steer = temp_steer_with_noises[0]
             
        # carla.VehicleControl.gear = gear
        self.vehicle.apply_control(
            carla.VehicleControl(throttle = control_throttle, steer=control_steer, brake=control_brake, reverse=vehicle_reverse, hand_brake = vehicle_handbrake, manual_gear_shift = vehicle_mg_shift, gear = vehicle_gear)
        )
