import time
import random
import rospy
import carla
from carla_msgs.msg import ( CarlaWorldInfo, CarlaEgoVehicleDoorStatus )
from config_param import ParkingScenario
from carla import VehicleLightState as vls
import math
import random

class Vehicle_Control():
    def __init__(self):
        """
        construct object CarlaVehicle with server connection and
        ros node initiation
        """
        rospy.init_node('park_vehithrottlecle', anonymous=True)
        self.parkingScenrio = ParkingScenario()
        self.actor_list = []
        self.lights = carla.VehicleLightState.NONE
        self.door_status = CarlaEgoVehicleDoorStatus()
        self.door_status.FL = "Close"
        self.door_status.FR = "Close"
        self.door_status.RL = "Close"
        self.door_status.RR = "Close"


    def create_car(self, world):

        blueprint_library = world.get_blueprint_library()\
        
        #create ego vehicle
        bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
        ap = random.choice(blueprint_library.filter('vehicle.dodge.charger_2020'))

        ap.set_attribute('role_name', "hero")
        new_location = carla.Location(x=-246.8, y=-35.2, z=12)
        new_transform = carla.Transform(new_location, carla.Rotation(yaw=90))
        
        # blueprint = random.choice(self.get_actor_blueprints(world, 'vehicle.*', "2"))
        # blueprint.set_attribute('role_name', "ego_vehicle")
        # if blueprint.has_attribute('color'):
        #     color = random.choice(blueprint.get_attribute('color').recommended_values)
        #     blueprint.set_attribute('color', color)
        # if blueprint.has_attribute('driver_id'):
        #     driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        #     blueprint.set_attribute('driver_id', driver_id)
        # if blueprint.has_attribute('is_invincible'):
        #     blueprint.set_attribute('is_invincible', 'true')
        # # set the max speed
        # if blueprint.has_attribute('speed'):
        #     self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
        #     self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # DReyeVR_vehicles: str = "harplab.dreyevr_vehicle.*"
        # ego_vehicles_in_world = list(world.get_actors().filter(DReyeVR_vehicles))
        
        # if len(ego_vehicles_in_world) >= 1:
        #     print(
        #         f"Found a DReyeVR EgoVehicle in the world ({ego_vehicles_in_world[0].id})"
        #     )
        #     self.vehicle = ego_vehicles_in_world[0]
        #     # Set the vehicle's transform
        #     self.vehicle.set_transform(new_transform)
            
        #     if 'vehicle' in self.vehicle.type_id:
        #         # Get the 'role_name' attribute.
        #         role_name = self.vehicle.attributes.get('role_name', 'No role_name attribute found')
        #         print(role_name)
        #     else:
        #         print("Specified actor is not a vehicle.")
        # else:
        #     self.vehicle = world.spawn_actor(bp, self.parkingScenrio.vehicle_init_position[0])
        #     self.actor_list.append(self.vehicle)
        self.vehicle = world.spawn_actor(ap, new_transform)
        self.actor_list.append(self.vehicle)
        self.noises_enable = 0
        vehicle_width = self.vehicle.bounding_box.extent.x * 2
        vehicle_length = self.vehicle.bounding_box.extent.y * 2
        print("Vehicle Width:", vehicle_width)
        print("Vehicle Length:", vehicle_length)

        wheelbase_x = self.vehicle.get_physics_control().wheels[0].position.x - self.vehicle.get_physics_control().wheels[2].position.x
        wheelbase_y = self.vehicle.get_physics_control().wheels[0].position.y - self.vehicle.get_physics_control().wheels[2].position.y
        
        wheelbase = math.sqrt(wheelbase_x**2 + wheelbase_y**2)

        print(self.vehicle.get_physics_control().wheels[0].position.x)
        print(self.vehicle.get_physics_control().wheels[0].position.y)
        print(self.vehicle.get_physics_control().wheels[2].position.x)
        print(self.vehicle.get_physics_control().wheels[2].position.y)
        print("wheelbase", wheelbase)

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

    def toggle_FR_door(self):
        if self.door_status.FR == "Close":
            self.door_status.FR = "Open"
            self.vehicle.open_door(carla.VehicleDoor.FR)
        else:
            self.door_status.FR = "Close"
            self.vehicle.close_door(carla.VehicleDoor.FR)

    def toggle_FL_door(self):
        if self.door_status.FL == "Close":
            self.door_status.FL = "Open"
            self.vehicle.open_door(carla.VehicleDoor.FL)
        else:
            self.door_status.FL = "Close"
            self.vehicle.close_door(carla.VehicleDoor.FL)

    def toggle_RR_door(self):
        if self.door_status.RR == "Close":
            self.door_status.RR = "Open"
            self.vehicle.open_door(carla.VehicleDoor.RR)
        else:
            self.door_status.RR = "Close"
            self.vehicle.close_door(carla.VehicleDoor.RR)
    
    def toggle_RL_door(self):
        if self.door_status.RL == "Close":
            self.door_status.RL = "Open"
            self.vehicle.open_door(carla.VehicleDoor.RL)
        else:
            self.door_status.RL = "Close"
            self.vehicle.close_door(carla.VehicleDoor.RL)

    def set_door(self, status):
        self.door_status = status
        if status:
            self.vehicle.open_door(carla.VehicleDoor.All)
        else:
            self.vehicle.close_door(carla.VehicleDoor.All)

    def open_random_door(self):
        rl = 0
        rr = 0
        fl = 0
        fr = 0
        while (not (rl or rr or fl or fr)):
            rl = random.randint(0, 1)
            rr = random.randint(0, 1)
            fl = random.randint(0, 1)
            fr = random.randint(0, 1)
        
        if(rl):
            self.door_status.RL = "Open"
            self.vehicle.open_door(carla.VehicleDoor.RL)
        else:
            self.door_status.RL = "Close"
            self.vehicle.close_door(carla.VehicleDoor.RL)
        if(rr):
            self.door_status.RR = "Open"
            self.vehicle.open_door(carla.VehicleDoor.RR)
        else:
            self.door_status.RR = "Close"
            self.vehicle.close_door(carla.VehicleDoor.RR)
        if(fl):
            self.door_status.FL = "Open"
            self.vehicle.open_door(carla.VehicleDoor.FL)
        else:
            self.door_status.FL = "Close"
            self.vehicle.close_door(carla.VehicleDoor.FL)
        if(fr):
            self.door_status.FR = "Open"
            self.vehicle.open_door(carla.VehicleDoor.FR)
        else:
            self.door_status.FR = "Close"
            self.vehicle.close_door(carla.VehicleDoor.FR)

    def get_door_status(self):
        return self.door_status
    
    def set_light_on(self):
        # self.lights |= carla.VehicleLightState.LeftBlinker
        # self.lights |= carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState(vls.Position | vls.HighBeam))

    def set_light_off(self):
        # self.lights &= ~carla.VehicleLightState.LeftBlinker
        # self.lights &= ~carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState.NONE)

    def set_light_brake_on(self):
        self.lights |= carla.VehicleLightState.Brake
        # self.lights |= carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState(self.lights))

    def set_light_brake_off(self):
        self.lights &= ~carla.VehicleLightState.Brake
        # self.lights &= ~carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState(self.lights))

    def set_light_blink(self):
        self.lights |= carla.VehicleLightState.LeftBlinker
        self.lights |= carla.VehicleLightState.RightBlinker
        self.vehicle.set_light_state(carla.VehicleLightState(self.lights))

    def set_light_unblink(self):
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
