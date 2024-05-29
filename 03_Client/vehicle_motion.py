import glob
import os
import sys
import re
import weakref
import collections
import datetime
import random
import math
import time
import threading
import rospy
from std_msgs.msg import Int32
from configparser import ConfigParser


import numpy as np


try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_r
    from pygame.locals import K_t
    from pygame.locals import K_s
    from pygame.locals import K_w

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

import carla
from carla import ColorConverter as cc

def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class World(object):
    def __init__(self, carla_world, hud, actor_filter, _player, ros_connection):
        self.world = carla_world
        self.hud = hud
        self._player = _player
        self._ros_connection = ros_connection
        self.imu_sensor = IMUSensor(_player)
        self.player = None
        self.collision_sensor = None
        self.obstacle_detector = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self._player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self._player, self.hud)
        self.obstacle_detector = ObstacleDetector(self._player, self.hud, self._ros_connection)
        self.gnss_sensor = GnssSensor(self._player)
        self.camera_manager = CameraManager(self._player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self._player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self._player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock, self._player)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.obstacle_detector.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        # if self._player is not None:
        #     self._player.destroy()
# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
class HUD(object):
    def __init__(self, width, height, _ros):
        self.invasion = 0
        self.collision = 0
        self.crossRTL = 0

        self.detectedTSDSpeed = 0
        self.detectedTSDStop = 0
        self.detectedTSDDirect = 0
        self.detectedTrafficLight = 0
        self.detectedPedestrian = 0
        self.detectedCar = 0
        self.detectedWeather = 0
        self.detectedDoor = 0
        self.score = 100
        self.ros = _ros
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 100))
        # self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.is_start = 0
        self.time_start = 0
        self.time_end = 0

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        if (not self.is_start):
            self.simulation_time = timestamp.elapsed_seconds - timestamp.elapsed_seconds
            self.time_start = timestamp.elapsed_seconds
        elif (self.is_start == 2):
            self.simulation_time = self.time_end - self.time_start
        else:
            self.simulation_time = timestamp.elapsed_seconds - self.time_start
            self.time_end = timestamp.elapsed_seconds


    def tick(self, world, clock, player):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = player.get_transform()
        v = player.get_velocity()
        c = player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        if ( (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)) > 1 and self.is_start != 2):
            self.is_start = 1
            if (( (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)) > 60)):
                self.is_start = 2
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('static.*')
        self.ros.publish_speed(int(round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2), 0)))
        self.ros.publish_brake(round(c.brake*110, 2))
        self.ros.publish_steer(int(round(c.steer*540,0)))
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            '',
            'Vehicle: % 20s' % get_actor_display_name(player, truncate=20),
            'Map:     % 20s' % world.world.get_map().name.split('/')[-1],
            'Run time: % 18s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',]
    
        self._info_text += [
            'Number of lane invasion: % 4d' % self.invasion
            ]
        self._info_text += [
            'Number of collision: % 4d' % self.collision
            ]
        self._info_text += [
            'Number of cross Red TL: % 4d' % self.crossRTL
            ]
        self._info_text += [
            'Number of cross Red TL: % 4d' % self.crossRTL
            ]
        self._info_text += [
            'Score: % 4d' % self.score
            ]
        self._info_text += [
            'Traffic sign detected (stop): % 4d' % self.detectedTSDStop
            ]
        self._info_text += [
            'TSD (Speed Limited): % 4d' % self.detectedTSDSpeed
            ]
        self._info_text += [
            'TSD (Direct): % 4d' % self.detectedTSDDirect
            ]
        self._info_text += [
            'Traffic light detected: % 4d' % self.detectedTrafficLight
            ]
        self._info_text += [
            'Pedestrian detected: % 4d' % self.detectedPedestrian
            ]
        self._info_text += [
            'Car detected: % 4d' % self.detectedCar
            ]
        self._info_text += [
            'Weather detected: % 4d' % self.detectedWeather
            ]
        self._info_text += [
            'Door open detected: % 4d' % self.detectedDoor
            ]
    def toggle_info(self):
        self._show_info = not self._show_info

    def minus_score(self, score_minus):
        self.score = self.score - score_minus
        time.sleep(0.1)
        
    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        # self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        
        
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        # print(intensity)
        if (intensity > 500):
        # self.hud.score = self.hud.score - 1
            self.hud.minus_score(1)
            self.hud.collision+=1
            self.hud.notification('Collision with %r => score - 1' % actor_type)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.hud.invasion = self.hud.invasion + 1
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.score = self.hud.score - 1
        self.hud.notification('Crossed line => score - 1', seconds = 1)
        

# ==============================================================================
# -- ObstacleDetector --------------------------------------------------------
# ==============================================================================
        
class ObstacleDetector(object):
    def __init__(self, parent_actor, hud, _ros_connection):
        self.sensor = None
        self.ros_connection = _ros_connection
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.obstacle')
        bp.set_attribute("only_dynamics",str(True))
        bp.set_attribute("distance",str(20))
        # bp.set_attribute("debug_linetrace",str(True))
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: ObstacleDetector._on_detection(weak_self, event))

    @staticmethod
    def _on_detection(weak_self, event):
        self = weak_self()
        if not self:
            return
        obstacle_distance = event.distance
        obstacle_actor = event.other_actor
        # print("Detected obstacle at distance:", obstacle_distance, "meters")

        # Check if the detected actor is a vehicle or pedestrian
        if 'vehicle' in obstacle_actor.type_id:
            # print("Detected vehicle:", obstacle_actor.id)
            self.ros_connection.set_obstacle("vehicle", obstacle_distance)
        elif 'walker' in obstacle_actor.type_id:
            # print("Detected pedestrian:", obstacle_actor.id)
            self.ros_connection.set_obstacle("walker", obstacle_distance)
        # if not self:
        #     return
        # type_ids = set(x.type_id for x in event.other_actor)
        # text = ['%r' % str(x).split()[-1] for x in type_ids]
        self.hud.notification(get_actor_display_name(event.other_actor))

# class Obstacle_Sensor(object):
#     def __init__(self, parent_actor):
#         self.sensor = None
#         self._parent = parent_actor
#         world = self._parent.get_world()
#         bp = world.get_blueprint_library().find('sensor.other.obstacle')
#         self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
#         # We need to pass the lambda a weak reference to self to avoid circular
#         # reference.
#         weak_self = weakref.ref(self)
#         self.sensor.listen(lambda event: Obstacle_Sensor._on_obstacle_event(weak_self, event))

#     @staticmethod
#     def _on_obstacle_event(weak_self, event):
#         self = weak_self()
#         if not self:
#             return
#         obstacle_distance = event.distance
#         obstacle_actor = event.other_actor

#         print("Detected obstacle at distance:", obstacle_distance, "meters")

#         # Check if the detected actor is a vehicle or pedestrian
#         if 'vehicle' in obstacle_actor.type_id:
#             print("Detected vehicle:", obstacle_actor.id)
#         elif 'walker' in obstacle_actor.type_id:
#             print("Detected pedestrian:", obstacle_actor.id)

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data) # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, parent):
        # start_in_autopilot
        self._parent = parent
        self._steer_cache = 0.0
        self._control = carla.VehicleControl()
        # initialize steering wheel
        self.is_connect_with_VM = 0

        if (self.is_connect_with_VM):
            pygame.joystick.init()

            joystick_count = pygame.joystick.get_count()
            if joystick_count > 1:
                raise ValueError("Please Connect Just One Joystick")

            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()

            self._parser = ConfigParser()
            self._parser.read('wheel_config.ini')
            # print(self._parser)
            
            # self._steer_idx = int(
            #     self._parser.get('G29 Racing Wheel', 'steering_wheel'))
            # self._throttle_idx = int(
            #     self._parser.get('G29 Racing Wheel', 'throttle'))
            # self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
            # self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
            # self._handbrake_idx = int(
            #     self._parser.get('G29 Racing Wheel', 'handbrake'))
            
            # self._gear_1 = int(self._parser.get('G29 Racing Wheel', 'gear_1'))
            # self._gear_2 = int(self._parser.get('G29 Racing Wheel', 'gear_2'))
            # self._gear_3 = int(self._parser.get('G29 Racing Wheel', 'gear_3'))
            # self._gear_4 = int(self._parser.get('G29 Racing Wheel', 'gear_4'))
            # self._gear_5 = int(self._parser.get('G29 Racing Wheel', 'gear_5'))
            # self._gear_R = int(self._parser.get('G29 Racing Wheel', 'gear_R'))

            self._steer_idx = 0
            self._throttle_idx = 2
            self._brake_idx = 3
            self._reverse_idx = 5
            self._handbrake_idx = 4
            
            self._gear_1 = 12
            self._gear_2 = 13
            self._gear_3 = 14
            self._gear_4 = 15
            self._gear_5 = 16
            self._gear_R = 17

    def create_pedestrian(self, world, location, end_point):
        # Get the blueprint library
        blueprint_library = world.get_blueprint_library()

        # Choose a random pedestrian model from the blueprint library
        pedestrian_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))

        # Set the spawn location and rotation
        transform = carla.Transform(location, carla.Rotation(yaw=0))

        # Spawn the pedestrian
        pedestrian = world.spawn_actor(pedestrian_bp, transform)

        # Move the pedestrian along the defined path
        thread = threading.Thread(target=self.move_pedestrian_along_path, args=(pedestrian, location, end_point))

        # Start the thread
        thread.start()
        
    def move_pedestrian_along_path(self, pedestrian, start_point, end_point):
        # Move the pedestrian along the predefined path
        # print(pygame.time.Clock().tick())
        pedestrian.set_simulate_physics(True)
        pedestrian_control = carla.WalkerControl()
        pedestrian_control.speed = 1.4
        pedestrian_control.direction = (end_point - start_point).make_unit_vector()
        pedestrian.apply_control(pedestrian_control)


        while (pedestrian.get_location().distance(end_point) > 20.0):
            time.sleep(1)
        pedestrian_control1 = carla.WalkerControl()
        pedestrian_control1.speed = 0
        pedestrian.apply_control(pedestrian_control1)

        while (pedestrian.get_location().distance(self._parent.vehicle_controller.vehicle.get_transform().location) > 15):
            time.sleep(1)
        time.sleep(8)
        pedestrian.apply_control(pedestrian_control)
        time.sleep(15)
        pedestrian.destroy()

    def create_pedestria_1(self, world):
        # Define the start and end points of the crosswalk
        start_point = carla.Location(x=42, y=-11, z=1)  # Adjust the coordinates as needed
        end_point = carla.Location(x=42, y=13, z=0.5)  # Adjust the coordinates as needed

        # Create a path for the pedestrian
        path = [start_point, carla.Location(start_point.x, end_point.y, end_point.z), end_point]

        # Spawn the pedestrian at the start point of the crosswalk
        self.create_pedestrian(world, start_point, end_point)

    def create_car_1(self, world):
        # Define the start and end points of the crosswalk
        start_point = carla.Location(x=66.5, y=5.1, z=1)  # Adjust the coordinates as needed
        end_point = carla.Location(x=42, y=13, z=0.5)  # Adjust the coordinates as needed

        # blueprint = random.choice(world.get_blueprint_library().filter('vehicle.dodge.charger_2020'))

        blueprint = random.choice(world.get_blueprint_library().filter("vehicle.ford.ambulance"))
        if blueprint.has_attribute("driver_id"):
            driver_id = random.choice(
                blueprint.get_attribute("driver_id").recommended_values
            )
            blueprint.set_attribute("driver_id", driver_id)
        try:
            blueprint.set_attribute("role_name", "autopilot")
        except IndexError:
            pass
        vehicle_init_position  = [
            carla.Transform(start_point, carla.Rotation(yaw=-2))]

        self.vehicle_1 = world.spawn_actor(blueprint, vehicle_init_position[0])

        thread = threading.Thread(target=self.control_car_1)
        
        # Start the thread
        thread.start()

    def control_car_1(self):
        self.vehicle_1.apply_control(carla.VehicleControl(throttle=1, steer=0.0))
        time.sleep(4.6)
        self.vehicle_1.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(4)
        self.vehicle_1.apply_control(carla.VehicleControl(throttle=1, steer=0.0))
        time.sleep(10)
        self.vehicle_1.destroy()


    def parse_events(self, world, clock, player, world_carla):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                elif event.button == 2:
                    world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._gear_1:
                    self._parent.gear = 1
                elif event.button == self._gear_2:
                    self._parent.gear = 2
                elif event.button == self._gear_3:
                    self._parent.gear = 3
                elif event.button == self._gear_4:
                    self._parent.gear = 4
                elif event.button == self._gear_5:
                    self._parent.gear = 5
                elif event.button == 7:
                    self._parent.release_control = 1
                elif event.button == 6:
                    self._parent.release_control = 0
                elif event.button == self._gear_R:
                    self._parent.gear = -1
                elif event.button == self._reverse_idx:
                    # self._control.gear = 1 if self._control.reverse else -1
                    self._parent._reverse = 0 if self._parent._reverse else 1
                elif event.button == 23:
                    world.camera_manager.next_sensor()
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == self._gear_1:
                    self._parent.gear = 0
                elif event.button == self._gear_2:
                    self._parent.gear = 0
                elif event.button == self._gear_3:
                    self._parent.gear = 0
                elif event.button == self._gear_4:
                    self._parent.gear = 0
                elif event.button == self._gear_5:
                    self._parent.gear = 0
                elif event.button == self._gear_R:
                    self._parent.gear = 0
            elif event.type == pygame.KEYUP:
                if event.key == K_BACKSPACE:
                    return True
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_g:
                    # self._parent.vehicle_controller.set_door(1)
                    self.pub_obstacle_distance = rospy.Publisher("/carla/hero/vehicle_toggle_FL_door", Int32, queue_size=10)
                    self.pub_obstacle_distance.publish(0)
                elif event.key == K_h:
                    self._parent.vehicle_controller.set_door(0)
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_q:
                    self._parent.gear = 1 if self._parent._reverse else -1
                elif event.key == K_p:
                    # self._parent.move_to_parking_slot()
                    # self._parent.move_to_parking_slot()
                    # spawn_points = world.get_map().get_spawn_points()
                    # blueprint = random.choice(world_carla.get_blueprint_library().filter('vehicle.dodge.charger_2020'))
                    # # blueprints = world.get_blueprint_library().filter("vehicle.*")
                    # if blueprint.has_attribute("driver_id"):
                    #     driver_id = random.choice(
                    #         blueprint.get_attribute("driver_id").recommended_values
                    #     )
                    #     blueprint.set_attribute("driver_id", driver_id)
                    # try:
                    #     blueprint.set_attribute("role_name", "autopilot")
                    # except IndexError:
                    #     pass
                    # vehicle_init_position  = [
                    #     carla.Transform(carla.Location(x=-265, y=-21, z=0.05), carla.Rotation(yaw=270))]

                    # vehicle = world_carla.spawn_actor(blueprint, vehicle_init_position[0])

                    # vehicle.set_autopilot(True)
                    # vehicle.enable_constant_velocity(carla.Vector3D(2, 0, 0))
                    self.create_car_1(world_carla)
                elif event.key == K_o:
                    # self._parent.is_parking = 1 if self._parent.is_parking == 0 else 0
                    # self._parent.state = 0 if self._parent.state == -1 else -1
                    self.create_pedestria_1(world_carla)
                elif event.key == K_r:
                    self._parent.release_control = 1
                elif event.key == K_t:
                    self._parent.release_control = 0
                elif event.key == K_m:
                    self._parent.manual_gear_shift = not self._parent.manual_gear_shift
                    self._parent.gear = player.get_control().gear
                    print(self._parent.manual_gear_shift)
                elif self._parent.manual_gear_shift and event.key == K_COMMA:
                    self._parent.gear = max(-1, self._parent.gear - 1)
                    print(self._parent.gear)
                elif self._parent.manual_gear_shift and event.key == K_PERIOD:
                    self._parent.gear = self._parent.gear + 1
                #     elif event.key == K_p:
                #         self._autopilot_enabled = not self._autopilot_enabled
                #         world.player.set_autopilot(self._autopilot_enabled)
                #         world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

            self._parent._reverse = self._parent.gear < 0
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            if (self.is_connect_with_VM):
                self._parse_vehicle_wheel()

    def _parse_vehicle_keys(self, keys, milliseconds):
        
        if keys[K_UP] or keys[K_w]:
            self._parent._throttle = min(self._parent._throttle + 0.01, 1.00)
        else:
            self._parent._throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._parent._brake = min(self._parent._brake + 0.2, 1)
        else:
            self._parent._brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._parent._steer = round(self._steer_cache, 1)
        self._parent._hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        # steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])
        steerCmd = K1 * math.tan(0.8 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._parent._steer = steerCmd
        self._parent._brake = brakeCmd
        self._parent._throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        self._parent._hand_brake = bool(jsButtons[self._handbrake_idx])

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)