#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul  5 10:20:35 2020

@author: shijiliu
"""


import carla
from carla_env import CARLA_ENV 
import math
import time

DEBUG_INIT = True

# color for debug use
red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)


# distance of lane points from traffic light
END1 = -5.5
END2 = -9.0
START1 = -12.5
START2 = -16.0


def get_traffic_lights(actor_list):
    # get all the available traffic lights
    traffic_light_list = []
    for actor in actor_list:
        if 'traffic_light' in actor.type_id:
            traffic_light_list.append(actor)
    return traffic_light_list

class Intersection():
    def __init__(self, env, world_pos, traffic_light_list, distance = 75.0, yaw = 0.0):
        '''
        

        Parameters
        ----------
        env: CARLA_ENV
            the simulation environment
        world_pos : (float,float)
            the (rough) central point of the intersection.
        traffic_light_list : list
            list of all available traffic lights.
        distance : float, optional
            width and height of the intersection. The default is 75.0 (m).
        yaw : float, optional
            define the direction the ego vehicle will pass through the intersection. The default is 0.

        Returns
        -------
        None.

        '''
        
        self.env = env
        self.distance = distance
        self.get_local_traffic_lights(world_pos,traffic_light_list)
        self.get_lane_points()
        
    def get_local_traffic_lights(self, world_pos,traffic_light_list):
        '''
        

        Parameters
        ----------
        world_pos : (float,float)
            the (rough) central point of the intersection.
        traffic_light_list : list
            list of all available traffic lights.

        Returns
        -------
        None.

        '''
        self.local_traffic_lights = []
        for light in traffic_light_list:
            location = light.get_location()
            distance = math.sqrt((location.x - world_pos[0])**2 + (location.y - world_pos[1]) ** 2) # get the 2d Euclidean distance
            if distance < self.distance / 2:
                self.local_traffic_lights.append(light)
                
        assert(len(self.local_traffic_lights) == 4) # should contain and only contain 4 lights
        
        x = 0
        y = 0
        for light in self.local_traffic_lights:
            x += light.get_location().x
            y += light.get_location().y
        
        self.world_pos = (x / len(self.local_traffic_lights),y / len(self.local_traffic_lights)) 
        
        if DEBUG_INIT:
            print(self.world_pos)
            for light in self.local_traffic_lights:
                print(light.get_location())
                self.env.world.debug.draw_point(light.get_location(),size = 0.5, color = blue, life_time=0.0, persistent_lines=True)
                

    def get_lane_points(self):
        self.carla_map = self.env.world.get_map()
        self.out_lane_points = []
        self.into_lane_points = []
        for light in self.local_traffic_lights:
            light_location = light.get_location()
            vector = light.get_transform().get_forward_vector()
            end_1 = carla.Location(x = light_location.x + vector.x * END1,y = light_location.y + vector.y * END1, z = light_location.z + vector.z * END1) 
            end_2 = carla.Location(x = light_location.x + vector.x * END2,y = light_location.y + vector.y * END2, z = light_location.z + vector.z * END2)
            start_1 = carla.Location(x = light_location.x + vector.x * START1,y = light_location.y + vector.y * START1, z = light_location.z + vector.z * START1)
            start_2 = carla.Location(x = light_location.x + vector.x * START2,y = light_location.y + vector.y * START2, z = light_location.z + vector.z * START2)
            into_1 = self.carla_map.get_waypoint(end_1)
            into_2 = self.carla_map.get_waypoint(end_2)
            out_1 = self.carla_map.get_waypoint(start_1)
            out_2 = self.carla_map.get_waypoint(start_2)
            self.out_lane_points.append(out_1)
            self.out_lane_points.append(out_2)
            self.into_lane_points.append(into_1)
            self.into_lane_points.append(into_2)
        
        if DEBUG_INIT:
            for pt in self.out_lane_points:
                self.env.world.debug.draw_point(pt.transform.location,size = 0.1, color = green, life_time=0.0, persistent_lines=True)
                forward_vector = pt.transform.get_forward_vector()
                start = pt.transform.location
                end = carla.Location(x = start.x + forward_vector.x, y = start.y + forward_vector.y, z = start.z + forward_vector.z)
                self.env.world.debug.draw_arrow(start,end,thickness=0.1, arrow_size=0.2, color = green, life_time=0.0, persistent_lines=True)
                
            for pt in self.into_lane_points:
                self.env.world.debug.draw_point(pt.transform.location,size = 0.1, color = red, life_time=0.0, persistent_lines=True)
                forward_vector = pt.transform.get_forward_vector()
                start = pt.transform.location
                end = carla.Location(x = start.x + forward_vector.x, y = start.y + forward_vector.y, z = start.z + forward_vector.z)
                self.env.world.debug.draw_arrow(start,end,thickness=0.1, arrow_size=0.2, color = red, life_time=0.0, persistent_lines=True)
                
                
                
                
client = carla.Client("localhost",2000)
client.set_timeout(10.0)
world = client.load_world('Town05')
 
# set the weather
weather = carla.WeatherParameters(
    cloudiness=10.0,
    precipitation=0.0,
    sun_altitude_angle=90.0)
world.set_weather(weather)

# set the spectator position for demo purpose
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(carla.Location(x=0.0, y=0.0, z=20.0), carla.Rotation(pitch=-31.07, yaw= -90.868, roll=1.595))) # plain ground

env = CARLA_ENV(world) 
time.sleep(2) # sleep for 2 seconds, wait the initialization to finish

world_pos = (25.4,0.0)
traffic_light_list = get_traffic_lights(world.get_actors())
intersection1 = Intersection(env, world_pos, traffic_light_list)