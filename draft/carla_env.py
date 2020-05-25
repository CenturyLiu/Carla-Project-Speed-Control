#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 24 13:45:54 2020

@author: shijiliu
"""
import glob
import os
import sys
import time
import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



import carla
import matplotlib.pyplot as plt
import numpy as np
from collections import deque


class CARLA_ENV():
    def __init__(self,client):
        #self.client = carla.Client('localhost', 2000)
        #self.client.set_timeout(10.0)
        self.client = client
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_dict = {}
        self.walker_dict = {}
        self.sensor_dict = {}
        self.config_env()
        
        
    def config_env(self, synchrony = True, delta_seconds = 0.02):
        '''
        Effects
        -------
        Config the carla world's synchrony and time-step
        tutorial: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/
        
        Parameters
        ----------
        synchrony : TYPE, optional
            DESCRIPTION. The default is True.
        delta_seconds : TYPE, optional
            DESCRIPTION. The default is 0.02.

        Returns
        -------
        None.

        '''
        self.synchrony = synchrony
        self.delta_seconds = delta_seconds
        settings = self.world.get_settings()
        settings.synchronous_mode = synchrony
        settings.fixed_delta_seconds = delta_seconds
        self.world.apply_settings(settings)
        
    def spawn_vehicle(self, model_name = None, spawn_point = None):
        '''
        Parameters
        ----------
        model_name : str TYPE, optional
            DESCRIPTION:  The default is None.
        spawn_point : carla.Transform() TYPE, optional
            DESCRIPTION. The default is None.

        Returns
        -------
        Uniquename of the actor.

        '''
        if model_name == None:
            bp = random.choice(self.blueprint_library.filter('vehicle.*.*'))
        else:
            bp = random.choice(self.blueprint_library.filter(model_name))
        
        if spawn_point == None:
            spawn_point = random.choice(self.world.get_map().get_spawn_points())
        
        vehicle = self.world.spawn_actor(bp,spawn_point)
        self.vehicle_dict[vehicle.type_id + '_' + str(vehicle.id)] = vehicle
        return vehicle.type_id + '_' + str(vehicle.id)

    def destroy_actors(self):
        '''
        Effects
        -------
        Destroy all actors that have been spawned

        Returns
        -------
        None.

        '''
        for index in self.vehicle_dict.keys():
            self.vehicle_dict[index].destroy()
        for index in self.walker_dict.keys():
            self.walker_dict[index].destroy()
        for index in self.sensor_dict.keys():
            self.sensor_dict[index].destroy()
            
        self.vehicle_dict.clear()
        self.walker_dict.clear()
        self.sensor_dict.clear()
        print("destroyed all actors")
        
    def apply_vehicle_control(self, uniquename, vehicle_control):
        '''
        Effects: apply control to a specific vehicle

        Parameters
        ----------
        uniquename : str TYPE
            DESCRIPTION.
        vehicle_control : vehicle control TYPE, https://carla.readthedocs.io/en/latest/python_api/#carla.Vehicle
            DESCRIPTION.

        Returns
        -------
        None.

        '''
        vehicle = self.vehicle_dict[uniquename]
        vehicle.apply_control(vehicle_control)
        
    def get_forward_speed(self, uniquename):
        '''
        Get the forward speed of the vehicle

        Parameters
        ----------
        uniquename : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        '''
        vehicle = self.vehicle_dict[uniquename]
        velocity = vehicle.get_velocity()
        return (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)**0.5
    
    


client = carla.Client("localhost",2000)
client.set_timeout(2.0)
env = CARLA_ENV(client)

try:
    env.spawn_vehicle()
finally:
    env.destroy_actors()