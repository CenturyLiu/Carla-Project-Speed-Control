#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 16:19:17 2020

@author: shijiliu
"""


import carla
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import time

import control # the python-control package, install first
from carla_env import CARLA_ENV # self-written class that provides help functions, should be in the same folder
import random

import copy

# PI controller constants
KI = 0.01
KP = 0.5

def get_PI_controller(delta_seconds):
    '''
    Effects: create a discrete state-space PI controller
    '''
    num_pi = [KP, KI] # numerator of the PI transfer function (KP*s + KI)
    den_pi = [1.0, 0.01*KI/KP] # denominator of PI transfer function (s + 0.01*KI/KP)

    sys = control.tf(num_pi,den_pi) # get transfer function for PI controller (since the denominator has a small term 0.01*KI/KP, it is actually a lag-compensator)
    sys = control.sample_system(sys, delta_seconds) # discretize the transfer function (from s-domain which is continuous to z-domain)
                                                        #since our simulation is discrete
    sys = control.tf2ss(sys) # transform transfer function into state space.
    return sys

def speed_control(sys, ref_speeds, curr_speeds, init_values):
    '''
    Effects: get the reference speed, current (measured) speed and initial values
             Use the difference 
                               e = ref_speeds - curr_speeds 
             as the input for the PI controller, derive the new throttle

    Parameters
    ----------
    sys : control.ss 
        state space controller 
    ref_speeds : list of float
        the desired speed we need
    curr_speeds : list of float
        the current speed
    init_values : the initial_values of the system
        DESCRIPTION.

    Returns
    -------
    throttle : float type
        DESCRIPTION.

    '''
    U0 = np.array(ref_speeds) - np.array(curr_speeds)
    #print(U0)
    _,y0,x0 = control.forced_response(sys,U = U0,X0 = init_values[0]) # y0 is the next values, x0 is the state evolution
                                                                      # see https://python-control.readthedocs.io/en/0.8.3/generated/control.forced_response.html#control.forced_response 
    init_values.append(x0[-1])
    throttle = y0[-1]
    return throttle, init_values

def throttle_brake_control(env, sys, speed_timeline, sim_time, model_uniquename):
    '''
    Effects: this is the function where the whole control system in part 2 is implemented. 
    -------
    
    Parameters:
    -----------
    env: CARLA_ENV
    
    speed_timeline: a list of tuples of (time (unit: s), speed (unit: m/s)).
        Example: [(1,25),(10,10),(20,15)] 
                 The car will start accelerate from 0 to 25 m/s (90 km/h) at t = 1
                 And decelerate from 25 m/s to 10 m/s at t = 10
                 And accelerate again from 10 m/s to 15m/s at t = 15
                 
    sim_time: Total time for simulation. Its value should be larger than the last time in the speed_timeline list
    
    model_uniquename: str type
        The uniquename the vehicle, which was assigned when an actor is spawned
       
    '''
    
    vehicle_distance = 20
    init_d = 20
    
    curr_transform = env.get_transform_2d(model_uniquename)
    curr_pos = curr_transform[0]
    target_pos = curr_pos
    target_pos[0] += init_d
    
    # essential storages for the controller to work
    init_values = deque(maxlen = 2) # the state space values of the system. For a control system to be fully functional
                                    # we need to give initial value
    ref_speeds = deque(maxlen = 2) # the reference / target speed
    curr_speeds = deque(maxlen = 2) # the measured speed of the vehicle
    
    # storage for distance controller
    distance_init_values = deque(maxlen = 2)
    ref_distance = deque(maxlen = 2)
    curr_distance = deque(maxlen = 2)
    
    
    # storage for visualizing the reference speed, throttle and measured speed.
    speed = []
    throttles = []
    reference_speed = []
    
    # storage for visualizing the reference distance and measured distance
    reference_distances = []
    current_distances = []
    
    
    # give initial values to storage, assume the car is released at rest, with no initial speed or acceleration
    init_values.append(0) 
    ref_speeds.append(0)
    curr_speeds.append(0)
    
    distance_init_values.append(0)
    ref_distance.append(vehicle_distance)
    curr_distance.append(init_d)
    
    
    count = 0 #timestep count
    end_t = int(sim_time / env.delta_seconds) #the time at which the simulation ends
    
    target_speed = np.zeros(end_t)
    target_speed[:int(end_t/4)] = 10
    target_speed[int(end_t/4):int(end_t/2)] = 20
    target_speed[int(end_t/2):int(end_t*3/4)] = 5
    target_speed[int(end_t*3/4):] = 15
    
    current_ref_speed = 0
    
    distance_sys = get_distance_controller(env.delta_seconds)
    
    while True: #loop for applying control
        env.world.tick()
        curr_speed = env.get_forward_speed(model_uniquename)
        curr_transform = env.get_transform_2d(model_uniquename)
        curr_pos = curr_transform[0]
        speed.append(curr_speed)
        
        if count >= end_t:
            break
        
        target_pos[0] += target_speed[count] * env.delta_seconds
        
        
        ref_distance.append(vehicle_distance)
        reference_distances.append(vehicle_distance)
        curr_distance.append(target_pos[0] - curr_pos[0])
        current_distances.append(target_pos[0] - curr_pos[0])
        
        if count == 1:
            print(target_pos[0] - curr_pos[0])
        
        current_ref_speed,  distance_init_values = get_reference_speed(distance_sys,ref_distance, curr_distance, distance_init_values)
        
        '''
        if timeline_count < len(timeline):
            if count >= timeline[timeline_count][0]:  # need to get a new reference speed
                current_ref_speed = timeline[timeline_count][1]
                timeline_count += 1
        '''
        
        reference_speed.append(current_ref_speed) #for visualization
        ref_speeds.append(current_ref_speed) #for control
        curr_speeds.append(curr_speed) #for control
        
        throttle, init_values = speed_control(sys, ref_speeds, curr_speeds, init_values) # get the throttle control based on reference and current speed
        throttle = np.clip(throttle,0,1) # throttle value is [0,1]
        throttles.append(throttle) # for visualization
        
        if curr_speed <= current_ref_speed:
            vehicle_control = carla.VehicleControl(throttle = throttle,steer=0.0) 
        else:
            vehicle_control = carla.VehicleControl(throttle = throttle,steer=0.0,brake = 0.5)
        env.apply_vehicle_control(model_uniquename, vehicle_control) # apply control to vehicle
        
        count += 1
        # end loop
        
        
    return throttles, speed, reference_speed, current_distances, reference_distances

def get_distance_controller(delta_seconds):
    '''
    Effects: create a discrete state-space PI controller
    '''
    KP_1 = 1.0
    KI_1 = 1.0
    num_pi = [-KP_1, -KI_1] # numerator of the PI transfer function (KP*s + KI)
    den_pi = [1.0, 0.01*KI_1/KP_1] # denominator of PI transfer function (s + 0.01*KI/KP)

    sys = control.tf(num_pi,den_pi) # get transfer function for PI controller (since the denominator has a small term 0.01*KI/KP, it is actually a lag-compensator)
    sys = control.sample_system(sys, delta_seconds) # discretize the transfer function (from s-domain which is continuous to z-domain)
                                                        #since our simulation is discrete
    sys = control.tf2ss(sys) # transform transfer function into state space.
    
    
    return sys


def get_reference_speed(distance_sys,ref_distance, curr_distance, init_values):
    '''
    

    Parameters
    ----------
    distance_sys : control.ss 
        state space controller.
    ref_distance : float
        the reference distance.
    curr_distance : float
        current distance between two vehicles.
    init_values : the initial_values of the system

    Returns
    -------
    None.

    '''
    U0 = np.array(ref_distance) - np.array(curr_distance)
    print(U0)
    _,y0,x0 = control.forced_response(distance_sys,U = U0,X0 = init_values[0]) # y0 is the next values, x0 is the state evolution
                                                                      # see https://python-control.readthedocs.io/en/0.8.3/generated/control.forced_response.html#control.forced_response 
    init_values.append(x0[-1])
    ref_speed = y0[-1]
    return ref_speed, init_values

client = carla.Client("localhost",2000)
client.set_timeout(10.0)
world = client.load_world('Town06')
weather = carla.WeatherParameters(
    cloudiness=10.0,
    precipitation=0.0,
    sun_altitude_angle=90.0)
world.set_weather(weather)
# set the spectator position for demo purpose
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(carla.Location(x=-68.29, y=151.75, z=170.8), carla.Rotation(pitch=-31.07, yaw= -90.868, roll=1.595))) # plain ground

env = CARLA_ENV(world) 
time.sleep(2) # sleep for 2 seconds, wait the initialization to finish

# spawn a vehicle, here I choose a Tesla model
spawn_point = carla.Transform(carla.Location(x=-277.08, y=-15.39, z=4.94), carla.Rotation(pitch=0.000000, yaw= 0, roll=0.000000))
model_name = "vehicle.tesla.model3"
model_uniquename = env.spawn_vehicle(model_name,spawn_point) # spawn the model and get the uniquename, the CARLA_ENV class will store the vehicle into vehicle actor list

#create timeline and sim_time
sim_time = 20
speed_timeline = [(1,25),(7,10),(12,15)]
sys = get_PI_controller(env.delta_seconds)
time.sleep(2)

try:

    throttles, speed, reference_speed, curr_distance, reference_distances= throttle_brake_control(env, sys,speed_timeline, sim_time, model_uniquename)
    
    fig,a =  plt.subplots(5,1)
    
    #plt.subplot(3,1,1)
    a[0].plot(reference_speed)
    a[0].set_title('reference speed')
    #plt.subplot(3,1,2)
    a[1].plot(throttles)
    a[1].set_title('throttle applied')
    a[2].plot(speed)
    a[2].set_title('measured speed')
    a[3].plot(curr_distance)
    a[3].set_title('measured distance')
    a[4].plot(reference_distances)
    a[4].set_title('reference distance')
    
    
finally:
    env.destroy_actors()