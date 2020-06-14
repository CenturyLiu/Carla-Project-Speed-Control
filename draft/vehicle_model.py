#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 24 16:57:42 2020

@author: shijiliu
"""


import control
import carla
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from carla_env import CARLA_ENV
import time


def generate_throttle_signal(frequency,sampling_period, sim_time):
    '''
    Effects:
        Create a signal within [0,1] to be used as the throttle_signal  

    Parameters
    ----------
    frequency : int TYPE
        DESCRIPTION: The frequency of the signal
        
    sampling_period : float TYPE
        DESCRIPTION: The period we sample the control signal
                     This should match the timestep for carla simulation
    sim_time : int TYPE
        DESCRIPTION: Total time for simulation
                   

    Returns
    -------
    TYPE
        DESCRIPTION: return the discretized version of the throttle control signal

    '''
    frequency = int(frequency)
    sim_time = int(sim_time)
    sample_number = int(sim_time / sampling_period)
    sampling_time = np.linspace (0, sim_time, sample_number)
    
    amplitude = 0.5
    phase = 0
    return amplitude * (np.sin (2 * np.pi * frequency * sampling_time + phase) + 1)
    
    
    
def get_frequency_response_data(env,frequency,sim_time):
    '''
    

    Parameters
    ----------
    env : TYPE
        DESCRIPTION.
    frequency : float TYPE
        DESCRIPTION.
    sim_time : int TYPE
        DESCRIPTION.

    Returns
    -------
    throttle_signal : TYPE
        DESCRIPTION.
    forward_speed : TYPE
        DESCRIPTION.

    '''
    sim_time = int(sim_time)
    throttle_signal = generate_throttle_signal(frequency,env.delta_seconds,10) #simulate for 10 seconds, 500 data points
    spawn_point = carla.Transform(carla.Location(x=6.078289, y=-160, z=1.843106), carla.Rotation(pitch=0.000000, yaw=88.876099, roll=0.000000))
    model_name = "model3"
    model_unique_name = env.spawn_vehicle(model_name,spawn_point)
    forward_speed = []
    
    end_t = sim_time / env.delta_seconds
    
    count = 0
    while True:
        env.world.tick()# use sychronous model with fixed timestep
        
        vehicle_control = carla.VehicleControl(throttle = throttle_signal[count],steer=0.0)
        env.apply_vehicle_control(model_unique_name, vehicle_control)
        speed = env.get_forward_speed(model_unique_name)
        forward_speed.append(speed)
        count += 1
        if count == end_t:
            break
    env.destroy_actors()
    return throttle_signal , forward_speed

def speed_control(env, sys, ref_speeds, curr_speeds, init_values):
    '''
    

    Parameters
    ----------
    env : CARLA_ENV
        DESCRIPTION.
    sys : control.ss 
        speed controller
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
    _,y0,x0 = control.forced_response(sys,U = U0,X0 = init_values[0])
    init_values.append(x0[-1])
    throttle = y0[-1]
    return throttle, init_values
    
def speed_control_wrapper(env, sim_time):
    sim_time = int(sim_time)
    #spawn_point = carla.Transform(carla.Location(x=6.078289, y=-160, z=1.843106), carla.Rotation(pitch=0.000000, yaw=88.876099, roll=0.000000))
    
    #spawn_point = carla.Transform(carla.Location(x= 79, y=-4.89, z=4.93), carla.Rotation(pitch=-5.5, yaw= -87, roll=0.00))#carla.Transform(carla.Location(x=-277.08, y=-15.39, z=4.94), carla.Rotation(pitch=0.000000, yaw= 0, roll=0.000000))#carla.Transform(carla.Location(x=387.10, y=330.08, z=8.34), carla.Rotation(pitch=0.000000, yaw=180, roll=0.000000))
    spawn_point = carla.Transform(carla.Location(x=-277.08, y=-15.39, z=4.94), carla.Rotation(pitch=0.000000, yaw= 0, roll=0.000000))
    model_name = "vehicle.tesla.model3"
    model_unique_name = env.spawn_vehicle(model_name,spawn_point)
    end_t = sim_time / env.delta_seconds
    
    KI = 0.01
    KP = 0.5
    num_pi = [KP, KI]
    den_pi = [1.0, 0.01*KI/KP]

    sys = control.tf(num_pi,den_pi)
    sys = control.sample_system(sys, env.delta_seconds)
    sys = control.tf2ss(sys)
    
    count = 0
    speed = []
    throttles = []
    reference_speed = []
    
    
    init_values = deque(maxlen = 2)
    ref_speeds = deque(maxlen = 2)
    curr_speeds = deque(maxlen = 2)
    init_values.append(0)
    ref_speeds.append(0)
    curr_speeds.append(0)
    
    #env.config_env(synchrony = True)
    
    while True:
        env.world.tick()
        curr_speed = env.get_forward_speed(model_unique_name)
        print('curr_speed == ', curr_speed)
        speed.append(curr_speed)
        if count == end_t:
            break
        if count <= 50:
            reference_speed.append(0)
            ref_speeds.append(0)
            curr_speeds.append(curr_speed)
        if count > 50 and count < 350:
            reference_speed.append(20)
            ref_speeds.append(20)
            curr_speeds.append(curr_speed)
        if count >= 350 and count <= 600:
            reference_speed.append(10)
            ref_speeds.append(10)
            curr_speeds.append(curr_speed)
        if count > 600:
            reference_speed.append(0)
            ref_speeds.append(0)
            curr_speeds.append(curr_speed)
        throttle, init_values = speed_control(env, sys, ref_speeds, curr_speeds, init_values)
        throttle = np.clip(throttle,0,1)
        throttles.append(throttle)
        
        #vehicle_control = carla.VehicleControl(throttle = throttle,steer=0.0)
        if curr_speed <= reference_speed[-1]:
            vehicle_control = carla.VehicleControl(throttle = throttle,steer=0.0)
        else:
            vehicle_control = carla.VehicleControl(throttle = throttle, steer = 0.0, brake = 0.5)
        
        env.apply_vehicle_control(model_unique_name, vehicle_control)
        count += 1
    
    return throttles, speed, reference_speed

throttle_signal = [] 
forward_speed = []

client = carla.Client("localhost",2000)
client.set_timeout(10.0)
world = client.load_world('Town06')
time.sleep(5)
#world = client.get_world()
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(carla.Location(x=-68.29, y=151.75, z=170.8), carla.Rotation(pitch=-31.07, yaw= -90.868, roll=1.595))) # plain ground
#spectator.set_transform(carla.Transform(carla.Location(x=79.95, y=-13.13, z=84.69), carla.Rotation(pitch=-47.2, yaw= -90.86, roll=0.000000))) #slope
'''
weather = carla.WeatherParameters(
    cloudiness=10.0,
    precipitation=0.0,
    sun_altitude_angle=90.0)
world.set_weather(weather)
'''
env = CARLA_ENV(world)
time.sleep(2)
try:
    '''
    throttle_signal , forward_speed = get_frequency_response_data(env,10,10)
    plt.subplot(2,1,1)
    plt.plot(throttle_signal)
    plt.subplot(2,1,2)
    plt.plot(forward_speed)
    '''
    throttles, speed, reference_speed = speed_control_wrapper(env, 20)
    
    fig,a =  plt.subplots(3,1)
    
    #plt.subplot(3,1,1)
    a[0].plot(reference_speed)
    a[0].set_title('reference speed')
    #plt.subplot(3,1,2)
    a[1].plot(throttles)
    a[1].set_title('throttle applied')
    a[2].plot(speed)
    a[2].set_title('measured speed')
    
    
finally:
    env.destroy_actors()
    