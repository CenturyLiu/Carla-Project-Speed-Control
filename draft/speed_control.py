#!/usr/bin/env python

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random
#import cv2
import numpy as np

'''
IM_WIDTH = 640
IM_HEIGHT = 480

def process_img(image):
    i = np.array(image.raw_data)
    #print(dir(image))
    i2 = i.reshape((IM_HEIGHT,IM_WIDTH,4))#rgba
    i3 = i2[:,:,0:3]
    cv2.imshow("img.jpg",i3)
    cv2.waitKey(1)
    print(i3)
    return i3/255.0
'''

actor_list = []



try:
    client = carla.Client("localhost",2000)
    client.set_timeout(2.0)

    world = client.get_world()
    weather = carla.WeatherParameters(cloudiness=20.0,precipitation=0.0,wind_intensity = 50,  sun_azimuth_angle=90.0, sun_altitude_angle=45.000000)
    world.set_weather(weather)
    blueprint_library = world.get_blueprint_library()



    #get a car
    bp = blueprint_library.filter("model3")[0] #a Tesla model

    spawn_point = carla.Transform(carla.Location(x=6.078289, y=-50, z=1.843106), carla.Rotation(pitch=0.000000, yaw=-88.876099, roll=0.000000))#random.choice(world.get_map().get_spawn_points())

    tesla_vehicle = world.spawn_actor(bp,spawn_point)
    #vehicle.set_autopilot(True)

    tesla_vehicle.apply_control(carla.VehicleControl(throttle=1.0,steer=0.0))
    actor_list.append(tesla_vehicle)

    count = 0
    while True:
        timestamp = world.wait_for_tick()
        vehicles = world.get_actors().filter('vehicle.*')
        for vehicle in vehicles:
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            control = vehicle.get_control()
            print('forward_speed == %f, vx == %f,vy == %f,vz == %f,throttle == %f,steer == %f,brake == %f,hand_brake == %r,reverse == %r,manual_gear_shift == %r,gear == %d\n' % ((velocity.x**2+velocity.y**2+velocity.z**2)**0.5,velocity.x, velocity.y, velocity.z, control.throttle, control.steer, control.brake, control.hand_brake, control.reverse, control.manual_gear_shift, control.gear))
        count += 1
        if count == 100:
            tesla_vehicle.apply_control(carla.VehicleControl(throttle=0.2,steer=0.0))

        if count == 500:
            break

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
