import sys
import airsim
import pygame
import asyncio
import csv
#import numpy as np
#import time

#pygame setting
pygame.init()
screen = pygame.display.set_mode((320, 240))
pygame.display.set_caption('keyboard ctrl')
screen.fill((0, 0, 0))

#airsim setting
vehicle_name = "Drone0"
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name=vehicle_name)
AirSim_client.armDisarm(True, vehicle_name=vehicle_name)
AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

vehicle_velocity = 2.0
speedup_ratio = 4.0
speedup_flag = False
vehicle_yaw_rate = 5.0

#紀錄位置
trajectory = []
last_position = None

with open('waypoints.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['x', 'y', 'z'])
    #標題

async def draw_trajectory():
    while True:
        if len(trajectory) > 1:
            AirSim_client.simPlotLineStrip(trajectory, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=10.0, is_persistent=True)
        await asyncio.sleep(1.2)

async def main():
    global last_position

    while True:
        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        scan_wrapper = pygame.key.get_pressed()

        if scan_wrapper[pygame.K_SPACE]:
            scale_ratio = speedup_ratio
        else:
            scale_ratio = 1.0

        if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
            yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate

        if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
            velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio * vehicle_velocity

        if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
            velocity_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio * vehicle_velocity

        if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
            velocity_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio * vehicle_velocity

        AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=1,
                                                   yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate), vehicle_name=vehicle_name)

        #獲取位置
        position = AirSim_client.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated.position
        current_position = airsim.Vector3r(position.x_val, position.y_val, position.z_val)

        #位置發生變化紀錄
        if last_position is None or (abs(current_position.x_val - last_position.x_val) > 0.1 or
                                     abs(current_position.y_val - last_position.y_val) > 0.1 or
                                     abs(current_position.z_val - last_position.z_val) > 0.1):
            trajectory.append(current_position)
            last_position = current_position

            with open('waypoints.csv', mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([current_position.x_val, current_position.y_val, current_position.z_val])

        if scan_wrapper[pygame.K_ESCAPE]:
            pygame.quit()
            sys.exit()

        await asyncio.sleep(0.4)

loop = asyncio.get_event_loop()
loop.create_task(draw_trajectory())
loop.run_until_complete(main())