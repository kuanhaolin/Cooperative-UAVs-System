import sys
import time
import airsim
import pygame
#import cv2
#import numpy as np

#pygame settings
pygame.init()
screen = pygame.display.set_mode((320, 240))
pygame.display.set_caption('keyboard ctrl')
screen.fill((0, 0, 0))

#AirSim settings
#更改控制無人機的設定(settings.json)
vehicle_name = "Drone0"
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name=vehicle_name)
AirSim_client.armDisarm(True, vehicle_name=vehicle_name)
AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

#速度
vehicle_velocity = 2.0
#加速度
speedup_ratio = 10.0
#臨時速度
speedup_flag = False
#偏航速率
vehicle_yaw_rate = 10.0

while True:
        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0
        time.sleep(0.02)

        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()

        scan_wrapper = pygame.key.get_pressed()

        #空格加速10倍
        if scan_wrapper[pygame.K_SPACE]:
                scale_ratio = speedup_ratio
        else:
                scale_ratio = speedup_ratio / 5

        #A和D鍵控制偏航速率
        if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
                yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate

        #UP和DOWN鍵控制pitch軸速度(機翼為軸)
        if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
                velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio

        #LEFT和RIGHT鍵控制roll軸速度(機身為軸)
        if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
                velocity_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio

        #W和S鍵控z軸速度
        if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
                velocity_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio

        # print(f": Expectation gesture: {velocity_x}, {velocity_y}, {velocity_z}, {yaw_rate}")

        #設置速度控制及偏航控制
        AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=0.02, yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate), vehicle_name=vehicle_name)

        if scan_wrapper[pygame.K_ESCAPE]:
                pygame.quit()
                sys.exit()