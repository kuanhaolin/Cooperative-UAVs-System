import sys
import time
import airsim
import pygame
import cv2
#import numpy as np

pygame.init()

screen = pygame.display.set_mode((800, 144))
pygame.display.set_caption('screen')
screen.fill((0, 0, 0))

#airsim setting
vehicle_name = "Drone0"
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name=vehicle_name)
AirSim_client.armDisarm(True, vehicle_name=vehicle_name)
AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()
image_types = {
    "scene": airsim.ImageType.Scene,
    "depth": airsim.ImageType.DepthVis,
    "seg": airsim.ImageType.Segmentation,
    "normals": airsim.ImageType.SurfaceNormals,
    "segmentation": airsim.ImageType.Segmentation,
    "disparity": airsim.ImageType.DisparityNormalized,
    "Infrared": airsim.ImageType.Infrared
}

vehicle_velocity = 2.0
speedup_ratio = 10.0
speedup_flag = False
vehicle_yaw_rate = 5.0

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

    if scan_wrapper[pygame.K_SPACE]:
        scale_ratio = speedup_ratio
    else:
        scale_ratio = speedup_ratio / speedup_ratio
    if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
        yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate
    if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
        velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio
    if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
        velocity_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio
    if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
        velocity_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio

    #print(f": Expectation gesture: {velocity_x}, {velocity_y}, {velocity_z}, {yaw_rate}")

    AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=1, yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate), vehicle_name=vehicle_name)
    temp_image1 = AirSim_client.simGetImage('0', image_types["scene"], vehicle_name=vehicle_name)
    image1 = cv2.imdecode(airsim.string_to_uint8_array(temp_image1), cv2.IMREAD_COLOR)
    cv2.imwrite('screen/visual1.png', image1)
    screen_image1 = pygame.image.load("screen/visual1.png")
    screen.blit(screen_image1, (0, 0))
    pygame.display.flip()
    pygame.display.update()

    temp_image2 = AirSim_client.simGetImage('0', image_types["Infrared"], vehicle_name=vehicle_name)
    image2 = cv2.imdecode(airsim.string_to_uint8_array(temp_image2), cv2.IMREAD_COLOR)
    cv2.imwrite('screen/visual2.png', image2)
    screen_image2 = pygame.image.load("screen/visual2.png")
    screen.blit(screen_image2, (272, 0))
    pygame.display.flip()
    pygame.display.update()

    temp_image3 = AirSim_client.simGetImage('0', image_types["segmentation"], vehicle_name=vehicle_name)
    image3 = cv2.imdecode(airsim.string_to_uint8_array(temp_image3), cv2.IMREAD_COLOR)
    cv2.imwrite('screen/visual3.png', image3)
    screen_image3 = pygame.image.load("screen/visual3.png")
    screen.blit(screen_image3, (544, 0))
    pygame.display.flip()
    pygame.display.update()

    if scan_wrapper[pygame.K_ESCAPE]:
        pygame.quit()
        sys.exit()