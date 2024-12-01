import sys
import time
import airsim
import pygame
import cv2
# import numpy as np
# import threading

def drone(*args, **kwargs):
    vehicle_names = []
    for i, arg in enumerate(args):
        vehicle_names.append(arg)

    num_drones = len(args)
    #pygame setting
    pygame.init()
    screen = pygame.display.set_mode((800, 144 * num_drones))
    pygame.display.set_caption("Drone Screen")
    screen.fill((0, 0, 0))

    #airsim setting
    AirSim_client = airsim.MultirotorClient()
    AirSim_client.confirmConnection()

    #image setting
    image_types = {
        "scene": airsim.ImageType.Scene,
        "depth": airsim.ImageType.DepthVis,
        "seg": airsim.ImageType.Segmentation,
        "normals": airsim.ImageType.SurfaceNormals,
        "segmentation": airsim.ImageType.Segmentation,
        "disparity": airsim.ImageType.DisparityNormalized,
        "Infrared": airsim.ImageType.Infrared
    }

    for vehicle_name in vehicle_names:
        AirSim_client.enableApiControl(True, vehicle_name=vehicle_name)
        AirSim_client.armDisarm(True, vehicle_name=vehicle_name)
        AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

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

        screen_images =[]
        for idx, vehicle_name in enumerate(vehicle_names):
            #print(f": Expectation gesture: {velocity_x}, {velocity_y}, {velocity_z}, {yaw_rate}")
            AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=1, yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate), vehicle_name=vehicle_name)

            temp_image1 = AirSim_client.simGetImage('0', image_types["scene"], vehicle_name=vehicle_name)
            image1 = cv2.imdecode(airsim.string_to_uint8_array(temp_image1), cv2.IMREAD_COLOR)
            cv2.imwrite(f'screen/{vehicle_name}_visual1.png', image1)
            screen_images.append(pygame.image.load(f"screen/{vehicle_name}_visual1.png"))
            # screen.blit(screen_image1, (0, 0))
            # pygame.display.flip()
            # pygame.display.update()

            temp_image2 = AirSim_client.simGetImage('0', image_types["Infrared"], vehicle_name=vehicle_name)
            image2 = cv2.imdecode(airsim.string_to_uint8_array(temp_image2), cv2.IMREAD_COLOR)
            cv2.imwrite(f'screen/{vehicle_name}_visual2.png', image2)
            screen_images.append(pygame.image.load(f"screen/{vehicle_name}_visual2.png"))
            # screen.blit(screen_image2, (272, 0))
            # pygame.display.flip()
            # pygame.display.update()

            temp_image3 = AirSim_client.simGetImage('0', image_types["segmentation"], vehicle_name=vehicle_name)
            image3 = cv2.imdecode(airsim.string_to_uint8_array(temp_image3), cv2.IMREAD_COLOR)
            cv2.imwrite(f'screen/{vehicle_name}_visual3.png', image3)
            screen_images.append(pygame.image.load(f"screen/{vehicle_name}_visual3.png"))
            # screen.blit(screen_image3, (544, 0))
            # pygame.display.flip()
            # pygame.display.update()

            # screen.blit(screen_image1, (0, 144 * idx))
            # screen.blit(screen_image2, (272, 144 * idx))
            # screen.blit(screen_image3, (544, 144 * idx))
        for idx, image in enumerate(screen_images):
            screen.blit(image, ((272 * (idx % 3)), 144 * (idx // 3)))
        pygame.display.update()

        if scan_wrapper[pygame.K_ESCAPE]:
            pygame.quit()
            sys.exit()

def main():
    #drone("Drone0", "Drone1", "Drone2")
    drone("Drone0")

if __name__ == "__main__":
    main()