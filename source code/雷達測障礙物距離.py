#setting.json
# {
#     "SettingsVersion": 1.2,
#     "SimMode": "Multirotor",
#     "Vehicles": {
#         "Drone0": {
#             "VehicleType": "SimpleFlight",
#             "X": -20,
#             "Y": -12,
#             "Z": 0,
#             "Pitch": 0,
# 			"Yaw": 0,
#             "Sensors": {
#                 "LidarSensor1": {
#                     "SensorType": 6,
#                     "Range": 10,
#                     "Enabled": true,
#
#                     "NumberOfChannels": 16,
#                     "PointsPerSecond": 10000,
#                     "RotationsPerSecond": 0,
#
#                     "VerticalFOVUpper": 0,
#                     "VerticalFOVLower": 0,
#                     "HorizontalFOVStart": -90,
#                     "HorizontalFOVEnd": 90,
#
#                     "DrawDebugPoints": true,
#                     "DataFrame": "SensorLocalFrame",
#                     "X": 0, "Y": 0, "Z": 0,
#                     "Pitch": 0,
# 			        "Yaw": 0
#
#
#                 }
#             }
#         }
#     }
# }

#===================================#
#===================================#
#===================================#

import airsim
import time
import numpy as np
import csv
import os

#連接AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

#座標與感測存檔
output_file = "lidar_obstacle_data.csv"
if not os.path.exists(output_file):
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Point Index", "Distance", "X", "Y", "Z", "NX", "NY", "NZ"])

def calculate_distances():
    #無人機當前位置
    drone_position = client.getMultirotorState().kinematics_estimated.position
    #drone_x, drone_y, drone_z = drone_position.x_val, drone_position.y_val, drone_position.z_val

    #LiDAR數據
    lidar_data = client.getLidarData("LidarSensor1")
    if len(lidar_data.point_cloud) < 3:
        print("雷達沒有偵測到數據")#目前setting 是設定"Range": 10
        return

    #點雲轉換為座標
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

    #計算每個點到無人機的距離
    #distances = np.linalg.norm(points - np.array([drone_x, drone_y, drone_z]), axis=1)
    #save_obstacle_points(points, distances)

    #篩選障礙物距離
    threshold_distance = 5
    with open(output_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        for i, (x, y, z) in enumerate(points):
            #歐氏距離計算(x^2+y^2+z^2)^0.5
            distance = np.linalg.norm([x, y, z])
            if distance < threshold_distance:
                print(f"偵測障礙物！ 距離無人機為{distance:.2f}公尺")
            else:
                print("無障礙物")
            writer.writerow([i, x, y, z, distance, drone_position.x_val, drone_position.y_val, drone_position.z_val])
while True:
    calculate_distances()
    time.sleep(10)