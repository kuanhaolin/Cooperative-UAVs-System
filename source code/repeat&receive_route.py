import csv
import time
import airsim

#airsim setting
vehicle_name = "Drone"
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name=vehicle_name)
AirSim_client.armDisarm(True, vehicle_name=vehicle_name)
AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

#receive csv
waypoints = []
with open('waypoints.csv', mode='r') as file:
    reader = csv.reader(file)
    next(reader)
    #跳過標題
    for row in reader:
        x, y, z = map(float, row)
        waypoints.append(airsim.Vector3r(x, y, z))

#繪製路徑，紅線表示
AirSim_client.simPlotLineStrip(waypoints, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=10.0, is_persistent=True)
time.sleep(2)

#根據路徑飛行
for point in waypoints:
    AirSim_client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, 10.0, vehicle_name=vehicle_name).join()
    time.sleep(0.1)

AirSim_client.landAsync(vehicle_name=vehicle_name).join()
AirSim_client.armDisarm(False, vehicle_name=vehicle_name)
AirSim_client.enableApiControl(False, vehicle_name=vehicle_name)