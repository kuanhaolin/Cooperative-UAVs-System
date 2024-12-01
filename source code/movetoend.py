import airsim
import time

from tornado.gen import sleep

# 連接 AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone0")
client.armDisarm(True, "Drone0")

# 起飛
client.takeoffAsync(vehicle_name="Drone0").join()
client.moveToPositionAsync(0, 0, -10, 7, vehicle_name="Drone0").join()
time.sleep(2)
# 移動到指定位置
client.moveToPositionAsync(26, 26, -10, 7, vehicle_name="Drone0").join()
time.sleep(2)
# 返回原點
#client.moveToPositionAsync(0, 0, -10, 5, vehicle_name="Drone0").join()
# 降落
client.landAsync(vehicle_name="Drone0").join()
time.sleep(2)
# 解除控制
client.armDisarm(False, "Drone0")
client.enableApiControl(False, "Drone0")
