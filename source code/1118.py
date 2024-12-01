import airsim
import time
import math

# 連接 AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# 啟動無人機
vehicles = ["Drone0", "Drone1"]
for drone in vehicles:
    client.enableApiControl(True, drone)
    client.armDisarm(True, drone)

# 起飛所有無人機
for drone in vehicles:
    client.takeoffAsync(vehicle_name=drone).join()

def move_leader(client, duration=10, speed=5):
    """
    領導者的簡單直線運動：沿 X 軸運動。
    """
    for t in range(duration):
        x = speed * t
        client.moveToPositionAsync(x, 0, -10, speed, vehicle_name="Drone0").join()
        time.sleep(0.5)  # 更新頻率

def update_followers(client, offset=5):
    """
    更新跟隨者的位置，使其保持與領導者一定距離。
    """
    leader_position = client.getMultirotorState(vehicle_name="Drone0").kinematics_estimated.position
    leader_x, leader_y, leader_z = leader_position.x_val, leader_position.y_val, leader_position.z_val

    # 跟隨者 1 跟隨
    client.moveToPositionAsync(
        leader_x - offset, leader_y - offset, leader_z, 5, vehicle_name="Drone1"
    ).join()

    # 跟隨者 2 跟隨
    # client.moveToPositionAsync(
    #     leader_x + offset, leader_y + offset, leader_z, 5, vehicle_name="Follower2"
    # ).join()

def main():
    try:
        for i in range(50):  # 控制 50 次更新
            move_leader(client, duration=1, speed=5)  # 領導者移動
            update_followers(client, offset=5)       # 跟隨者調整
            time.sleep(0.1)  # 控制頻率
    finally:
        # 降落並解除控制
        for drone in vehicles:
            client.landAsync(vehicle_name=drone).join()
            client.armDisarm(False, vehicle_name=drone)
            client.enableApiControl(False, vehicle_name=drone)

if __name__ == "__main__":
    main()

