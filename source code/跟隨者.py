import time
import airsim
import numpy as np
import math

def calculate_distance(point1, point2):
    """計算兩點之間的歐幾里得距離"""
    return np.linalg.norm(np.array(point1) - np.array(point2))

def find_target_position(leader_path, follower_position, desired_distance):
    """找到與follower距離最近但保持指定距離的路徑點"""
    for i in range(len(leader_path) - 1, -1, -1):  # 從最新的點開始檢查
        distance = calculate_distance(leader_path[i], follower_position)
        if distance >= desired_distance:
            return leader_path[i]
    return leader_path[0]  # 若未找到，返回最早的點

def move_follower(client, follower_name, follower_position, target_position, speed):
    """移動follower到目標位置"""
    direction = np.array(target_position) - np.array(follower_position)
    distance = np.linalg.norm(direction)
    if distance < 0.1:  # 避免頻繁修正
        return
    velocity = speed * direction / distance  # 確保速度恒定
    client.moveByVelocityAsync(velocity[0], velocity[1], velocity[2], 0.1, vehicle_name=follower_name).join()

def main():
    # 連接到AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    leader_name = "Drone0"
    follower_name = "Drone2"
    desired_distance = 2.0  # 保持的距離
    speed = 2.0  # follower的速度

    # 初始化follower
    client.enableApiControl(True, vehicle_name=follower_name)
    client.armDisarm(True, vehicle_name=follower_name)

    # follower起飛
    follower_z = -7  # 高度
    client.takeoffAsync(vehicle_name=follower_name).join()
    client.moveToZAsync(follower_z, 2, vehicle_name=follower_name).join()

    leader_path = []  # 用於記錄leader的移動路徑
    try:
        while True:
            # 獲取leader位置
            leader_state = client.getMultirotorState(vehicle_name=leader_name)
            leader_position = [
                leader_state.kinematics_estimated.position.x_val,
                leader_state.kinematics_estimated.position.y_val,
                leader_state.kinematics_estimated.position.z_val
            ]
            leader_path.append(leader_position)  # 記錄leader位置

            # 獲取follower位置
            follower_state = client.getMultirotorState(vehicle_name=follower_name)
            follower_position = [
                follower_state.kinematics_estimated.position.x_val,
                follower_state.kinematics_estimated.position.y_val+2,
                follower_state.kinematics_estimated.position.z_val
            ]

            # 計算目標位置
            target_position = find_target_position(leader_path, follower_position, desired_distance)

            # 移動follower
            move_follower(client, follower_name, follower_position, target_position, speed)

            # 等待一小段時間
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("中斷操作，follower降落中...")
    finally:
        # 降落
        client.landAsync(vehicle_name=follower_name).join()
        client.armDisarm(False, vehicle_name=follower_name)
        client.enableApiControl(False, vehicle_name=follower_name)

if __name__ == "__main__":
    main()


# import time
# import airsim
# import numpy as np
# import math
#
#
# def calculate_acceleration_and_yaw(a, b, speed=5.0):
#     delta = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]])
#     distance = np.linalg.norm(delta)
#
#     if distance < 0.1:
#         return 0, 0, 0, 0
#     direction = delta / distance
#     velocity = speed * direction
#     ax, ay, az = velocity[0], velocity[1], velocity[2]
#     yaw = math.atan2(-ay, ax)
#     return ax, ay, az, yaw
#
#
# def follow_leader(client, leader_name, follower_name, offset, speed=2.0):
#     follower_trajectory = []
#
#     while True:
#         # 獲取leader的位置
#         leader_state = client.getMultirotorState(vehicle_name=leader_name)
#         leader_position = np.array([
#             leader_state.kinematics_estimated.position.x_val,
#             leader_state.kinematics_estimated.position.y_val,
#             leader_state.kinematics_estimated.position.z_val
#         ])
#
#         # follower目標位置（相對於leader位置的偏移）
#         target_position = leader_position + offset
#
#         # 獲取follower當前位置
#         follower_state = client.getMultirotorState(vehicle_name=follower_name)
#         follower_position = np.array([
#             follower_state.kinematics_estimated.position.x_val,
#             follower_state.kinematics_estimated.position.y_val,
#             follower_state.kinematics_estimated.position.z_val
#         ])
#
#         # 計算加速度與偏航角
#         ax, ay, az, yaw = calculate_acceleration_and_yaw(follower_position, target_position, speed)
#
#         # 移動follower
#         client.moveByVelocityAsync(ax, ay, az, 0.1, vehicle_name=follower_name).join()
#
#         # 記錄軌跡
#         follower_trajectory.append((follower_position[0], follower_position[1], follower_position[2]))
#         client.simPlotLineStrip(
#             [airsim.Vector3r(*pos) for pos in follower_trajectory],
#             color_rgba=[1, 0, 0, 1],
#             thickness=5.0,
#             is_persistent=True
#         )
#
#         # 判斷是否接近目標，防止多次修正
#         distance_to_target = np.linalg.norm(follower_position - target_position)
#         if distance_to_target < 0.1:
#             print(f"{follower_name} 已接近目標位置。")
#             break
#
#         time.sleep(0.1)
#
#
# def main():
#     # 連接到AirSim
#     client = airsim.MultirotorClient()
#     client.confirmConnection()
#
#     leader_name = "Drone0"
#     follower_name = "Drone1"
#
#     # 初始化follower
#     client.enableApiControl(True, vehicle_name=follower_name)
#     client.armDisarm(True, vehicle_name=follower_name)
#
#     # follower起飛
#     follower_z = -1  # 高度固定
#     client.takeoffAsync(vehicle_name=follower_name).join()
#     client.moveToZAsync(follower_z, 2, vehicle_name=follower_name).join()
#
#     # 定義follower相對leader的偏移 (-2 in x-direction)
#     offset = np.array([-2.0, 0.0, 0.0])
#
#     try:
#         while True:
#             follow_leader(client, leader_name, follower_name, offset, speed=2.0)
#     except KeyboardInterrupt:
#         print("中斷跟隨動作。")
#     finally:
#         # 降落
#         client.landAsync(vehicle_name=follower_name).join()
#         client.armDisarm(False, vehicle_name=follower_name)
#         client.enableApiControl(False, vehicle_name=follower_name)
#
#
# if __name__ == "__main__":
#      main()
