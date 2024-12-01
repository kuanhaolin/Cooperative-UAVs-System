# import airsim
# import numpy as np
# import time
# import math
#
#
# class ObstacleAvoidanceCarrotChasing:
#     def __init__(self, client, waypoints, lidar_name, carrot_distance=2.0, repulsive_gain=1000.0, attractive_gain=5.0):
#         self.client = client
#         self.waypoints = waypoints
#         self.lidar_name = lidar_name
#         self.carrot_distance = carrot_distance  # 胡蘿蔔點距離 "設定導航點的前方距離 無人機不會直接往最終點移動 而是持續跟隨前方的胡蘿蔔點"
#         self.repulsive_gain = repulsive_gain  # 排斥力增益
#         self.attractive_gain = attractive_gain  # 吸引力增益
#         self.max_repulsive_force = 1000.0
#
#     # 兩點直線距離
#     def distance(self, p1, p2):
#         # return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5
#         return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)
#
#     def get_obstacle_force(self, lidar_data):
#         # 初始化排斥力
#         repulsive_force = np.array([0.0, 0.0, 0.0])
#         for i in range(0, len(lidar_data.point_cloud), 3):
#             obstacle_point = np.array(lidar_data.point_cloud[i:i + 3])
#             distance = np.linalg.norm(obstacle_point)
#             if distance > 0:
#                 # 計算排斥力（距離越近，排斥力越強）
#                 force_direction = -obstacle_point / distance
#                 force_magnitude = self.repulsive_gain / (distance ** 2)
#                 repulsive_force += force_magnitude * force_direction
#         return repulsive_force
#
#     def follow_path(self):
#         for waypoint in self.waypoints:
#             while True:
#                 # 獲取無人機當前位置
#                 position = self.client.getMultirotorState().kinematics_estimated.position
#                 drone_position = np.array([position.x_val, position.y_val, position.z_val])
#
#                 # 檢查是否接近目標點
#                 if self.distance(drone_position, waypoint) < self.carrot_distance:
#                     break  # 接近目標點，前往下一點
#
#                 # 吸引力：從無人機位置指向目標點
#                 attractive_force = self.attractive_gain * (np.array(waypoint) - drone_position)
#
#                 # 排斥力：從 LiDAR 獲取障礙物數據
#                 lidar_data = self.client.getLidarData(lidar_name=self.lidar_name)
#                 if len(lidar_data.point_cloud) >= 3:
#                     # 將point_cloud資料轉換成Nx3的(x, y, z)形式
#                     obstacles = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
#                     # obstacles中的每一行是 (x, y, z)，表示障礙物相對於無人機的方向和距離
#                     for obstacle_vector in obstacles:
#                         distance = np.linalg.norm(obstacle_vector)  # 計算距離
#                         print("偵測到障礙物相對位置:", obstacle_vector, "距離:", distance)
#                 else:
#                     print("未偵測到障礙物")
#
#                 if lidar_data.point_cloud:
#                     #如果有偵測到障礙物，計算排斥力
#                     repulsive_force = self.get_obstacle_force(lidar_data)
#                 else:
#                     #如果沒有障礙物，排斥力為零
#                     repulsive_force = np.array([0.0, 0.0, 0.0])
#                 # repulsive_force = self.get_obstacle_force(lidar_data)
#
#                 repulsive_force_magnitude = np.linalg.norm(repulsive_force)
#                 if repulsive_force_magnitude > self.max_repulsive_force:
#                     repulsive_force = (repulsive_force / repulsive_force_magnitude) * self.max_repulsive_force
#
#                 # Combine forces
#                 total_force = attractive_force + repulsive_force
#
#                 # 合成力：吸引力和排斥力之和
#                 #total_force = attractive_force + repulsive_force
#                 if np.linalg.norm(total_force) > 0:
#                     direction = total_force / np.linalg.norm(total_force)  # 規範化方向
#                 else:
#                     direction = np.array([0.0, 0.0, 0.0])  # 沒有移動方向
#                 # direction = total_force / np.linalg.norm(total_force)  # 規範化方向
#
#                 print(f"Drone position: {drone_position}")
#                 print(f"Target waypoint: {waypoint}")
#                 print(f"Attractive force: {attractive_force}")
#                 print(f"Repulsive force: {repulsive_force}")
#                 print(f"Total force: {total_force}")
#                 print(f"Direction to move: {direction}")
#
#                 # 設定無人機移動目標
#                 move_x, move_y, move_z = drone_position + direction
#                 self.client.moveToPositionAsync(move_x, move_y, move_z, 5).join()
#                 print(f"Next position: ({move_x}, {move_y}, {move_z})")
#
#                 time.sleep(0.1)  # 添加小延遲以避免過快刷新
#
#
# if __name__ == "__main__":
#     # 連接airsim
#     vehicle_name = "Drone0"
#     client = airsim.MultirotorClient()
#     client.confirmConnection()
#     client.enableApiControl(True, vehicle_name=vehicle_name)
#     client.armDisarm(True, vehicle_name=vehicle_name)
#
#     # 起飛
#     client.takeoffAsync().join()
#
#     # 設定路徑點
#     waypoints = [
#         (0, 0, -3),
#         #起始點
#         (27, 0, -3),
#         (27, 27, -3),
#         (0, 27, -3)
#         # 目標點
#     ]
#
#     # 創建Carrot Chasing對象並跟隨路徑
#     avoidance_chaser = ObstacleAvoidanceCarrotChasing(client, waypoints, lidar_name="LidarSensor1")
#     avoidance_chaser.follow_path()
#
#     # 降落
#     client.landAsync().join()
#     client.armDisarm(False)
#     client.enableApiControl(False)

import airsim
import numpy as np
import time

# 初始化 AirSim 無人機客戶端
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 起飛並穩定到一定高度
client.takeoffAsync().join()
client.moveToZAsync(-3, 5).join()

# 設定起點 A 和目標點 B
start_position = client.getMultirotorState().kinematics_estimated.position
target_position = airsim.Vector3r(27, 0, -3)  # B 點

# 參數設置
obstacle_distance_threshold = 5.0  # 障礙物偵測距離
carrot_distance = 3.0  # Carrot 距離（虛擬目標點）
repulsive_gain = 100.0  # 人工勢場法中的斥力增益
attractive_gain = 1.0  # 人工勢場法中的引力增益


def get_lidar_data():
    """
    獲取激光雷達數據，並返回最近的障礙物位置
    """
    lidar_data = client.getLidarData("LidarSensor1")
    if lidar_data.point_cloud:  # 確保有返回點雲數據
        # 將點雲數據轉換為 numpy 陣列
        lidar_points = np.array(lidar_data.point_cloud).reshape(-1, 3)
        current_pos = client.getMultirotorState().kinematics_estimated.position
        current_pos = np.array([current_pos.x_val, current_pos.y_val, current_pos.z_val])
        absolute_points = lidar_points + current_pos #障礙物絕對座標
        #假設只關注最近的障礙物
        #distances = np.linalg.norm(lidar_points, axis=1)  # 計算每個點的距離
        distances = np.linalg.norm(absolute_points - current_pos, axis=1)
        #min_distance_idx = np.argmin(distances)  #找到最近的點
        #nearest_point = lidar_points[min_distance_idx]
        return absolute_points, distances
    return None, None


def calculate_attractive_force(current_pos, target_pos):
    """
    計算到目標點的吸引力
    """
    direction = np.array([target_pos.x_val - current_pos.x_val,
                          target_pos.y_val - current_pos.y_val,
                          target_pos.z_val - current_pos.z_val])
    distance = np.linalg.norm(direction)
    #距離A-B
    attractive_force = attractive_gain * direction / distance
    #吸引力
    return attractive_force


def calculate_repulsive_force(current_pos, obstacle_pos):
    """
    計算避障的斥力
    """
    direction = np.array([current_pos.x_val - obstacle_pos.x_val,
                          current_pos.y_val - obstacle_pos.y_val,
                          current_pos.z_val - obstacle_pos.z_val])
    distance = np.linalg.norm(direction)
    if distance < obstacle_distance_threshold:
        repulsive_force = repulsive_gain * (1.0 / distance - 1.0 / obstacle_distance_threshold) * (
                    direction / distance ** 2)
    else:
        repulsive_force = np.array([0.0, 0.0, 0.0])
    return repulsive_force


def carrot_chasing(current_pos, target_pos, carrot_distance=5.0):
    """
    使用 Carrot Chasing 方法計算下一個目標位置
    """
    direction = np.array([target_pos.x_val - current_pos.x_val,
                          target_pos.y_val - current_pos.y_val,
                          target_pos.z_val - current_pos.z_val])
    distance = np.linalg.norm(direction)

    # 避免除以零
    if distance == 0:
        return current_pos

    # 計算方向向量並轉換為 airsim.Vector3r
    direction = direction / distance * carrot_distance
    carrot_point = airsim.Vector3r(current_pos.x_val + direction[0],
                                    current_pos.y_val + direction[1],
                                    current_pos.z_val + direction[2])
    return carrot_point


def navigate_to_target():
    """
    導航到目標點，遇到障礙物則啟用人工勢場法避障
    """
    while True:
        current_pos = client.getMultirotorState().kinematics_estimated.position
        #目前位置
        distance_to_target = np.linalg.norm(
            [target_position.x_val - current_pos.x_val,
             target_position.y_val - current_pos.y_val,
             target_position.z_val - current_pos.z_val]
        )
        #計算當前與目標距離

        #檢查是否已接近B點
        if distance_to_target < 1.0:
            print("Reached target B!")
            break

        #檢查是否有障礙物
        obstacle_pos, obstacle_distance = get_lidar_data()
        if obstacle_pos is not None and obstacle_distance < obstacle_distance_threshold: #5
            print(f"距離{obstacle_distance}公尺處 偵測到障礙物 進行躲避。")
            # 計算人工勢場法的合力
            attractive_force = calculate_attractive_force(current_pos, target_position)
            repulsive_force = calculate_repulsive_force(current_pos, obstacle_pos)
            total_force = attractive_force + repulsive_force

            # 計算下一步位置
            next_position = airsim.Vector3r(
                current_pos.x_val + total_force[0],
                current_pos.y_val + total_force[1],
                current_pos.z_val + total_force[2]
            )
            client.moveToPositionAsync(next_position.x_val, next_position.y_val, next_position.z_val, 2).join()

        else:
            # 沒有障礙物，使用 Carrot Chasing 前進
            carrot_point = carrot_chasing(current_pos, target_position)
            client.moveToPositionAsync(carrot_point.x_val, carrot_point.y_val, carrot_point.z_val, 1).join()
        #time.sleep(0.5)

# 開始導航至目標 B
navigate_to_target()

# 任務完成，降落
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)