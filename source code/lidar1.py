import airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 起點 A 和 目標點 B
start_position = np.array([0, 0, -3])  # A 点
goal_position = np.array([27, 27, -3])  # B 点

# 吸引力和排斥力系数
k_att = 1.0  # 吸引力系数
k_rep = 100.0  # 排斥力系数
d0 = 5.0  # 排斥力的影响范围

# 移动步长
step_size = 1

# 计算吸引力和排斥力的合力
def calculate_forces(current_position, obstacles, goal_position):
    # 吸引力
    attractive_force = k_att * (goal_position - current_position)

    # 排斥力
    repulsive_force = np.zeros(3)
    for obstacle in obstacles:
        distance = np.linalg.norm(current_position - obstacle)
        if distance < d0:
            repulsive_force += k_rep * (1.0 / distance - 1.0 / d0) * (1.0 / distance ** 2) * (current_position - obstacle) / distance

    # 合力
    total_force = attractive_force + repulsive_force
    return total_force

# 路徑導航和避障
def navigate_with_obstacle_avoidance():
    while True:
        # 获取无人机当前位置
        drone_position = client.getMultirotorState().kinematics_estimated.position
        current_position = np.array([drone_position.x_val, drone_position.y_val, drone_position.z_val])

        # 读取 LiDAR 数据以检测障碍物
        lidar_data = client.getLidarData("LidarSensor1")
        if len(lidar_data.point_cloud) < 3:
            continue

        obstacles = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

        # 计算总合力
        total_force = calculate_forces(current_position, obstacles, goal_position)
        print("当前合力:", total_force)

        # 更新位置并朝向合力方向移动
        velocity = total_force * step_size
        print("移动速度:", velocity)
        client.moveByVelocityAsync(velocity[0], velocity[1], velocity[2], 1).join()

        # 检查无人机是否接近目标点
        if np.linalg.norm(goal_position - current_position) < 1.0:
            print("到达目标点 B！")
            break

# 执行导航和避障
navigate_with_obstacle_avoidance()

client.armDisarm(False)
client.enableApiControl(False)

# import airsim
# import numpy as np
# import time
#
# # Connect to the AirSim simulator
# client = airsim.MultirotorClient()
# client.confirmConnection()
# client.enableApiControl(True)
#
# # Set the UAV's initial position
# start_position = airsim.Vector3r(0, 0, -3)
# client.moveByVelocityZBodyFrameAsync(0, 0, -3, 5).join()
#
# # Artificial Potential Field parameters
# goal_position = airsim.Vector3r(26, 26, -3)
# kp = 1.0  # Attractive potential gain
# kr = 2.0  # Repulsive potential gain
# safety_radius = 5.0  # Obstacle's influence radius
#
# def get_lidar_obstacles():
#     """
#     Retrieve the obstacles detected by the UAV's LiDAR sensor.
#     Returns a list of obstacle positions.
#     """
#     lidar_data = client.getLidarData("LidarSensor1")
#     obstacle_positions = [airsim.Vector3r(x, y, z) for x, y, z in lidar_data.point_cloud]
#     return obstacle_positions
#
# def calculate_potential_field(uav_position, goal_position, obstacle_positions):
#     """
#     Calculate the artificial potential field for the UAV.
#     Returns the attractive and repulsive forces.
#     """
#     attractive_force = kp * (goal_position - uav_position)
#     repulsive_force = np.zeros(3)
#     for obstacle in obstacle_positions:
#         distance = np.linalg.norm(obstacle - uav_position)
#         if distance < safety_radius:
#             repulsive_force += kr * (1/distance - 1/safety_radius) * (obstacle - uav_position) / distance**2
#     return attractive_force, repulsive_force
#
# while True:
#     # Get the UAV's current position
#     uav_position = client.getMultirotorState().kinematics_estimated.position
#
#     # Get the obstacles detected by the LiDAR sensor
#     obstacle_positions = get_lidar_obstacles()
#
#     # Calculate the artificial potential field
#     attractive_force, repulsive_force = calculate_potential_field(uav_position, goal_position, obstacle_positions)
#     total_force = attractive_force + repulsive_force
#
#     # Move the UAV based on the potential field
#     client.moveByVelocityZBodyFrameAsync(total_force.x_val, total_force.y_val, -1, 1).join()
#
#     time.sleep(0.1)