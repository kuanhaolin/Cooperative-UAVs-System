import time
import airsim
import math
import numpy as np
import matplotlib.pyplot as plt
"""
  +x
-y + +y
  -y
"""
#計算吸引力
def calculate_attraction(position, goal, coefficient):
    return coefficient * (goal - position)

#計算排斥力
def calculate_repulsion(position, obstacles, coefficient, threshold):
    repulsion_force = np.array([0.0, 0.0, 0.0])
    for obstacle in obstacles:
        obstacle_vec = position - obstacle
        distance = np.linalg.norm(obstacle_vec)
        if distance < threshold:
            #增加距離判斷，以防除以0
            if distance == 0:
                distance = 0.01 #避免除以0
        repulsion_force += coefficient * (1 / threshold - 1 / distance) * (obstacle_vec / distance ** 3)
    return repulsion_force

#計算從當前位置到目標位置的加速度和偏航角
def calculate_acceleration_and_yaw(a, b, speed=2.0):
    delta = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]]) #兩點向量
    distance = np.linalg.norm(delta) #歐式距離

    if distance < 0.1:
        return 0, 0, 0, 0
    direction = delta / distance
    velocity = speed * direction
    ax, ay, az = velocity[0], velocity[1], velocity[2]
    yaw = math.atan2(-ay, ax)  #計算偏航角
    return ax, ay, az, yaw

def lidar(client):
    lidar_data = client.getLidarData("LidarSensor1")
    if len(lidar_data.point_cloud) < 3:
        print("雷達沒有偵測到數據")#目前setting 是設定"Range": 10
        return 0

    #將點雲轉換成座標
    obstacles = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    #篩選障礙物距離
    threshold_distance = 3
    for i, (x, y, z) in enumerate(obstacles):
        #歐氏距離計算(x^2+y^2+z^2)^0.5
        distance = np.linalg.norm([x, y, z])
        if distance < threshold_distance:
            print(f"\n偵測障礙物！ 距離無人機為{distance:.2f}公尺")
            return x, y, z, distance
        else:
            print("無障礙物")
            return 0

def get_global_coordinates(client, lidar_data):
    #無人機的姿態
    state = client.getMultirotorState()
    drone_position = np.array([
        state.kinematics_estimated.position.x_val,
        state.kinematics_estimated.position.y_val,
        state.kinematics_estimated.position.z_val
    ])
    drone_orientation = state.kinematics_estimated.orientation
    rotation_matrix = quaternion_to_rotation_matrix(drone_orientation)

    #點雲轉換
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)  #局部座標
    global_points = np.dot(rotation_matrix, points.T).T + drone_position  #轉換到全局座標
    return global_points

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    R = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])
    return R

def fly_to_point(client, P1, P2, height, speed=2.0):
    current_pos = P1
    target_pos = np.array([P2[0], P2[1], height])
    path = []

    #初始化軌跡列表
    trajectory = [airsim.Vector3r(*current_pos)]

    while True:
        path.append((current_pos[0], current_pos[1]))
        obstacles = lidar(client)
        if obstacles != 0:
            lidar_data = client.getLidarData("LidarSensor1")
            global_obstacles = get_global_coordinates(client, lidar_data)
            for i, obstacle in enumerate(global_obstacles):
                print(f"障礙物座標: ({obstacle[0]:.2f}, {obstacle[1]:.2f}, {obstacle[2]:.2f})\n無人機座標: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
            #人工勢場法
            attraction = calculate_attraction(current_pos, target_pos, attraction_coefficient) #1
            repulsion = calculate_repulsion(current_pos, global_obstacles, repulsion_coefficient, repulsion_threshold)
            force = attraction - repulsion  #1
            print(f"吸引力: {attraction}  排斥力: {repulsion}  合力: {force}")
            new_pos = current_pos + step_size * force / np.linalg.norm(force) #1
            print(f"({new_pos[0]:.2f}, {new_pos[1]:.2f}, {new_pos[2]:.2f})")
            client.moveToPositionAsync(new_pos[0], new_pos[1], new_pos[2], 0.1).join() #1
            #0 repulsion_factor = 0.1
            #client.moveByVelocityBodyFrameAsync(vx=0, vy=1, vz=0, duration=0.02)
            #0 target_adjusted = target_pos + repulsion * repulsion_factor
            #更新飞行目标
            #0 ax, ay, az, yaw = calculate_acceleration_and_yaw(current_pos,np.array([target_adjusted[0], target_adjusted[1], height]),speed)
            #client.moveByRollPitchYawZAsync(0, 0, yaw, height, 0.1).join()
            #client.moveByVelocityAsync(ax, ay, az, 0.1).join()
            #0 adjusted_velocity = np.array([ax, ay, az]) + repulsion * repulsion_factor
            #max_velocity = 2.0
            #adjusted_velocity = np.clip(adjusted_velocity, -max_velocity, max_velocity)
            #0 print(f"(x:{adjusted_velocity[0]}, y:{adjusted_velocity[1]})")
            #0 client.moveByVelocityAsync(adjusted_velocity[0], adjusted_velocity[1], 0,duration=0.1).join()
        else:
            ax, ay, az, yaw = calculate_acceleration_and_yaw(current_pos, target_pos, speed)
            client.moveByRollPitchYawZAsync(0, 0, yaw, height, 0.1).join()
            client.moveByVelocityAsync(ax, ay, az, 0.1).join()

        #記錄軌跡點
        trajectory.append(airsim.Vector3r(current_pos[0], current_pos[1], current_pos[2]))
        client.simPlotLineStrip(trajectory, color_rgba=[0, 0, 1, 1], thickness=5.0, is_persistent=True)

        #更新當前位置
        state = client.getMultirotorState()
        current_pos = np.array([
            state.kinematics_estimated.position.x_val,
            state.kinematics_estimated.position.y_val,
            state.kinematics_estimated.position.z_val
        ])

        #判斷是否接近目標
        distance_to_target = np.linalg.norm(current_pos - target_pos)
        if distance_to_target < 1:
            print("到達目標點")
            break
    return path

def main():
    #連接到AirSim
    leader = "Drone0"
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=leader)
    client.armDisarm(True, vehicle_name=leader)

    #z軸(固定)
    z = -1
    #起飛
    client.takeoffAsync().join()
    client.moveToZAsync(z, 2).join()

    #設定目標點
    points = [(0,0,z), (5,5,z), (5,-5,z), (-15,15,z)]

    #畫座標點
    client.simSetTraceLine([1, 0, 0, 1], thickness=5)
    client.simFlushPersistentMarkers()
    points1 = [airsim.Vector3r(0,0,z), airsim.Vector3r(5,5,z), airsim.Vector3r(5,-5,z), airsim.Vector3r(-15,15,z)]
    client.simPlotPoints(points1, color_rgba=[0, 1, 0, 1], size=30, is_persistent=True)

    full_path = []  # 紀錄完整路徑

    for i in range(len(points) - 1):
        path = fly_to_point(client, points[i], points[i + 1], z)
        full_path.extend(path)
        time.sleep(0.5)

    #繪製路徑
    # x_coords, y_coords = zip(*full_path)  #解壓縮為 x, y
    # plt.figure(figsize=(8, 8))
    # plt.plot(y_coords, x_coords, marker="o", color="b", label="Flight Path")
    # #plt.scatter(*zip(*[(p[1], p[0]) for p in points]), color="r", label="Waypoints")  # 繪製目標點
    # plt.xlabel("Y-axis")
    # plt.ylabel("X-axis")
    # plt.title("Flight Path")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    #降落
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

if __name__ == "__main__":
    attraction_coefficient = 0.1
    repulsion_coefficient = 100 #100
    repulsion_threshold = 3
    max_iterations = 1000
    step_size = 0.25 #0.25
    main()