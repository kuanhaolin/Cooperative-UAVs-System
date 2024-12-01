import time
import airsim
import math
import numpy as np
"""
  +x
-y + +y
  -y
"""
#計算從當前位置到目標位置的加速度和偏航角
def calculate_acceleration_and_yaw(a, b, speed=5.0):
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
    threshold_distance = 5
    for i, (x, y, z) in enumerate(obstacles):
        #歐氏距離計算(x^2+y^2+z^2)^0.5
        distance = np.linalg.norm([x, y, z])
        if distance < threshold_distance:
            print(f"偵測障礙物！ 距離無人機為{distance:.2f}公尺")
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
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)  # 局部座標
    global_points = np.dot(rotation_matrix, points.T).T + drone_position  # 轉換到全局座標
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

    while True:
        obstacles = lidar(client)
        if obstacles != 0:
            print(f"障礙物({obstacles[0]:.2f},{obstacles[1]:.2f},{obstacles[2]:.2f}), 實際座標({client.getMultirotorState().kinematics_estimated.position.x_val:.2f},{client.getMultirotorState().kinematics_estimated.position.y_val:.2f},{client.getMultirotorState().kinematics_estimated.position.z_val:.2f})")
            #避障
            client.moveByVelocityBodyFrameAsync(vx=0, vy=1, vz=0, duration=0.02)
        else:
            ax, ay, az, yaw = calculate_acceleration_and_yaw(current_pos, target_pos, speed)
            client.moveByRollPitchYawZAsync(0, 0, yaw, height, 0.1).join()
            client.moveByVelocityAsync(ax, ay, az, 0.1).join()

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

def main():
    #連接到airsim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    #z軸(固定)
    z = -7
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

    for i in range(len(points) - 1):
        fly_to_point(client, points[i], points[i + 1], z)
        time.sleep(0.5)

    #降落
    client.landAsync(2).join()
    client.armDisarm(False)
    client.enableApiControl(False)

if __name__ == "__main__":
    main()