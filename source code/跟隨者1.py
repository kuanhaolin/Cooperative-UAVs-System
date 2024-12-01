import airsim
import math
import numpy as np
import time

def distance(leader_position, follower_position):
    return np.linalg.norm(leader_position - follower_position)

def calculate_acceleration_and_yaw(a, b, speed):
    delta = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]]) #兩點向量
    distance = np.linalg.norm(delta) #歐式距離
    # if distance < 2:
    #     return 0, 0, 0, 0
    direction = delta / distance
    velocity = speed * direction
    ax, ay, az = velocity[0], velocity[1], velocity[2]
    yaw = math.atan2(-ay, ax)  #計算偏航角
    return ax, ay, az, yaw

def is_leader_landed(client, leader_name):
    leader_state = client.getMultirotorState(vehicle_name=leader_name)
    landed_state = leader_state.landed_state
    return landed_state == airsim.LandedState.Landed

def fly_to_leader(client, leader_name, follower_name):
    #leader的位置和偏航角
    leader_state = client.getMultirotorState(leader_name)
    leader_position = np.array([
        leader_state.kinematics_estimated.position.x_val,
        leader_state.kinematics_estimated.position.y_val,
        leader_state.kinematics_estimated.position.z_val
    ])
    #leader_orientation = leader_state.kinematics_estimated.orientation

    follower_state = client.getMultirotorState(follower_name)
    follower_position = np.array([
        follower_state.kinematics_estimated.position.x_val-2,
        follower_state.kinematics_estimated.position.y_val,
        follower_state.kinematics_estimated.position.z_val
    ])
    #follower_orientation = leader_state.kinematics_estimated.orientation

    while True:
        if distance(leader_position, follower_position) > 2.0:
            ax, ay, az, yaw = calculate_acceleration_and_yaw(follower_position, leader_position, 2.0)
            client.moveByRollPitchYawZAsync(0, 0, yaw, -1, 0.1, vehicle_name=follower_name).join()
            client.moveByVelocityAsync(ax, ay, az, 0.1, vehicle_name=follower_name).join()
        else:
            # repulsion = follower_position - leader_position
            # repulsion /= np.linalg.norm(repulsion)
            # repulsion_strength = 2.0
            # ax, ay, az = repulsion_strength * repulsion
            # client.moveByVelocityAsync(ax, ay, az, 0.1, vehicle_name="Drone1").join()
            ax, ay, az, yaw = calculate_acceleration_and_yaw(follower_position, leader_position, -2.0)  # 往反方向移動
            client.moveByVelocityAsync(ax, ay, az, 0.25, vehicle_name=follower_name)
            print("距離過近")

        #leader的位置和偏航角
        leader_state = client.getMultirotorState(leader_name)
        leader_position = np.array([
            leader_state.kinematics_estimated.position.x_val,
            leader_state.kinematics_estimated.position.y_val,
            leader_state.kinematics_estimated.position.z_val
        ])
        #leader_orientation = leader_state.kinematics_estimated.orientation

        follower_state = client.getMultirotorState(follower_name)
        follower_position = np.array([
            follower_state.kinematics_estimated.position.x_val - 2,
            follower_state.kinematics_estimated.position.y_val,
            follower_state.kinematics_estimated.position.z_val
        ])
        #follower_orientation = leader_state.kinematics_estimated.orientation

        if is_leader_landed(client, leader_name):
            break
        #time.sleep(0.1)

def main():
    #連接到AirSim
    leader_name = "Drone0"
    follower_name = "Drone1"
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, follower_name)
    client.armDisarm(True, follower_name)

    #起飛
    follower_z = -1
    client.takeoffAsync(follower_name).join()
    client.moveToZAsync(follower_z, 2, follower_name).join()

    fly_to_leader(client, leader_name, follower_name)

    #降落
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)

if __name__ == "__main__":
    main()