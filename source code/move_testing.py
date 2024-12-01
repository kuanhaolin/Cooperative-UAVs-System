import airsim
import time

def square_flight():
    client.moveToZAsync(-3, 1).join()
    #Z軸移動到3m, 速度1m/s
    # (x, y, z)(-4814, 4208, 15492)
    client.moveToPositionAsync(5, 0, -3, 1).join()
    #移動到相對座標(5, 0, -3), 速度1m/s
    client.moveToPositionAsync(5, 5, -3, 1).join()
    client.moveToPositionAsync(0, 5, -3, 1).join()
    client.moveToPositionAsync(0, 0, -3, 1).join()

def velocity_flight():
    client.moveToZAsync(-3, 1).join()
    client.moveByVelocityAsync(10, 0, 0, 5).join()
    #線性移動
    client.moveByVelocityAsync(0, 10, 0, 5).join()
    client.moveByVelocityAsync(-10, 0, 0, 5).join()
    client.moveByVelocityAsync(0, -10, 0, 5).join()
    client.hoverAsync().join()
    #停懸
    time.sleep(2)

client = airsim.MultirotorClient()
#connect airsim
client.confirmConnection()
client.enableApiControl(True)
#get control
client.armDisarm(True)
#unlock

client.takeoffAsync().join()
#.join -> step by step
#square_flight()
#velocity_flight()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
