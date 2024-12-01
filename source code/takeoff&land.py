import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)