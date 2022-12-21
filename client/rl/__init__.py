from gym.envs.registration import register

register(id='ParkingLot-v0', entry_point='client.rl.env:ParkingLotEnv')
