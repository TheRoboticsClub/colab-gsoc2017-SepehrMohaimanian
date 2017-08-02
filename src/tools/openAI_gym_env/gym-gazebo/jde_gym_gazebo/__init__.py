from gym.envs.registration import register

register(
  id='jde-gazebo-kobuki-laser-v0',
  entry_point='jde_gym_gazebo.envs:KobukiLaserEnv',
)
