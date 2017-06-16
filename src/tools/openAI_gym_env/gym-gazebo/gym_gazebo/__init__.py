from gym.envs.registration import register

register(
  id='gazebo-kobuki-v0',
  entry_point='gym_gazebo.envs:KobukiEnv',
)
