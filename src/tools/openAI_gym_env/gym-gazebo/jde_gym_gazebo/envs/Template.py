#Gym stuff
import gym
from gym import error, spaces, utils
from gym.utils import seeding

#JdeRobot stuff
import easyiceconfig as EasyIce
import jderobotComm as comm
from jderobotTypes import LaserData
from jderobotTypes import CMDVel

#The rest
import numpy as np
import random
import sys
import time
import cv2

class TemplateJdeEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    # Initialize ICE
    ic = EasyIce.initialize(["YourRGBEnv", "PATH/TO/YOUR/ICE/CONFIG/FILE"])
    ic, node = comm.init(ic)
    # Initializing laser scanner from config file:
    self.laser_client = comm.getLaserClient(ic, "ROBOT.Laser")
    # Initializing motors from config file:
    self.motors_client = comm.getMotorsClient(ic, "ROBOT.Motors")
    # Initializing gazebo resetter from config file:
    self.gazebo_resetter =  comm.getGazeboActionClient(ic, "ROBOT.Reset")  
    # Initializing camera from config file:
    self.camera_client =  comm.getCameraClient(ic, "ROBOT.Camera")  
    
    # Initializing the environment:
    # Create your observation space (84x84 is default DQN agent)
    self.observation_dims = [ 84, 84]
    self.observation_space = spaces.Box(low=0, high=255, shape=( self.observation_dims[0], self.observation_dims[1], 1))
    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1],3), np.uint8)
    self.image = self.camera_client.getImage()
    # Crete your action space (either discrete to be mapped to velocities or continous, based on your agent)
    self.action_space = spaces.Discrete( NUmberOfActions)
    # Simple viwer for renderring the agent's input
    self.viewer = None
    # For collision detection in each step
    self.collision = False
    # Min safe distance from obstacle, if robot gets closer to an obstacle than this distance scenario will be terminated
    self.obstale_threshold = 0.5
    self.timeStamp = 0.0
    self.frames_skip = 0   
    print "Inited !"

  # Each step your agent will execute this function for its observation and gets a new obsevation/reward
  def _step(self, action):
    #For converting the discrete action for robot velocity
    self.actionToVel( action)
    # Get the sensors update
    self.getUpdate()
    
    # here define your reward function
    reward = 0
    if self.collision:
      reward = -99 # A huge penalty for hitting an obstacle
    else:
      if action == 1:
        reward = 0.6
      else:
        reward = -0.03

    # Extra information to be sent to agent for debug purposes
    info = {}
    # Send new observation, this step's reward, if we have collided with an obstacle or not, and extra info to the agent based on its last decision
    return self.screen, reward, self.collision, info

  # Used for resetting the Gazebo when an episode finishes
  def _reset(self):
    vel = CMDVel()
    vel.vx = 0.0
    vel.az = 0.0
    self.motors_client.sendVelocities(vel)
    self.gazebo_resetter.sendReset()
    self.getUpdate()
    return self.screen, 0,0 

  # For renderring the observation
  def _render(self, mode='human', close=False):
    if close:
      if self.viewer is not None:
        self.viewer.close()
        self.viewer = None
      return
    img = self.screen
    if mode == 'human':
      from gym.envs.classic_control import rendering
      if self.viewer is None:
        self.viewer = rendering.SimpleImageViewer()
      self.viewer.imshow(img)

  # Converting the action to robot velocities
  def actionToVel( self, action):
    action -= 1
    vel = CMDVel()
    if action == 0:
      vel.vx = 0.3
    else:
      vel.vx = 0.1
    vel.az = action*1.2
    self.motors_client.sendVelocities(vel)
  
  # Read sensory data and convert the to observation 
  def getUpdate( self):
    image = self.camera_client.getImage()
    # in Gazebo/JdeRobot sometimes the camera image data does not update and frame freezes, to avoid srcwing up the training:
    while np.array_equal(image.data ,self.image.data):
      image = self.camera_client.getImage() 
    self.image = image
    
    # If you want to skip some frames
    #for i in range(self.frames_skip):
    #  while image.timeStamp == self.camera_client.getImage().timeStamp:
    #    pass
    #  image = self.camera_client.getImage()
    
    # Using laser for collision detection
    laser = self.laser_client.getLaserData()
    self.collision = False
    if np.min( laser.values) < self.obstale_threshold:
      self.collision = True
    self.timeStamp = image.timeStamp
    self.screen = cv2.resize(image.data, (self.observation_dims[0],self.observation_dims[1]))
    
