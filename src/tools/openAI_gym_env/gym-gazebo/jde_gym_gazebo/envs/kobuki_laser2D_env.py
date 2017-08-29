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

class KobukiLaser2DEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    ic = EasyIce.initialize(["KobukiLaser2DEnv", "gym_kobuki_conf.cfg"])
    ic, node = comm.init(ic)
    #initializing laser scanner from config file:
    self.laser_client = comm.getLaserClient(ic, "kobuki.Laser")
    #initializing motors from config file:
    self.motors_client = comm.getMotorsClient(ic, "kobuki.Motors")
    #initializing gazebo resetter from config file:
    self.gazebo_resetter =  comm.getGazeboActionClient(ic, "kobuki.Reset")  
    
    #initializing the environment:
    laser = self.laser_client.getLaserData()
    print "--------------------------------"
    print laser
    self.observation_dims = [ len(laser.values), len(laser.values)]
    self.observation_space = spaces.Box(low=0, high=255, shape=( self.observation_dims[0], self.observation_dims[1], 1))
    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1]), np.uint8)
    self.action_space = spaces.Discrete( 3)
    self.viewer = None
    self.collision = False
    self.obstale_threshold = 0.5
    self.timeStamp = 0.0
    self.frames_skip = 0   
    print "Inited !"

  def _step(self, action):
    self.actionToVel( action)
    self.getUpdate()
    reward = 0
    if self.collision:
      reward = -99
    else:
      if action == 1:
        reward = 0.9
      else:
        reward = -0.003
    info = {}
    return self.screen, reward, self.collision, info

  def _reset(self):
    vel = CMDVel()
    vel.vx = 0.0
    vel.az = 0.0
    self.motors_client.sendVelocities(vel)
    self.gazebo_resetter.sendReset()
    self.getUpdate()
    return self.screen, 0,0 

  def _render(self, mode='human', close=False):
    if close:
      if self.viewer is not None:
        self.viewer.close()
        self.viewer = None
      return
    img = np.concatenate((np.expand_dims(self.screen, axis=2), np.expand_dims(self.screen, axis=2),np.expand_dims(self.screen, axis=2)), axis=2)
    if mode == 'human':
      from gym.envs.classic_control import rendering
      if self.viewer is None:
        self.viewer = rendering.SimpleImageViewer()
      self.viewer.imshow(img)

  def actionToVel( self, action):
    action -= 1
    vel = CMDVel()
    if action == 0:
      vel.vx = 0.9
    else:
      vel.vx = 0.3
    vel.az = action*0.9
    self.motors_client.sendVelocities(vel)
  
  def getUpdate( self):
    while self.timeStamp == self.laser_client.getLaserData().timeStamp:
      pass
    laser = self.laser_client.getLaserData()
    for i in range(self.frames_skip):
      while laser.timeStamp == self.laser_client.getLaserData().timeStamp:
        pass
      laser = self.laser_client.getLaserData()
    self.collision = False
    if np.min( laser.values) < self.obstale_threshold:
      self.collision = True
    for i in range( len(laser.values)):
      value = int( (laser.values[i]/10.0)*self.observation_dims[1])
      self.screen[i] = np.concatenate( (np.ones(value)*255, np.zeros(180-value)))
    self.timeStamp = laser.timeStamp
