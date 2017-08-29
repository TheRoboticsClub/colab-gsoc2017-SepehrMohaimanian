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

class KobukiRGBEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    ic = EasyIce.initialize(["KobukiRGBEnv", "gym_kobuki_conf.cfg"])
    ic, node = comm.init(ic)
    #initializing laser scanner from config file:
    self.laser_client = comm.getLaserClient(ic, "kobuki.Laser")
    #initializing motors from config file:
    self.motors_client = comm.getMotorsClient(ic, "kobuki.Motors")
    #initializing gazebo resetter from config file:
    self.gazebo_resetter =  comm.getGazeboActionClient(ic, "kobuki.Reset")  
    #initializing camera from config file:
    self.camera_client =  comm.getCameraClient(ic, "kobuki.Camera")  
    
    #initializing the environment:
    laser = self.laser_client.getLaserData()
    print "--------------------------------"
    print laser
    self.observation_dims = [ len(laser.values), len(laser.values)]
    self.observation_space = spaces.Box(low=0, high=255, shape=( self.observation_dims[0], self.observation_dims[1], 1))
    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1],3), np.uint8)
    self.image = self.camera_client.getImage()
    self.action_space = spaces.Discrete( 3)
    self.viewer = None
    self.collision = False
    self.obstale_threshold = 0.5
    self.timeStamp = 0.0
    self.frames_skip = 0   
    self.display= True
    self.pixel_thresh = 60
    self.l = 0
    self.r = 0
    print "Inited !"

  def _step(self, action):
    self.actionToVel( action)
    self.getUpdate()
    reward = 0
    if self.collision:
      reward = -99
    else:
      if action == 1:
        reward = 0.6+(self.r*0.9 + self.l*0.9) 
      else:
        reward = -0.03
        #extra = 0
        #if self.r == 2 and self.l == 2:
        #  extra = 1
        #reward = -0.03+extra
       # if self.r == -1 and self.l == -1:
       #   reward = 0.03
       # elif self.r == -1:
       #   if action == 0:
       #     reward = 0.9
       #   else:
       #     reward = -0.03
       # else:
       #   if action == 0:
       #     reward = -0.03
       #   else:
       #     reward = 0.9

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
    img = self.screen
    if mode == 'human':
      from gym.envs.classic_control import rendering
      if self.viewer is None:
        self.viewer = rendering.SimpleImageViewer()
      self.viewer.imshow(img)

  def actionToVel( self, action):
    action -= 1
    vel = CMDVel()
    if action == 0:
      vel.vx = 0.3
    else:
      vel.vx = 0.1
    vel.az = action*1.2
    self.motors_client.sendVelocities(vel)
  
  def getUpdate( self):
    image = self.camera_client.getImage()
    #while self.timeStamp == self.camera_client.getImage().timeStamp:
    #  pass
    while np.array_equal(image.data ,self.image.data):
      image = self.camera_client.getImage() 
    self.image = image
    #for i in range(self.frames_skip):
    #  while image.timeStamp == self.camera_client.getImage().timeStamp:
    #    pass
    #  image = self.camera_client.getImage()
    r_raw = sum(image.data[-1,320:,:] > self.pixel_thresh) 
    if max(r_raw) == 0:
      self.r = -1
    else:
      self.r = 3-sum( r_raw == max(r_raw))
    l_raw = sum(image.data[-1,:320,:] > self.pixel_thresh) 
    if max(l_raw) == 0:
      self.l = -1
    else:
      self.l = 3-sum( l_raw == max(l_raw))
    laser = self.laser_client.getLaserData()
    self.collision = False
    if np.min( laser.values) < self.obstale_threshold:
      self.collision = True
    self.timeStamp = image.timeStamp
    self.screen = cv2.resize(image.data, (self.observation_dims[0],self.observation_dims[1]))
    
