#Gym stuff
import gym
from gym import error, spaces, utils
from gym.utils import seeding

#JdeRobot stuff
import easyiceconfig as EasyIce
import jderobotComm as comm
from jderobotTypes import LaserData
from jderobotTypes import CMDVel
from parallelIce.cameraClient import CameraClient

#The rest
import numpy as np
import random
import sys
import time
import cv2

class KobukiRGBEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    ic = EasyIce.initialize(["KobukiRGBEnv", "/home/sepehr/Dev/colab-gsoc2017-SepehrMohaimanian/src/tools/openAI_gym_env/gym-gazebo/jde_gym_gazebo/envs/kobuki_conf.cfg"])
    ic, node = comm.init(ic)
    #initializing laser scanner from config file:
    self.laser_client = comm.getLaserClient(ic, "kobuki.Laser")
    #initializing motors from config file:
    self.motors_client = comm.getMotorsClient(ic, "kobuki.Motors")
    #initializing gazebo resetter from config file:
    self.gazebo_resetter =  comm.getGazeboActionClient(ic, "kobuki.Reset")  
    #initializing camera from config file:
    self.camera_client = CameraClient(ic, "kobuki.Camera", True)  
    
    #initializing the environment:
    laser = self.laser_client.getLaserData()
    print "--------------------------------"
    print laser
    self.observation_dims = [ len(laser.values), len(laser.values)]
    self.observation_space = spaces.Box(low=0, high=255, shape=( self.observation_dims[0], self.observation_dims[1], 1))
    self.screen = np.zeros((self.observation_dims[0],self.observation_dims[1],3), np.uint8)
    self.action_space = spaces.Discrete( 3)
    self.collision = False
    self.obstale_threshold = 0.5
    self.timeStamp = 0.0
    self.frames_skip = 0   
    self.display= False
    self.pixel_thresh = 60
    self.l = 0
    self.r = 0
    self.image = self.camera_client.getImage()
    print "Inited !"

  def _step(self, action):
    self.actionToVel( action)
    self.getUpdate()
    reward = 0
    if self.collision:
      reward = -99
    else:
      if action == 1:
        reward = 0.6+(self.r*0.075 + self.l*0.075) 
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
    time.sleep(0.3)
    self.getUpdate()
    return self.screen, 0,0 

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
    #while self.timeStamp == self.camera_client.getImage().timeStamp:
    #  pass
    image = self.camera_client.getImage()
    while image is None or np.array_equal(image ,self.image):
      image = self.camera_client.getImage()
    self.image = image
    #for i in range(self.frames_skip):
    #  while image.timeStamp == self.camera_client.getImage().timeStamp:
    #    pass
    #  image = self.camera_client.getImage()
    r_raw = sum(self.image[-1,320:,:] > self.pixel_thresh) 
    self.r = sum( r_raw == max(r_raw))-1 
    l_raw = sum(self.image[-1,:320,:] > self.pixel_thresh) 
    self.l = sum( l_raw == max(l_raw))-1 
    laser = self.laser_client.getLaserData()
    self.collision = False
    if np.min( laser.values) < self.obstale_threshold:
      self.collision = True
    #self.timeStamp = image.timeStamp
    self.screen = cv2.resize(self.image, (self.observation_dims[0],self.observation_dims[1]))
    if self.display:
      cv2.imshow("Screen", cv2.cvtColor(self.screen, cv2.COLOR_BGR2RGB))
      cv2.waitKey(100)
    
