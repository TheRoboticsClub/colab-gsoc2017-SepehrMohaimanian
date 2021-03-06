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

class KobukiLaserEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    ic = EasyIce.initialize(["KobukiLaserEnv", "kobuki_conf.cfg"])
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
    self.observation_dims = [ len(laser.values)]
    self.observation_space = spaces.Discrete( self.observation_dims)
    self.laser = np.zeros(self.observation_dims, np.float32)
    self.action_space = spaces.Discrete( 3)
    self.collision = False
    self.obstale_threshold = 0.5
    self.timeStamp = 0.0
    print "Inited !"

  def _step(self, action):
    self.actionToVel( action)
    #time.sleep(0.03)
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
    return self.laser, reward, self.collision, info

  def _reset(self):
    vel = CMDVel()
    vel.vx = 0.0
    vel.az = 0.0
    self.motors_client.sendVelocities(vel)
    self.gazebo_resetter.sendReset()
    self.getUpdate()
    return self.laser, 0,0 

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
#    for i in range(self.frames_skip):
#      while laser.timeStamp == self.laser_client.getLaserData().timeStamp:
#        pass
#      laser = self.laser_client.getLaserData()
    self.collision = False
    if np.min( laser.values) < self.obstale_threshold:
      self.collision = True
    self.laser = laser.values
    self.timeStamp = laser.timeStamp
#    for i in range( len(laser.values)):
#      self.laser[i] = laser.values[i] #float( laser.values[i])/laser.maxRange
#      if self.laser[i] < self.obstale_threshold:
#        self.collision = True
