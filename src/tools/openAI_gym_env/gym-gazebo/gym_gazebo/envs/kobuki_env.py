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

class KobukiEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    ic = EasyIce.initialize(["KobukiEnv", "kobuki_conf.cfg"])
    ic, node = comm.init(ic)
    #initializing laser scanner from config file:
    self.laser_client = comm.getLaserClient(ic, "kobuki.Laser")
    #initializing motors from config file:
    self.motors_client = comm.getMotorsClient(ic, "kobuki.Motors")
    #initializing gazebo resetter from config file:
    self.gazebo_resetter =  = comm.getGazeboActionClient(ic, "kobuki.Reset")  
    
    #initializing the environment:
    laser = self.laser_client.getLaserData()
    self.observation_dims = [ len(laser.values)]
    self.observation_space = spaces.Discerete( self.observation_dims)
    self.laser = np.zeros(self.observation_dims, np.float32)
    self.action_space = spaces.Discrete( 3)
    self.collision = False
    self.obstale_threshold = 0.0
    print "Inited !"

  def _step(self, action):
    self.actionToVel( action)
    reward = 0
    if self.collision:
      reward = -99
    else:
      if action == 1:
        reward = 3
      else
        reward = 0.6
    info = {}
    self.getUpdate()
    return self.laser, reward, self.collision, info

  def _reset(self):
    vel = CMDVel()
    vel.vx = 0.0
    vel.az = 0.0
    self.motors_client.sendVelocities(vel)
    self.gazebo_resetter.sendReset() 

  def actionToVel( action):
    action -= 1
    vel = CMDVel()
    vel.vx = 0.3
    vel.az = action*0.3
    self.motors_client.sendVelocities(vel)
  
  def getUpdate( self):
    laser = self.laser_client.getLaserData()
    self.collision = False
    for i in range( len(laser.values)):
      self.laser[i] = float( laser.values[i])/laser.maxRange
      if self.laser[i] <= self.obstale_threshold:
        self.collision = True
