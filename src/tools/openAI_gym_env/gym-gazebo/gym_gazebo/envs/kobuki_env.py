#Gym stuff
import gym
from gym import error, spaces, utils
from gym.utils import seeding

#JdeRobot stuff
import easyiceconfig as EasyIce
import jderobotComm as comm
from jderobotTypes import LaserData

#The rest
import numpy as np
import random
import sys

class KobukiEnv( gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__( self):
    #initializing laser scanner from config file:
    ic = EasyIce.initialize(["KobukiEnv", "kobuki_conf.cfg"])
    ic, node = comm.init(ic)
    robot_laser = comm.getLaserClient(ic, "kobuki.Laser")
    
    #initializing the environment:
    laser = robot_laser.getLaserData()
    self.observation_dims = [ len(laser.values)]
    self.observation_space = spaces.Discerete( self.observation_dims[0])
    print "Inited !"

  
