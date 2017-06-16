#Gym stuff
import gym
from gym import error, spaces, utils
from gym.utils import seeding

#JdeRobot stuff

#The rest
import numpy as np
import random

class KobukiEnv( gym.Env):
  def __init__( self):
    print "initialized"
