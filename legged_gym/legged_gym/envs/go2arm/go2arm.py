from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch

from legged_gym.envs import LeggedRobot
from legged_gym import LEGGED_GYM_ROOT_DIR

from .go2arm_config import Go2armCfg

class Go2arm(LeggedRobot):
    cfg : Go2armCfg
    def __init__(self, cfg, sim_params, physics_engine, sim_device, headless):
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless)
        
        # print the list of all the joints name
        for jnt in self.dof_names:
            print('- '+jnt)