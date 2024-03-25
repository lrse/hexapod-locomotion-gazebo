import torch
import numpy as np
from torch import nn
import torch.nn.functional as F

from typing import Callable, Dict, List, Optional, Tuple, Type, Union

from gymnasium import spaces
import torch as th
from torch import nn

from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy

class CustomNetwork(nn.Module):
    
    """
    
    robot_state_dim: The dimenion of the Robot state vector o_t.

    privileged_info_dim: The dimension of the Privileged Information vector x_t.

    out_dim: The dimension of the action space.

    """
    def __init__(self, robot_state_dim, privileged_info_dim, out_dim):
        
        super(CustomNetwork, self).__init__()

        self.robot_state_dim = robot_state_dim
        self.privileged_info_dim = privileged_info_dim
        self.out_dim = out_dim
   
        # Save output layer dimensions, used to create the distributions 
        # This is used internally by Stable Baselines 3.  
        self.latent_dim_pi = out_dim    
        self.latent_dim_vf = out_dim
        # Defines the dimension of the encoding (latent) space for the privileged information
        self.latent_space_dimension = 64

        self.layer1 = nn.Linear(privileged_info_dim, 72)
        self.layer2 = nn.Linear(72, self.latent_space_dimension)
        self.layer3 = nn.Linear(robot_state_dim + self.latent_space_dimension, 256)
        self.layer4 = nn.Linear(256, 128)
        self.layer5 = nn.Linear(128, 64)
        self.layer6 = nn.Linear(64, out_dim)  # out_dim = 24, latent_dim = 64
        
    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, obs: th.Tensor) -> th.Tensor:
        # Convert observation to tensor if it's a numpy array
        if isinstance(obs, np.ndarray):
            obs = torch.tensor(obs, dtype=torch.float)
        
        # dim = obs.shape[0]
        # # Remove axis with dimension 1
        # if obs.shape[0] == 1:
        #     # Si no esta en bacht mode, es decir, que esta en el step el vector de obs es de dimension (1, ...)
        #     obs = torch.squeeze(obs)
        dim = obs.shape[1]

        # Step 1) Split our tensor into privileged information and robot state
        robot_state = obs[:,:self.robot_state_dim]
        privileged_information = obs[:,self.robot_state_dim:]

        # Calculate the latent (encoded) vector
        activation1 = torch.tanh(self.layer1(privileged_information))
        latent_vector = torch.tanh(self.layer2(activation1))

        # Concatenate (robot state, latent vector)
        full_vector = torch.cat([robot_state, latent_vector], -1)

        # Remaining layers
        z1 = torch.tanh(self.layer3(full_vector))
        z2 = torch.tanh(self.layer4(z1))
        z3 = torch.tanh(self.layer5(z2))
        z4 = self.layer6(z3) 

        if dim > 1: 
            # assert(z4.shape[0] == 400) #Assert batch size
            assert(z4.shape[1] == 24) #Assert output dim

        # Output
        return z4
        #return torch.cat([self.layer6(z3), latent_vector])
        
    # TODO: Do we want to use the same architecture for the critic?
    def forward_critic(self, obs: th.Tensor) -> th.Tensor:
        return self.forward_actor(obs)

class CustomActorCriticPolicy(ActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        *args,
        **kwargs,
    ):
        # Disable orthogonal initialization
        kwargs["ortho_init"] = False
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            # Pass remaining arguments to base class
            *args,
            **kwargs,
        )


    def _build_mlp_extractor(self) -> None:
        # TODO: Define robot_state_dim and privileged_info_dim with correct values.
        self.mlp_extractor = CustomNetwork(self.observation_space.shape[0]-105, 105, self.action_space.shape[0])

"""

# Example

obs = np.array([1,2,3,4,5,6,7,8,9])

robot_state_dim = 4
privileged_info_dim = 5
out_dim = 2

NN = CustomNetwork(robot_state_dim, privileged_info_dim, out_dim)
NN.forward_actor(obs)

"""