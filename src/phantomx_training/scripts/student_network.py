import torch as th
import torch.nn.functional as F
from torch import nn
from tcn import TemporalConvNet
import numpy as np
from gymnasium import spaces
from typing import Callable, Tuple
from stable_baselines3.common.policies import ActorCriticPolicy

"""
This class assumes:

	+ The observation has dimension (batch_size, num_channels=1, student_obs), where
	student_obs = h * history_state_dimension + robot_state_dimension

	+ student_obs is a vector respecting the following chronological order: [h_t-h, ..., h_t-1, o_t]
"""

class StudentNetwork(nn.Module):

	def __init__(self, robot_state_dim, history_state_dim, h, out_dim): # cambiar nombre

		super(StudentNetwork, self).__init__()
		self.init_param(robot_state_dim, history_state_dim, h, out_dim)
		
		self.tcn = TemporalConvNet(self.input_size, self.num_channels, kernel_size=self.kernel_size, dropout=self.dropout)
		self.linear_tcn = nn.Linear(self.num_channels[-1], self.latent_space_dimension)

		self.layer_mlp_1 = nn.Linear(self.robot_state_dim + self.latent_space_dimension, 256) 
		self.layer_mlp_2 = nn.Linear(256, 128)
		self.layer_mlp_3 = nn.Linear(128, 64)
		self.output_layer = nn.Linear(64, self.out_dim)

		# Simply for initialization. This value should never be read as None.
		self.tcn_latent = None

	def init_param(self, robot_state_dim, history_state_dim, h, out_dim):
		self.robot_state_dim = robot_state_dim
		self.h = h
		self.history_state_dim = history_state_dim
		self.tcn_input_dimension = history_state_dim * h
		self.out_dim = out_dim
		self.latent_space_dimension = 64
		self.latent_dim_pi = out_dim
		self.latent_dim_vf = out_dim
		self.input_size = 1
		self.num_channels = [44] * 6
		self.kernel_size = 5
		self.dropout = 0.05 

	def forward(self, obs: th.Tensor) -> th.Tensor:

		if isinstance(obs, np.ndarray) or isinstance(obs, type([])):
			obs = th.tensor(obs, dtype=th.float)

		robot_history = obs[:, :, :self.history_state_dim * self.h]
		obs_t = obs[:, :, self.history_state_dim * self.h:]

		y1 = self.tcn(robot_history)
		self.tcn_latent = th.tanh(self.linear_tcn(y1[:, :, -1]))

		obs_t = obs_t.squeeze(1)
		full_vector = th.cat([obs_t, self.tcn_latent], -1)

		z1 = th.tanh(self.layer_mlp_1(full_vector))
		z2 = th.tanh(self.layer_mlp_2(z1))
		z3 = th.tanh(self.layer_mlp_3(z2))
		z4 = self.output_layer(z3) 

		return z4, self.tcn_latent

def generate_test_data(batch_size, input_size, robot_state_dimension, history_dimension, h):

	sequence_length = history_dimension * h + robot_state_dimension
	action_space_dimension = 10

	random_tensor = np.random.rand(batch_size, input_size, sequence_length) # 5 batches of 11 elements

	out, latent = StudentNetwork(robot_state_dimension, history_dimension, h, action_space_dimension)(random_tensor)

	print(f"Out has type {type(out)}")
	print(f"Out has shape: {out.shape}")

	print(f"Latent has type {type(latent)}")
	print(f"Latent has shape: {latent.shape}")


#generate_test_data(batch_size = 5, input_size = 1, robot_state_dimension = 5, history_dimension = 2, h = 3)

"""
class StudentActorCriticPolicy(ActorCriticPolicy):
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
		ot = self.observation_space.shape[0] - 84
		ht = ot - 12
		h  = 10
		action_space = self.action_space.shape[0]
		self.mlp_extractor = StudentNetwork(ot, ht, h, action_space)
"""