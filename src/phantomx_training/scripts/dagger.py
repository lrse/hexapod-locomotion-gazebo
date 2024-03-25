import os
import sys

import time
import torch
import random
import pickle

import numpy as np
from env_atc import PhantomxAtc

from stable_baselines3 import PPO
from torch.utils.data import DataLoader
from student_network import StudentNetwork
from student_dataset import StudentDataset
from trajectory_roller import TrajectoryRoller

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize



 #def _normalize_obs(self, obs: np.ndarray, obs_rms: RunningMeanStd) -> np.ndarray:
 #       """
 #       Helper to normalize observation.
 #       :param obs:
 #       :param obs_rms: associated statistics
 #       :return: normalized observation
 #       """
 #       return np.clip((obs - obs_rms.mean) / np.sqrt(obs_rms.var + self.epsilon), -self.clip_obs, self.clip_obs)


"""
This class implements Dagger algorithm for training a student network based on expert observations.

"""

class Dagger():

	def __init__(self, history_dim, 
			           robot_state_dim, 
					   privileged_info_dim, 
					   action_space_dim, 
					   h, 
					   attrb_mapper, 
					   n_traj, 
					   max_n_steps_traj, 
					   batch_size, 
					   n_epochs, 
					   n_policies, 
					   lr):

		self.history_dim = history_dim
		self.robot_state_dim = robot_state_dim
		self.privileged_info_dim = privileged_info_dim
		self.action_space_dim = action_space_dim
		self.h = h
		self.attrb_mapper = attrb_mapper
		self.n_traj = n_traj
		self.max_n_steps_traj = max_n_steps_traj
		self.lr = lr

		self.batch_size = batch_size
		self.n_epochs = n_epochs
		self.n_policies = n_policies

		with open("statistic.pkl", "rb") as file_handler:
			self.vec_normalized = pickle.load(file_handler)
		
		self.obs_rms_mean = self.vec_normalized.obs_rms.mean
		self.obs_rms_var = self.vec_normalized.obs_rms.var

		# Load the PhantomX ATC environment
		self.env = DummyVecEnv([lambda: PhantomxAtc(max_timesteps_per_episode=self.max_n_steps_traj)])
		self.env = VecNormalize.load("statistic.pkl", self.env)
		self.env.training = False
		self.env.norm_reward = False
		# Load the teacher model
		self.teacher_model = PPO.load("phantomx.zip", env=self.env)
		teacher_model_network = self.teacher_model.policy.mlp_extractor

		# Load the student model
		self.student_model = StudentNetwork(self.robot_state_dim, self.history_dim, self.h, self.action_space_dim)

		pytorch_total_params = sum(p.numel() for p in self.student_model.parameters() if p.requires_grad)
		print(f"Total number of parameters in the student model: {pytorch_total_params}")

		# Copy teacher parameters to MLP block.
		# TODO: Verify this is working.
		self.student_model.layer_mlp_1.load_state_dict(teacher_model_network.layer3.state_dict())
		self.student_model.layer_mlp_2.load_state_dict(teacher_model_network.layer4.state_dict())
		self.student_model.layer_mlp_3.load_state_dict(teacher_model_network.layer5.state_dict())
		self.student_model.output_layer.load_state_dict(teacher_model_network.layer6.state_dict())

		# Dataset and Trajectory Roller
		self.dataset = StudentDataset()
		self.trajectory_roller = TrajectoryRoller(self.dataset, 
											      self.history_dim, 
												  self.robot_state_dim, 
												  self.privileged_info_dim, 
												  self.h, 
												  self.env, 
												  self.teacher_model, 
												  self.student_model, 
												  self.attrb_mapper,
												  self.obs_rms_mean,
												  self.obs_rms_var)

		# Create optimizer
		self.optim = torch.optim.Adam(self.student_model.parameters(), lr=self.lr)

	"""

	Loss function used for training. Supports batching.

	Input Dimensions: (batch_size, _)
 
	"""
	def loss_function(student_actions, teacher_actions, student_latents, teacher_latents):
		action_loss = torch.sum((student_actions - teacher_actions) ** 2)
		latent_loss = torch.sum((student_latents - teacher_latents) ** 2)
		return action_loss + latent_loss

	"""
	Trains for one epoch
	"""
	def train_epoch(self, epoch_idx, data_loader):

		running_loss = 0.
		last_loss = 0.
		
		# Loop through every batch of data
		for batch_idx, batch_data in enumerate(data_loader):

			print(f"Epoch: {epoch_idx}, Batch: {batch_idx}")

			student_observation_batch, teacher_action_batch, teacher_latent_batch = batch_data

			# Sanity checks
			assert student_observation_batch.shape[1] == (self.h * self.history_dim + self.robot_state_dim)
			assert teacher_action_batch.shape[1] == self.action_space_dim
			assert teacher_latent_batch.shape[1] == 64

			# Zero gradients for every batch
			self.optim.zero_grad()

			# Format inputs for feeding into student_model. (batch_size, 1, _)
			reshaped_student_obs = torch.unsqueeze(student_observation_batch, 1)

			assert reshaped_student_obs.shape[1] == 1
			assert reshaped_student_obs.shape[2] == (self.h * self.history_dim + self.robot_state_dim)

			# Make predictions for this batch
			student_action_batch, student_latent_batch = self.student_model(reshaped_student_obs)

			# Sanity checks
			assert student_action_batch.shape[1] == self.action_space_dim
			assert student_latent_batch.shape[1] == 64
			assert student_action_batch.shape[0] == student_latent_batch.shape[0]

			# Compute the loss and its gradients
			loss = self.loss_function(student_action_batch, teacher_action_batch, student_latent_batch, teacher_latent_batch)
			loss.backward()

			# Adjust learning weights
			self.optim.step()

			# Gather data and report. Note that this is minimal!!! We MUST implement better observability for training.
			running_loss += loss.item()
			if batch_idx % 100 == 99:
				last_loss = running_loss / 100 # loss per batch
				print('  batch {} loss per batch: {}'.format(batch_idx + 1, last_loss))
				#tb_x = epoch_index * len(training_loader) + i + 1
				#tb_writer.add_scalar('Loss/train', last_loss, tb_x)
				#running_loss = 0.

	def train(self):

		for n_policy in range(self.n_policies): # Total number of student policies to train.

			# We do not want to compute gradients while rolling trajectories.
			self.student_model.train(False)

			# Extend existing dataset for training the i-th policy.
			if n_policy == 0: 
				self.trajectory_roller.roll(self.n_traj, self.max_n_steps_traj, use_teacher = True)
			else: 
				self.trajectory_roller.roll(self.n_traj, self.max_n_steps_traj, use_teacher = False)


			# Turn training on back again
			self.student_model.train(True)

			# Create a shuffled data loader on the most recent dataset
			data_loader = DataLoader(dataset = self.dataset, batch_size=self.batch_size, shuffle=True)

			for epoch in range(self.n_epochs):

				self.train_epoch(epoch, data_loader)

########################################### UNIT TESTING ###########################################

def testDataLoader():

	# Example data
	student_observation = [[1, 2, 3], [3, 4, 3], [5, 6, 3], [7, 8, 3]]
	teacher_action = [[1, 2], [3, 4], [5, 6], [7, 8]]
	teacher_latent = [[1, 2], [3, 4], [5, 6], [7, 8]]

	dataset = StudentDataset()

	# Add all items to the Custom Dataset
	for idx in range(len(student_observation)):
		dataset.__additem__(student_observation[idx], teacher_action[idx], teacher_latent[idx])

	# Create a shuffled data loader on our new dataset
	data_loader = DataLoader(dataset = dataset, batch_size = 2, shuffle = True)

	# Enumerate over the DataLoader
	for index, batch_data in enumerate(data_loader):
		student_observation_batch, teacher_action_batch, teacher_latent_batch = batch_data
		print(f"Iteration {index}")
		print("Student Observation Batch:", student_observation_batch)
		print("Student Observation Batch shape:", student_observation_batch.shape) # Student Observation Batch shape: torch.Size([2, 3]). 
		print("Teacher Action Batch:", teacher_action_batch)
		print("Teacher Latent Batch:", teacher_latent_batch)
		print("---")


def testUnsqueeze():
	# Example tensor of shape (N, M)
	original_tensor = torch.randn(3, 4) 

	print(original_tensor)

	# Convert it to shape (N, 1, M)
	reshaped_tensor = torch.unsqueeze(original_tensor, 1)

	# Print the resulting tensor shape
	print(reshaped_tensor.shape)
	print(reshaped_tensor)

	print(reshaped_tensor[0][0])

def testLoss():
	# Example tensors

	batch_size = 10
	N = 5

	student_actions = torch.randn(batch_size, N)
	teacher_actions = torch.randn(batch_size, N)

	print(student_actions)
	print(teacher_actions)

	print(student_actions - teacher_actions)

	# Compute the element-wise difference, square it, and sum up for each batch
	difference_squared_sum = torch.sum((student_actions - teacher_actions) ** 2)

	# Compute the element-wise difference, square it, and sum up for each batch
	difference_squared_sum2 = torch.sum((student_actions - teacher_actions) ** 2)

	# Print the result
	print(difference_squared_sum + difference_squared_sum2)

def test_load_weights():
	teacher_model = PPO.load("phantomx.zip")
	teacher_model_network = teacher_model.policy.mlp_extractor
	print(type(teacher_model_network.layer3.state_dict()))

#test_load_weights()

"""

TODO: 

Test the entire thing.
Make graphs available for training.

Nico: Get obs parts.

"""

if __name__ == "__main__":



	Dagger(history_dim=181,
		   robot_state_dim=181,
		   privileged_info_dim=84,
		   action_space_dim=24,
		   h=5,
		   attrb_mapper=None,
		   n_traj=10,
		   max_n_steps_traj=1000,
		   batch_size=32,
		   n_epochs=10,
		   n_policies=10,
		   lr=0.001).train()
		  
	