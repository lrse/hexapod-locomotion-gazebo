import time
import torch 
from stable_baselines3 import PPO
from env import Phantomx
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import BaseCallback

import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from torchvision.transforms import ToTensor

class CustomDataset(Dataset):
    def __init__(self, transform=None, target_transform=None):
        self.actions = []
        self.observations = [] 
        self.transform = transform
        self.target_transform = target_transform

    def __len__(self):
        return len(self.observations)

    def __getitem__(self, idx):
        observation = self.observations[idx]
        action = self.actions[idx]
        if self.transform:
            observation = self.transform(observation)
        if self.target_transform:
            action = self.target_transform(action)
        return observation, action
    
    def __additem__(self, observation, action):
        # This function was added to have a dynamically growing dataset
        self.observations.append(observation)
        self.actions.append(action)

def train(dataloader, model, loss_fn, optimizer, device):
    size = len(dataloader.dataset)
    model.train()
    for batch, (X, y) in enumerate(dataloader):
        X, y = X.to(device), y.to(device)

        # Compute prediction error
        pred = model(X)
        loss = loss_fn(pred, y)

        # Backpropagation
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        if batch % 100 == 0:
            loss, current = loss.item(), (batch + 1) * len(X)
            print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")


if __name__ == '__main__':

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Device: ", device)
    # Load trained model:
    model_teacher = PPO.load("phantomx.zip", env=None, verbose=True)
    print("Loaded existing teacher model")
    

    # Student hyperparamets:
    max_timesteps_per_episode = 400#1e3
    lr = 5e-4
    # learning_rate_decay = exp(0.995, 100) # DUDA: cómo implemento esto?
    batch_size = int(20e3)
    minibatches = 5
    epochs = 4
    total_iterations = 400
    print("max_timesteps_per_episode", max_timesteps_per_episode)
    print("lr", lr)
    print("learning_rate_decay: NOT IMPLEMENTED")
    print("batch_size", batch_size)
    print("minibatches", minibatches, " (Still not used)")
    print("epochs", epochs)
    print("total_iterations", total_iterations)
    # Create the environment
    env_student = Monitor(Phantomx(max_timesteps_per_episode), 
                  filename="logs", allow_early_resets=True)
    vec_env_student = DummyVecEnv([lambda: env_student])
    vec_env_student = VecNormalize.load("statistic.pkl", vec_env_student)
    model_student = "MlpPolicy" #CustomActorCriticPolicyStudent

    Ntraj = int(6) # Number of trajectories per iteration
    loss_fn = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model_student.parameters(), lr=lr)
    # Initialize dataset
    training_data = CustomDataset()
    for it in range(0,total_iterations):
        for traj in range(0,Ntraj):
            obs, _ = vec_env_student.reset()
            for n_step in range(max_timesteps_per_episode):
                action_teacher, action_probs_teacher = model_teacher.predict(obs, deterministic=True)
                action_student = model_student(obs)
                # Add tuple of observation and teacher action:
                training_data.__additem__(obs, action_teacher)
                if it == 0:
                    obs, rewards, terminated, truncated , info = vec_env_student.step(action_teacher)
                else:
                    obs, rewards, terminated, truncated , info = vec_env_student.step(action_student)
                if truncated or terminated:
                    print("Episode finished after {} timesteps".format(n_step+1))
                    # Add final state:
                    action_teacher, _ = model_teacher.predict(obs, deterministic=True)
                    training_data.__additem__(obs, action_teacher)
                    break
            
        # Here we train a new student policy on the entire set of trajectories.
        # Create data loader (I think we need to re-do this everytime we modify the training data)
        train_dataloader = DataLoader(training_data, batch_size=batch_size)
        for t in range(epochs):
            print(f"Epoch {t+1}\n-------------------------------")
            train(train_dataloader, model_student, loss_fn, optimizer)
        print("Finished training iteration ", it)

    # Once we're done training the student, we save it:
    # model_student.save("student_model") 
    torch.save(model_student.state_dict(), "model_student.pth")
    print("Saved PyTorch Student Model State to model_student.pth")