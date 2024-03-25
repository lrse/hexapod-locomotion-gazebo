import time
import torch 
from env_atc import PhantomxAtc
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import BaseCallback
from custom_network import CustomActorCriticPolicy


"""
    - train /approx_kl: A higher value of KL divergence indicates a larger difference between the current policy and the old policy.
                      The goal of reinforcement learning algorithms is to minimize the KL divergence between the current policy and 
                      the old policy to ensure stability and smooth policy updates.

    - train/clip_fraction: graphics in Tensorboard shows the percentage of values that were clipped during the training process.

    - train /clip_range: In the PPO algorithm, the policy update is constrained to a certain range to prevent large policy updates 
                         that could lead to instability. The clip_range parameter determines the range within which the policy update 
                         is clipped. If the ratio between the new and old policy probabilities exceeds the clip_range, the update is 
                         clipped to the clip_range value. This helps to ensure stability during training.

    - train/entropy_loss: graph represents the entropy loss during training. Entropy loss is a term used in reinforcement learning 
                          algorithms that encourages exploration by adding entropy regularization to the policy optimization objective. 
                          It measures the uncertainty or randomness of the policy's action distribution. Higher entropy means the policy 
                          is more random and exploratory, while lower entropy means the policy is more deterministic and exploitative.

        Here are some key points to understand about the "train/entropy_loss" graph:

        - The entropy loss is a component of the total loss function used in policy optimization algorithms like 
          Proximal Policy Optimization (PPO) and Soft Actor-Critic (SAC).

        - The entropy loss term is used to balance exploration and exploitation in reinforcement learning. 
          It encourages the policy to explore different actions and prevents it from becoming too deterministic.

        - The entropy loss is usually represented as a negative value because it is maximized rather than minimized. 
          Maximizing the entropy loss encourages the policy to have a more diverse action distribution.

        - The "train/entropy_loss" graph shows the value of the entropy loss over time during training. 
          It provides insights into how the entropy loss changes as the policy learns and adapts.

        - Monitoring the entropy loss can help in understanding the trade-off between exploration and exploitation 
          in the RL agent's behavior. It can also be used to analyze the convergence and stability of the training process.

    
    - train/explained_variance: represents the explained variance of the returns at each iteration during training. 
                                Explained variance is a measure of how well the model's predictions explain the variance in 
                                the observed data. In the context of reinforcement learning, it is used to assess the quality 
                                of the value function estimation. A higher explained variance indicates that the model is better 
                                at estimating the expected returns for different states and actions[5].

    Here are the steps to interpret the train/explained_variance graph:

        - Understand explained variance: Explained variance is a statistical measure that quantifies the proportion of variance in the 
          dependent variable (returns in this case) that can be explained by the independent variables (state and action). 
          It is calculated as 1 minus the ratio of the residual variance to the total variance. A value of 1 indicates that the model 
          perfectly predicts the returns, while a value of 0 indicates that the model provides no information about the returns.

        - Interpret the graph: The train/explained_variance graph in Tensorboard shows the explained variance of the returns over 
          time during the training process. It provides insights into how well the model is learning to predict the returns. 
          A higher value of explained variance indicates that the model is learning better and improving its predictions. On the other hand, 
          a lower value suggests that the model is struggling to capture the underlying patterns in the data.

        - Monitor convergence: The train/explained_variance graph can be used to monitor the convergence of the training process. As the 
          model learns and improves, the explained variance should increase and approach a value close to 1. If the explained variance 
          remains low or fluctuates, it may indicate that the model is not learning effectively or has reached a suboptimal solution.

        - Compare different models: The train/explained_variance graph can also be used to compare the performance of different models 
          or algorithms. By plotting the explained variance for multiple models on the same graph, you can easily compare their learning 
          progress and choose the one that performs better in terms of explained variance.

    - train/learning_rate: represents the learning rate of the policy optimizer during training. The learning rate is a hyperparameter.

    - train/loss: the "train/loss" graphics represent the model's training loss over time. The training loss is a measure of how well 
                  the model is able to minimize the difference between its predicted outputs and the true outputs in the training data. 
                  The lower the training loss, the better the model is at fitting the training data.

    - train/policy_gradient_loss: represents the policy gradient loss during the training process. The policy gradient loss is a measure 
                                  of how well the policy network is learning to maximize the expected cumulative reward. It is computed 
                                  using the policy gradient algorithm, which uses the gradient of the policy network's output with respect 
                                  to its parameters to update the parameters in a way that maximizes the expected cumulative reward.

            Here are some key points to understand about the "train/policy_gradient_loss" graphics in Tensorboard:

                - The policy gradient loss is an important metric in reinforcement learning algorithms, such as Proximal Policy Optimization (PPO) and 
                  Twin Delayed Deep Deterministic Policy Gradient (TD3).

                - The policy gradient loss represents the discrepancy between the predicted action probabilities and the actual actions taken by the 
                  policy network during training.

                - A lower policy gradient loss indicates that the policy network is learning to choose actions that maximize the expected cumulative reward.

                - The policy gradient loss is typically minimized using gradient descent or other optimization algorithms.

                - Monitoring the policy gradient loss can help track the progress of the training process and identify any issues or anomalies that may arise.

    -train/value_loss: The value loss measures the difference between the estimated value and the actual value obtained during the training process. 
                       Minimizing the value loss helps the RL agent improve the accuracy of its value function estimation.

    -train/std: if decreases over iterations, it indicates that the model is becoming more stable and is converging towards an optimal solution. 
                On the other hand, if the training standard deviation remains high or increases, it suggests that the training process is unstable 
                and the model may not be learning effectively.

"""

class TensorboardCallback(BaseCallback):
    
  """  Custom callback for plotting additional values in tensorboard. """

  def __init__(self, verbose=0):
      super().__init__(verbose)

  def _on_step(self) -> bool:
      # Log scalar value (here a random variable)
      if 'stats' in self.locals["infos"][0]:
        #print(self.locals["infos"][0]['stats'])
        for k in self.locals["infos"][0]['stats']:
          self.logger.record("reward_component/" + k, self.locals["infos"][0]['stats'][k])
      return True
    
def main():

  device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
  print("Device: ", device)
  # Create the environment 
  max_timesteps_per_episode = 1000#400#
  env_r = Monitor(PhantomxAtc(max_timesteps_per_episode=max_timesteps_per_episode), 
                filename="logs", allow_early_resets=True)
  
  vec_env = DummyVecEnv([lambda: env_r])

  Ntraj = 6#6
  Nparticle = 1#10
  n_steps = int(Ntraj*Nparticle*max_timesteps_per_episode) #5000 #800 #
  # n_steps = int(5e3)#int(10*max_timesteps_per_episode)
  lr = 1e-3 #1e-4 #
  discount_factor = 0.995
  kl_d_threshold = 0.01
  cg_damping = 1e-1
  cg_iteration = 50
  batch_size = 2500#int(24e3) #400 #
  print("Ntraj", Ntraj)
  print("Nparticle", Nparticle)
  print("n_steps", n_steps)
  print("lr", lr)
  print("discount_factor", discount_factor)
  print("kl_d_threshold", kl_d_threshold)
  print("cg_damping", cg_damping)
  print("cg_iteration", cg_iteration)
  print("batch_size", batch_size)
    
  try:
      vec_env = VecNormalize.load("statistic.pkl", vec_env)
      model = PPO.load("phantomx.zip",
                        env=vec_env, 
                        device=device, 
                        learning_rate=lr,
                        # gamma=discount_factor,
                        n_steps=n_steps ,   
                        # target_kl=kl_d_threshold,  
                        batch_size=batch_size,
                        verbose=True)
      
      print("Loading existing model")

  except:
      vec_env = VecNormalize(vec_env, 
                              norm_obs = True, 
                              norm_reward = True, 
                              clip_obs = 35.0)
      model = PPO(CustomActorCriticPolicy, #"MlpPolicy", 
                  env=vec_env, 
                  device=device, 
                  learning_rate=lr,
                  # gamma=discount_factor,
                  n_steps=n_steps,     
                  # target_kl=kl_d_threshold,
                  batch_size=batch_size,
                  verbose=True,
                  tensorboard_log="./log/",)
      
      print("Creating new model")

  # CALLBACKS: Save a checkpoint every 5k steps
  checkpoint_callback = CheckpointCallback(
    save_freq=5000,
    save_path="./models/",
    name_prefix="rl_model",
    save_replay_buffer=True,
    save_vecnormalize=True,
  )
  # Train the agent
  model.learn(total_timesteps=4_000_000, progress_bar=True, callback=[checkpoint_callback, TensorboardCallback()])
  # Save the model
  model.save("phantomx")
  # Save and reload the VecNormalize object
  vec_env.save("statistic.pkl")

  # # Load the trained agent
  # model = PPO.load("phantomx.zip", env=vec_env, device=device)
  # # Enjoy trained agent
  # while True:
  #     obs = vec_env.reset()
  #     for i in range(1000):
  #         action, action_probs = model.predict(obs, deterministic=True)        
  #         obs, rewards, done, info = vec_env.step(action)
  #         time.sleep(0.02)
  #         if done:
  #             print("Episode finished after {} timesteps".format(i+1))
  #             break

if __name__ == '__main__':
# Access the optimizer and update learning rate
# optimizer = model.policy.optimizer
# new_learning_rate = 0.001  # New learning rate value
# optimizer.param_groups[0]["lr"] = new_learning_rate
# change the learning rate of the policy optimizer on runtime
# model.policy.optimizer.lr = 1e-4
    main()