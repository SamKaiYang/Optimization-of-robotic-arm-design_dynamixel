import math
import rospy
import numpy as np
from pytorch_DQN.config import Config
from pytorch_DQN.core.logger_v2 import TensorBoardLogger
from pytorch_DQN.core.util import get_output_folder
import tensorboardX
import sys
import os
import datetime

curr_path = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在绝对路径
parent_path = os.path.dirname(curr_path)  # 父路径
sys.path.append(parent_path)  # 添加路径到系统路径
file_path = curr_path + "/outputs/" 
curr_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")  # 获取当前时间

tb = tensorboardX.SummaryWriter()
class Trainer:
    def __init__(self, agent, env, config: Config):
        self.agent = agent
        self.env = env
        self.config = config

        # non-Linear epsilon decay
        epsilon_final = self.config.epsilon_min
        epsilon_start = self.config.epsilon
        epsilon_decay = self.config.eps_decay
        self.epsilon_by_frame = lambda frame_idx: epsilon_final + (epsilon_start - epsilon_final) * math.exp(
            -1. * frame_idx / epsilon_decay)

        self.outputdir = get_output_folder(self.config.output, self.config.env)
        self.agent.save_config(self.outputdir)
        # self.board_logger = TensorBoardLogger(self.outputdir)

    def train(self, pre_fr=0):
        losses = []
        all_rewards = []
        episode_reward = 0
        ep_num = 0
        is_win = False

        state = self.env.reset()
        for fr in range(pre_fr + 1, self.config.frames + 1):
            epsilon = self.epsilon_by_frame(fr)
            action = self.agent.act(state, epsilon)

            next_state, reward, done, _ = self.env.step(action)
            self.agent.buffer.add(state, action, reward, next_state, done)

            state = next_state
            episode_reward += reward

            loss = 0
            if self.agent.buffer.size() > self.config.batch_size:
                loss = self.agent.learning(fr)
                losses.append(loss)
                # self.board_logger.scalar_summary('Loss per frame', fr, loss)
                tb.add_scalar("/trained-model/Loss_per_frame/", loss, fr)
                
            if fr % self.config.print_interval == 0:
                # print("frames: %5d, reward: %5f, loss: %4f episode: %4d" % (fr, np.mean(all_rewards[-10:]), loss, ep_num))
                rospy.loginfo('frames: {}, reward: {}, loss: {} episode: {}'.format(fr, np.mean(all_rewards[-10:]), loss, ep_num))
            if fr % self.config.log_interval == 0:
                # self.board_logger.scalar_summary('Reward per episode', ep_num, all_rewards[-1])
                tb.add_scalar("/trained-model/Reward_per_episode/", all_rewards[-1], ep_num)
            if self.config.checkpoint and fr % self.config.checkpoint_interval == 0:
                self.agent.save_checkpoint(fr, self.outputdir)

            if done:
                state = self.env.reset()
                all_rewards.append(episode_reward)
                episode_reward = 0
                ep_num += 1
                avg_reward = float(np.mean(all_rewards[-100:]))
                # self.board_logger.scalar_summary('Best 100-episodes average reward', ep_num, avg_reward)
                tb.add_scalar("/trained-model/Best_100_episodes_average_reward/", avg_reward, ep_num)
                if len(all_rewards) >= 100 and avg_reward >= self.config.win_reward and all_rewards[-1] > self.config.win_reward:
                    is_win = True
                    self.agent.save_model(self.outputdir, 'best')
                    # print('Ran %d episodes best 100-episodes average reward is %3f. Solved after %d trials ✔' % (ep_num, avg_reward, ep_num - 100))
                    rospy.loginfo('Ran {} episodes best 100-episodes average reward is {}. Solved after {} trials ✔'.format(ep_num, avg_reward, ep_num - 100))
                    if self.config.win_break:
                        break

        if not is_win:
            print('Did not solve after %d episodes' % ep_num)
            self.agent.save_model(self.outputdir, 'last')
