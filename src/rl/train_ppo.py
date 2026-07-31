import argparse
import sys
import os

# Ensure src module is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

# Parse arguments
parser = argparse.ArgumentParser("DH Hand PPO Training")
parser.add_argument("--headless", action="store_true", default=False, help="Run headless")
args, unknown = parser.parse_known_args()

# Launch simulation app before importing other Omni modules
sim_app = SimulationApp({"headless": args.headless})

import torch
from omni.isaac.gym.vec_env import VecEnvBase
from src.rl.dh_hand_catch_task import DHHandCatchTask

try:
    from rl_games.common import env_configurations, vecenv
    from rl_games.torch_runner import Runner
    RL_GAMES_AVAILABLE = True
except ImportError:
    RL_GAMES_AVAILABLE = False
    print("rl_games not installed. Please pip install rl_games to run PPO.")

def main():
    if not RL_GAMES_AVAILABLE:
        sim_app.close()
        sys.exit(1)

    print("Initializing DHHandCatchTask...")
    task = DHHandCatchTask(name="DHHandCatchTask", num_envs=256, env_spacing=3.0)
    
    print("Wrapping in OmniIsaacGymEnvs VecEnvBase...")
    env = VecEnvBase(headless=args.headless)
    env.set_task(task, backend="torch")
    
    # Register environment in rl_games
    env_configurations.register('rlgpu', {
        'vecenv_type': 'RLGPU',
        'env_creator': lambda **kwargs: env
    })
    vecenv.register('RLGPU', lambda config_name, num_actors, **kwargs: env)

    # RL Games Config (PPO) for Isaac Sim
    rlg_config_dict = {
        'params': {
            'algo': {
                'name': 'a2c_continuous'
            },
            'model': {
                'name': 'continuous_a2c_logstd'
            },
            'network': {
                'name': 'actor_critic',
                'separate': False,
                'space': {
                    'continuous': {
                        'mu_activation': 'None',
                        'sigma_activation': 'None',
                        'mu_init': {
                            'name': 'default'
                        },
                        'sigma_init': {
                            'name': 'const_initializer',
                            'val': 0.0
                        },
                        'fixed_sigma': True
                    }
                },
                'mlp': {
                    'units': [256, 128, 64],
                    'activation': 'elu',
                    'd2rl': False,
                    'initializer': {
                        'name': 'default'
                    },
                    'regularizer': {
                        'name': 'None'
                    }
                }
            },
            'load_checkpoint': False,
            'load_path': '',
            'config': {
                'name': 'DHHandCatchTask',
                'env_name': 'rlgpu',
                'device': 'cuda:0',
                'device_name': 'cuda:0',
                'multi_gpu': False,
                'ppo': True,
                'mixed_precision': False,
                'normalize_input': True,
                'normalize_value': True,
                'value_bootstrap': True,
                'num_actors': 256,
                'reward_shaper': {
                    'scale_value': 1.0
                },
                'normalize_advantage': True,
                'gamma': 0.99,
                'tau': 0.95,
                'learning_rate': 3e-4,
                'lr_schedule': 'adaptive',
                'schedule_type': 'standard',
                'kl_threshold': 0.008,
                'score_to_win': 10000,
                'max_epochs': 5000,
                'save_best_after': 50,
                'save_frequency': 100,
                'print_stats': True,
                'grad_norm': 1.0,
                'entropy_coef': 0.0,
                'truncate_grads': True,
                'e_clip': 0.2,
                'horizon_length': 64,
                'minibatch_size': 1024,
                'mini_epochs': 4,
                'critic_coef': 2.0,
                'clip_value': True,
                'seq_len': 4,
                'bounds_loss_coef': 0.0001
            }
        }
    }
    
    print("Starting RL Games Runner...")
    runner = Runner()
    runner.load(rlg_config_dict)
    runner.reset()
    runner.run({
        'train': True,
        'play': False
    })

    sim_app.close()

if __name__ == '__main__':
    main()
