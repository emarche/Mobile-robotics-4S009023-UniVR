"""Launch file for the testing of the discrete DDQN algorithm

"""
import os
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

from agent import DDQN

def main():
    agent = DDQN()

    # Train the agent
    agent.test(
        n_episodes=10
    )

if __name__ == "__main__":
    main()
