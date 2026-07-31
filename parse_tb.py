import sys
import os
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

event_file = "logs/skrl/dh_hand_catch/PPO/events.out.tfevents.1785398383.ubuntu2204.673893.0"
ea = EventAccumulator(event_file)
ea.Reload()

if "Reward / Total reward (mean)" in ea.scalars.Keys():
    rewards = ea.scalars.Items("Reward / Total reward (mean)")
    print("Total reward (mean):")
    for r in rewards[::max(1, len(rewards)//10)]:
        print(f"Step {r.step}: {r.value:.2f}")
    if rewards:
        print(f"Final Step {rewards[-1].step}: {rewards[-1].value:.2f}")
else:
    print("No reward data found in the specified key. Available keys:", ea.scalars.Keys())
