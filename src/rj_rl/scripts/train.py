"""Training entry point for the RoboCup RL system.

Usage:
    python -m scripts.train
    python -m scripts.train --timesteps 50000 --seed 123
"""
import argparse
import sys

import numpy as np

from rj_rl.config import RLConfig
from rj_rl.trainer import Trainer


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train an RL agent for RoboCup SSL strategy."
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=100000,
        help="Total training timesteps.",
    )
    parser.add_argument(
        "--seed", type=int, default=42, help="Random seed."
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="checkpoints",
        help="Directory for saving model checkpoints.",
    )
    parser.add_argument(
        "--eval-episodes",
        type=int,
        default=10,
        help="Number of evaluation episodes after training.",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=3e-4,
        help="Learning rate.",
    )

    args = parser.parse_args()

    config = RLConfig()
    config.training.total_timesteps = args.timesteps
    config.training.seed = args.seed
    config.training.learning_rate = args.lr

    np.random.seed(args.seed)

    print("=" * 60)
    print("RoboCup SSL Reinforcement Learning Training")
    print("=" * 60)
    print(f"  Timesteps:     {args.timesteps}")
    print(f"  Seed:          {args.seed}")
    print(f"  Learning rate: {args.lr}")
    print(f"  Save dir:      {args.save_dir}")
    print("=" * 60)

    trainer = Trainer(config=config, save_dir=args.save_dir)
    results = trainer.train(total_timesteps=args.timesteps)

    print("\n" + "=" * 60)
    print("Training complete. Running evaluation...")
    print("=" * 60)

    eval_results = trainer.evaluate(n_episodes=args.eval_episodes)
    print(f"  Mean reward:     {eval_results['mean_reward']:.3f}")
    print(f"  Std reward:      {eval_results['std_reward']:.3f}")
    print(f"  Goals scored:    {eval_results['goals_scored']}")
    print(f"  Goals conceded:  {eval_results['goals_conceded']}")

    print(f"\nModel saved to: {args.save_dir}/policy_final")


if __name__ == "__main__":
    main()
