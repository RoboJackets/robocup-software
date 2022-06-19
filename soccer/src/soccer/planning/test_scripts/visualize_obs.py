import matplotlib.pyplot as plt
import numpy as np


def create_static_obs(robot_radius, robot_center, robot_velocity):
    obs_center_shift = 0.05
    obs_radius_inflation = 0.3

    vel_mag = np.linalg.norm(robot_velocity)
    unit_vel = robot_velocity / (vel_mag + 1e-9)
    obs_center = robot_center + (unit_vel * obs_center_shift * vel_mag)

    safety_margin = vel_mag * obs_radius_inflation
    obs_radius = robot_radius + robot_radius * safety_margin

    return obs_center, obs_radius


def main():
    robot_radius = 0.09
    robot_center = (0, 0)

    robot_shell = plt.Circle(robot_center, robot_radius, color="black")

    velocities = [
        np.array([0.0, 0.0]),
        np.array([1.0, 0.0]),
        np.array([2.0, 0.0]),
        np.array([3.0, 0.0]),
        np.array([4.0, 0.0]),
    ]

    obstacles = []
    for robot_velocity in velocities:
        obstacles.append(create_static_obs(robot_radius, robot_center, robot_velocity))

    fig, ax = plt.subplots()

    ax.add_patch(robot_shell)

    for obs in obstacles:
        obs_circle = plt.Circle(obs[0], obs[1], color="red", fill=False)
        ax.add_patch(obs_circle)

    # fig.savefig('plotcircles.png')
    ax.set_aspect("equal", adjustable="box")
    plt.axis("square")
    plt.xlim(-5 * robot_radius, 5 * robot_radius)
    plt.ylim(-5 * robot_radius, 5 * robot_radius)
    plt.show()


if __name__ == "__main__":
    main()
