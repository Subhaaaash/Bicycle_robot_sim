import pygame
from simulator import LIDAR_NUM_RAYS

def keyboard_control(x, y, theta, v, delta, lidar_distances):
    keys = pygame.key.get_pressed()
    steer_input = (-1 if keys[pygame.K_LEFT] else 1 if keys[pygame.K_RIGHT] else 0)
    accel_input = 1 if keys[pygame.K_UP] else -1 if keys[pygame.K_DOWN] else 0
    braking = keys[pygame.K_SPACE]
    reset = keys[pygame.K_r]
    return steer_input, accel_input, braking, reset


"""def simple_avoidance_control(x, y, theta, v, delta, lidar_distances):

    #Example simple obstacle avoidance:
   ## - Check front sector for obstacles.
   ## - Otherwise go straight.
    
    # Front sector indices (approx ±45 degrees)
    sector_size = LIDAR_NUM_RAYS // 4
    front_sector = lidar_distances[:sector_size] + lidar_distances[-sector_size:]
    min_dist = min(front_sector) if front_sector else 1000

    steer_input = 0
    accel_input = 1
    braking = False
    reset = False

    if min_dist < 100:
        # Obstacle close in front, steer right and slow down
        steer_input = 1
        accel_input = 0.5
    else:
        steer_input = 0
        accel_input = 1

    return steer_input, accel_input, braking, reset
"""