import pygame
import math

def keyboard_control(x, y, theta, v, delta):
    keys = pygame.key.get_pressed()
    steer_input = (-1 if keys[pygame.K_LEFT] else 1 if keys[pygame.K_RIGHT] else 0)
    accel_input = 1 if keys[pygame.K_UP] else -1 if keys[pygame.K_DOWN] else 0
    braking = keys[pygame.K_SPACE]
    reset = keys[pygame.K_r]
    return steer_input, accel_input, braking, reset


# Example: simple obstacle avoidance stub (replace with your logic)
def simple_avoidance_control(x, y, theta, v, delta):
    # This is a placeholder for your control algorithm.
    # For example, you can steer left slowly and accelerate forward.
    steer_input = 0.1  # small constant steering
    accel_input = 0.5  # moderate acceleration
    braking = False
    reset = False
    return steer_input, accel_input, braking, reset
