import pygame
import math
import random
from lidar import lidar_scan

# Constants
WIDTH, HEIGHT = 1600, 1000
BG_COLOR = (255, 255, 255)
OBSTACLE_COLOR = (100, 0, 0)
CIRC_OBS_COLOR = (0, 100, 0)
CAR_BODY_COLOR = (20, 20, 180)
WHEEL_COLOR = (30, 30, 30)
FPS = 60
SCALE = 0.8

# Vehicle parameters
L = 2.5
L_f = L_r = L / 2
CAR_WIDTH = 1.2 * 30 * SCALE
CAR_LENGTH = L * 30 * SCALE
MAX_STEER = math.radians(30)
ACCELERATION = 3.0
BRAKE_FORCE = 10.0  # braking acceleration magnitude
FRICTION = 1.0

# Obstacle settings
NUM_OBSTACLES = 20
OBSTACLE_SIZE = (80, 60)
CIRC_OBSTACLE_RADIUS = 30

# LiDAR settings
LIDAR_MAX_DISTANCE = 300  # pixels
LIDAR_NUM_RAYS = 36       # number of rays in 360 degrees


class BicycleSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Bicycle Simulator with LiDAR")
        self.clock = pygame.time.Clock()

        self.reset_simulation()

    def reset_simulation(self):
        self.theta = -math.pi / 2  # Facing upward
        self.v = 0
        self.delta = 0
        margin = 250
        self.x = WIDTH - CAR_LENGTH - margin
        self.y = HEIGHT - CAR_WIDTH - margin

        self.rect_obstacles = []
        self.circ_obstacles = []
        for _ in range(NUM_OBSTACLES):
            ox = random.randint(100, WIDTH - 200)
            oy = random.randint(100, HEIGHT - 200)
            if random.random() < 0.5:
                rect = pygame.Rect(ox, oy, *OBSTACLE_SIZE)
                self.rect_obstacles.append(rect)
            else:
                self.circ_obstacles.append((ox, oy, CIRC_OBSTACLE_RADIUS))

    @staticmethod
    def wrap_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def update_bicycle_model(self, x, y, theta, v, delta, a, dt):
        beta = math.atan2(L_r * math.tan(delta), L)
        x += v * math.cos(theta + beta) * dt * 30 * SCALE
        y += v * math.sin(theta + beta) * dt * 30 * SCALE
        theta += (v / L_r) * math.sin(beta) * dt
        v += a * dt
        return x, y, self.wrap_angle(theta), v

    def get_car_corners(self, x, y, theta):
        w = CAR_WIDTH
        h = CAR_LENGTH
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        dx = h / 2
        dy = w / 2
        corners = [
            (x + cos_t * dx - sin_t * dy, y + sin_t * dx + cos_t * dy),
            (x + cos_t * dx + sin_t * dy, y + sin_t * dx - cos_t * dy),
            (x - cos_t * dx + sin_t * dy, y - sin_t * dx - cos_t * dy),
            (x - cos_t * dx - sin_t * dy, y - sin_t * dx + cos_t * dy),
        ]
        return corners

    def draw_car(self, x, y, theta, delta):
        arrow_length = CAR_LENGTH
        arrow_width = CAR_WIDTH

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        front_x = x + arrow_length * cos_t
        front_y = y + arrow_length * sin_t

        left_x = x - (arrow_width / 2) * sin_t
        left_y = y + (arrow_width / 2) * cos_t

        right_x = x + (arrow_width / 2) * sin_t
        right_y = y - (arrow_width / 2) * cos_t

        pygame.draw.polygon(
            self.screen,
            CAR_BODY_COLOR,
            [(int(front_x), int(front_y)), (int(left_x), int(left_y)), (int(right_x), int(right_y))],
        )

        wheel_length = 10 * SCALE * 1.5
        wheel_width = 4 * SCALE * 1.5
        dx = CAR_LENGTH / 2 - 10 * SCALE
        dy = CAR_WIDTH / 2 - 5 * SCALE
        wheels = [
            (-dx, dy, 0),
            (-dx, -dy, 0),
            (dx, dy, delta),
            (dx, -dy, delta),
        ]
        for wx, wy, angle in wheels:
            wx_global = x + math.cos(theta) * wx - math.sin(theta) * wy
            wy_global = y + math.sin(theta) * wx + math.cos(theta) * wy
            wheel_surface = pygame.Surface((wheel_length, wheel_width), pygame.SRCALPHA)
            wheel_surface.fill(WHEEL_COLOR)
            wheel_rot = pygame.transform.rotate(wheel_surface, -math.degrees(theta + angle))
            rect_center = wheel_rot.get_rect(center=(wx_global, wy_global))
            self.screen.blit(wheel_rot, rect_center)

    def check_collision(self, corners):
        car_polygon = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(car_polygon, (255, 255, 255), corners)
        car_mask = pygame.mask.from_surface(car_polygon)

        for rect in self.rect_obstacles:
            obs_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            pygame.draw.rect(obs_surface, (255, 255, 255), rect)
            obs_mask = pygame.mask.from_surface(obs_surface)
            if car_mask.overlap(obs_mask, (0, 0)):
                return True

        for cx, cy, radius in self.circ_obstacles:
            obs_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            pygame.draw.circle(obs_surface, (255, 255, 255), (cx, cy), radius)
            obs_mask = pygame.mask.from_surface(obs_surface)
            if car_mask.overlap(obs_mask, (0, 0)):
                return True

        return False

    def perform_lidar_scan(self):
        distances = lidar_scan(
            self.x,
            self.y,
            self.theta,
            self.rect_obstacles,
            self.circ_obstacles,
            max_distance=LIDAR_MAX_DISTANCE,
            num_rays=LIDAR_NUM_RAYS,
        )
        return distances

    def draw_lidar(self, distances):
        angles = [self.theta + 2 * math.pi * i / LIDAR_NUM_RAYS for i in range(LIDAR_NUM_RAYS)]
        for dist, angle in zip(distances, angles):
            end_x = self.x + dist * math.cos(angle)
            end_y = self.y + dist * math.sin(angle)
            pygame.draw.line(self.screen, (0, 0, 255), (self.x, self.y), (end_x, end_y), 1)
            pygame.draw.circle(self.screen, (0, 0, 255), (int(end_x), int(end_y)), 3)

    def run(self, control_algorithm):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0
            self.screen.fill(BG_COLOR)

            # Draw obstacles
            for obs in self.rect_obstacles:
                pygame.draw.rect(self.screen, OBSTACLE_COLOR, obs)
            for cx, cy, r in self.circ_obstacles:
                pygame.draw.circle(self.screen, CIRC_OBS_COLOR, (cx, cy), r)

            # Perform LiDAR scan and get control inputs
            distances = self.perform_lidar_scan()
            steer_input, accel_input, braking, reset_flag = control_algorithm(
                self.x, self.y, self.theta, self.v, self.delta, distances
            )

            # Update steering
            self.delta += steer_input * math.radians(60) * dt
            self.delta = max(-MAX_STEER, min(MAX_STEER, self.delta))

            # Calculate acceleration
            if braking and abs(self.v) > 0:
                # Apply braking acceleration opposite to velocity direction
                a = -BRAKE_FORCE * (self.v / abs(self.v))
                # Prevent velocity from reversing direction
                if abs(self.v) < abs(a * dt):
                    a = -self.v / dt  # exactly cancel velocity to zero this frame
            elif accel_input != 0:
                a = accel_input * ACCELERATION
            else:
                # Apply friction to slow down vehicle smoothly to zero
                if abs(self.v) > 0:
                    friction_acc = -FRICTION * (self.v / abs(self.v))
                    if abs(self.v) < abs(friction_acc * dt):
                        a = -self.v / dt  # stop immediately
                    else:
                        a = friction_acc
                else:
                    a = 0

            # Update vehicle state
            self.x, self.y, self.theta, self.v = self.update_bicycle_model(
                self.x, self.y, self.theta, self.v, self.delta, a, dt
            )

            # Check collision
            corners = self.get_car_corners(self.x, self.y, self.theta)
            if self.check_collision(corners):
                print("💥 Collision detected!")
                self.v = 0

            # Draw LiDAR rays below vehicle
            self.draw_lidar(distances)

            # Draw vehicle on top
            self.draw_car(self.x, self.y, self.theta, self.delta)

            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.reset_simulation()

            pygame.display.flip()

        pygame.quit()
