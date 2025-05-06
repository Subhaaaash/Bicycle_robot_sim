import math

def lidar_scan(x, y, theta, obstacles_rect, obstacles_circ, max_distance=300, num_rays=36):
    """
    Simulate a 2D LiDAR scan from position (x, y) facing direction theta.
    Returns a list of distances for each ray.
    """

    angles = [theta + 2 * math.pi * i / num_rays for i in range(num_rays)]
    distances = []

    for angle in angles:
        dist = max_distance  # max range
        # Cast a ray in direction `angle`
        for d in range(0, max_distance, 5):
            rx = x + d * math.cos(angle)
            ry = y + d * math.sin(angle)

            # Check collision with rectangular obstacles
            point = (int(rx), int(ry))
            hit = False
            for rect in obstacles_rect:
                if rect.collidepoint(point):
                    dist = d
                    hit = True
                    break
            if hit:
                break

            # Check collision with circular obstacles
            for (cx, cy, r) in obstacles_circ:
                if (rx - cx) ** 2 + (ry - cy) ** 2 <= r ** 2:
                    dist = d
                    hit = True
                    break
            if hit:
                break

        distances.append(dist)

    return distances
