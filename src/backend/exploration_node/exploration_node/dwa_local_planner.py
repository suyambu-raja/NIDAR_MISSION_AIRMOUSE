#!/usr/bin/env python3
"""
dwa_local_planner.py — NIDAR AirMouse: Dynamic Window Approach (DWA) Local Planner
==================================================================================
Computes dynamic, collision-free velocity commands (v, omega) for indoor UAV navigation.
Evaluates admissible trajectories in the dynamic velocity window against local
occupancy grid obstacles, optimizing for goal progression, clearance, and smooth heading.
"""

import math
from typing import Tuple, Optional, List, Dict, Any


class DWALocalPlanner:
    """
    Dynamic Window Approach (DWA) local velocity trajectory planner.
    """

    def __init__(
        self,
        max_speed: float = 0.8,            # m/s
        min_speed: float = 0.0,            # m/s
        max_yaw_rate: float = 1.0,         # rad/s
        max_accel: float = 0.6,            # m/s^2
        max_dyaw_rate: float = 1.5,        # rad/s^2
        v_resolution: float = 0.08,        # velocity step
        yaw_rate_resolution: float = 0.1,  # yaw rate step
        sim_time: float = 1.5,             # trajectory horizon (s)
        sim_granularity: float = 0.15,     # dt per step (s)
        drone_radius: float = 0.28,        # vehicle footprint clearance buffer (m)
        emergency_brake_dist: float = 0.45,# emergency stop threshold (m)
        heading_weight: float = 0.25,
        clearance_weight: float = 0.45,
        velocity_weight: float = 0.30,
    ) -> None:
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_yaw_rate = max_yaw_rate
        self.max_accel = max_accel
        self.max_dyaw_rate = max_dyaw_rate
        self.v_res = v_resolution
        self.yaw_res = yaw_rate_resolution
        self.sim_time = sim_time
        self.sim_dt = sim_granularity
        self.drone_radius = drone_radius
        self.emergency_brake_dist = emergency_brake_dist

        self.w_heading = heading_weight
        self.w_clearance = clearance_weight
        self.w_velocity = velocity_weight

    def compute_velocity_command(
        self,
        current_pose: Tuple[float, float, float],    # (x, y, yaw)
        current_velocity: Tuple[float, float],       # (v, omega)
        target_waypoint: Tuple[float, float],        # (goal_x, goal_y)
        occupancy_map,                               # nav_msgs/OccupancyGrid
    ) -> Tuple[float, float, bool, float]:
        """
        Calculates the best velocity command (v, omega) using DWA.

        Returns:
            best_v (float): Linear forward velocity (m/s)
            best_omega (float): Angular yaw velocity (rad/s)
            emergency_brake (bool): True if obstacle closer than 0.45m
            min_clearance (float): Distance to closest detected obstacle (m)
        """
        x, y, yaw = current_pose
        v, omega = current_velocity
        goal_x, goal_y = target_waypoint

        # Calculate dynamic window based on current velocity and acceleration limits
        vs = [
            max(self.min_speed, v - self.max_accel * self.sim_dt),
            min(self.max_speed, v + self.max_accel * self.sim_dt),
            -self.max_yaw_rate,
            self.max_yaw_rate,
        ]

        # Extract local obstacles around current drone position for fast spatial evaluation
        obstacles = self._extract_nearby_obstacles(occupancy_map, (x, y), search_radius=3.0)

        # Check immediate obstacle proximity for emergency braking
        immediate_min_dist = float('inf')
        for ox, oy in obstacles:
            d = math.hypot(ox - x, oy - y)
            if d < immediate_min_dist:
                immediate_min_dist = d

        if immediate_min_dist < self.emergency_brake_dist:
            # Active emergency brake
            return 0.0, 0.0, True, immediate_min_dist

        best_score = -float('inf')
        best_v = 0.0
        best_omega = 0.0
        best_clearance = immediate_min_dist

        # Sample velocity space inside dynamic window
        v_samples = []
        cur_v = vs[0]
        while cur_v <= vs[1] + 1e-4:
            v_samples.append(cur_v)
            cur_v += self.v_res

        omega_samples = []
        cur_omega = vs[2]
        while cur_omega <= vs[3] + 1e-4:
            omega_samples.append(cur_omega)
            cur_omega += self.yaw_res

        for sample_v in v_samples:
            for sample_w in omega_samples:
                # Roll out trajectory
                traj = self._predict_trajectory(x, y, yaw, sample_v, sample_w)
                # Compute clearance
                min_dist = self._calc_trajectory_clearance(traj, obstacles)
                if min_dist <= self.drone_radius:
                    # Collision trajectory — discard
                    continue

                # Compute heading score (alignment with goal waypoint)
                end_x, end_y, end_yaw = traj[-1]
                angle_to_goal = math.atan2(goal_y - end_y, goal_x - end_x)
                heading_err = abs(self._normalize_angle(angle_to_goal - end_yaw))
                heading_score = math.pi - heading_err

                # Clearance score (capped at 2.0m to prevent infinite inflation)
                clearance_score = min(2.0, min_dist)

                # Velocity score
                velocity_score = sample_v / max(0.1, self.max_speed)

                # Total objective function
                score = (
                    self.w_heading * (heading_score / math.pi)
                    + self.w_clearance * (clearance_score / 2.0)
                    + self.w_velocity * velocity_score
                )

                if score > best_score:
                    best_score = score
                    best_v = sample_v
                    best_omega = sample_w
                    best_clearance = min_dist

        if best_score == -float('inf'):
            # No admissible trajectory found without collision -> brake
            return 0.0, 0.0, True, immediate_min_dist

        return round(best_v, 2), round(best_omega, 2), False, round(best_clearance, 2)

    def _predict_trajectory(
        self, x: float, y: float, yaw: float, v: float, omega: float
    ) -> List[Tuple[float, float, float]]:
        """Rolls out kinematic trajectory forward in time."""
        traj = [(x, y, yaw)]
        t = 0.0
        curr_x, curr_y, curr_yaw = x, y, yaw
        while t <= self.sim_time:
            curr_yaw += omega * self.sim_dt
            curr_x += v * math.cos(curr_yaw) * self.sim_dt
            curr_y += v * math.sin(curr_yaw) * self.sim_dt
            traj.append((curr_x, curr_y, curr_yaw))
            t += self.sim_dt
        return traj

    def _calc_trajectory_clearance(
        self, traj: List[Tuple[float, float, float]], obstacles: List[Tuple[float, float]]
    ) -> float:
        """Finds minimum distance between any point along trajectory and obstacles."""
        if not obstacles:
            return 3.0
        min_d = float('inf')
        for tx, ty, _ in traj:
            for ox, oy in obstacles:
                d = math.hypot(ox - tx, oy - ty)
                if d < min_d:
                    min_d = d
                    if min_d <= self.drone_radius:
                        return min_d
        return min_d

    def _extract_nearby_obstacles(
        self, map_msg, drone_pos: Tuple[float, float], search_radius: float = 3.0
    ) -> List[Tuple[float, float]]:
        """Extracts world coordinates of occupied cells within search_radius of drone."""
        if map_msg is None or not hasattr(map_msg, 'data') or not map_msg.data:
            return []

        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        px, py = drone_pos
        c_min = max(0, int((px - search_radius - ox) / res))
        c_max = min(w - 1, int((px + search_radius - ox) / res))
        r_min = max(0, int((py - search_radius - oy) / res))
        r_max = min(h - 1, int((py + search_radius - oy) / res))

        data = map_msg.data
        obstacles = []
        for r in range(r_min, r_max + 1):
            row_offset = r * w
            for c in range(c_min, c_max + 1):
                if data[row_offset + c] >= 50:
                    wx = ox + (c + 0.5) * res
                    wy = oy + (r + 0.5) * res
                    obstacles.append((wx, wy))
        return obstacles

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalizes angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
