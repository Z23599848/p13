import numpy as np
import os
import time
import csv
from scipy.spatial.transform import Rotation

# --- Constants & Simulation Parameters ---
dt = 0.05             # Timestep (s)

# Earth Parameters
R_EARTH = 6371000.0   # Radius (m)
M_EARTH = 5.972e24    # Mass (kg)
G = 6.67430e-11       # Gravitational constant

# Rocket (Stick) Properties - SpaceX Falcon 9
DRY_MASS = 25600.0     # kg
START_MASS = 549000.0   # kg (fully loaded)
length = 70.0          # Length (m)
radius = 1.85          # Radius (m)

# Actuator Limits
MAX_THRUST = 7.6e6     # N (9 Merlin engines at sea level)
ISP_SL = 282.0         # s
G0 = 9.80665           # Standard gravity for Isp
MASS_FLOW_RATE = MAX_THRUST / (ISP_SL * G0) # kg/s

MAX_RW_TORQUE = 5e6    # N*m (cold gas thrusters / RCS)
MAX_GIMBAL = 0.15      # radians

def clip(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def latlon_to_ecef(lat_deg, lon_deg, alt=0.0):
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    r = R_EARTH + alt
    x = r * np.cos(lat) * np.cos(lon)
    y = r * np.cos(lat) * np.sin(lon)
    z = r * np.sin(lat)
    return np.array([x, y, z])

def ecef_to_latlon(pos):
    r = np.linalg.norm(pos)
    if r == 0: return 0, 0, 0
    lat = np.arcsin(pos[2] / r)
    lon = np.arctan2(pos[1], pos[0])
    alt = r - R_EARTH
    return np.degrees(lat), np.degrees(lon), alt

def get_gravity(pos):
    r_mag = np.linalg.norm(pos)
    if r_mag < 1.0: return np.zeros(3)
    g_mag = G * M_EARTH / (r_mag**2)
    return -g_mag * (pos / r_mag)

class Phase:
    ASCENT = 0
    COAST_TO_DEPLOY = 1
    DEPLOY = 2
    COAST_DOWN = 3
    LANDING = 4
    LANDED = 5

    @staticmethod
    def to_string(phase):
        names = ["ASCENT", "COAST_TO_DEPLOY", "DEPLOY", "COAST_DOWN", "LANDING", "LANDED"]
        return names[phase] if 0 <= phase < len(names) else "UNKNOWN"

class RocketSimulation:
    def __init__(self, config):
        self.config = config
        
        # Target vectors
        self.pos_launch = latlon_to_ecef(config['launch_lat'], config['launch_lon'], length/2.0)
        self.pos_deploy = latlon_to_ecef(config['payload_lat'], config['payload_lon'], config['payload_alt'])
        self.pos_land = latlon_to_ecef(config['land_lat'], config['land_lon'], length/2.0)

        # State
        self.pos = np.copy(self.pos_launch)
        self.vel = np.zeros(3)
        self.w_world = np.zeros(3)
        self.body_y_world = self.pos / np.linalg.norm(self.pos)
        self.mass = START_MASS
        
        # Build initial rotation matrix where Y points UP
        up = self.body_y_world
        east = np.cross(np.array([0, 0, 1]), up)
        if np.linalg.norm(east) < 1e-6:
            east = np.array([1, 0, 0])
        east = east / np.linalg.norm(east)
        north = np.cross(up, east)
        self.R_world_body = np.column_stack((east, up, north))
        
        self.current_phase = Phase.ASCENT
        self.t = 0.0

        # --- Dual Logging System ---
        timestamp = int(time.time())
        os.makedirs("logs", exist_ok=True)
        
        # 1. Vehicle Flight State (Basic trajectory & orientation)
        self.state_path = f"logs/flight_state_{timestamp}.csv"
        self.state_file = open(self.state_path, "w", newline="")
        self.state_writer = csv.writer(self.state_file)
        self.state_writer.writerow([
            "time", "phase", "lat", "lon", "alt", 
            "vel_x", "vel_y", "vel_z", "vel_mag",
            "qx", "qy", "qz", "qw"
        ])
        
        # 2. Mission Telemetry (Detailed internal diagnostics)
        self.telemetry_path = f"logs/mission_telemetry_{timestamp}.csv"
        self.telemetry_file = open(self.telemetry_path, "w", newline="")
        self.telemetry_writer = csv.writer(self.telemetry_file)
        self.telemetry_writer.writerow([
            "time", "mass", "thrust", "throttle",
            "gimbal_x", "gimbal_y", "gimbal_z",
            "rw_x", "rw_y", "rw_z",
            "accel_x", "accel_y", "accel_z",
            "force_grav_x", "force_grav_y", "force_grav_z",
            "force_thrust_x", "force_thrust_y", "force_thrust_z",
            "force_drag_x", "force_drag_y", "force_drag_z",
            "target_att_x", "target_att_y", "target_att_z"
        ])

    def get_inertia(self):
        """Dynamic moments of inertia based on current mass."""
        idx = (1/12) * self.mass * length**2
        iyy = 0.5 * self.mass * radius**2
        izz = idx
        return idx, iyy, izz

    def step(self):
        """Advances the simulation by one timestep and returns telemetry."""
        if self.current_phase == Phase.LANDED:
            return self._get_telemetry()

        # Altitude and up vector
        alt = np.linalg.norm(self.pos) - R_EARTH
        global_up = self.pos / np.linalg.norm(self.pos)
        
        thrust = 0.0
        target_att = global_up
        
        # --- State Machine & Guidance ---
        if self.current_phase == Phase.ASCENT:
            err_pos = self.pos_deploy - self.pos
            dist_to_target = np.linalg.norm(err_pos)
            
            # Guidance direction
            horizontal_dir_to_target = err_pos - np.dot(err_pos, global_up)*global_up
            if np.linalg.norm(horizontal_dir_to_target) > 0:
                horizontal_dir_to_target = horizontal_dir_to_target / np.linalg.norm(horizontal_dir_to_target)
            else:
                horizontal_dir_to_target = np.array([1.0, 0.0, 0.0])

            # Safer gravity turn: limit tilt to maintain vertical TWR > 1.2
            g_loc = np.linalg.norm(get_gravity(self.pos))
            twr = MAX_THRUST / (self.mass * g_loc)
            
            # Max allowed tilt to keep vertical acceleration positive
            max_tilt = np.arccos(1.2 / twr) if twr > 1.2 else 0.0
            
            tilt_angle = min(max_tilt, alt * 0.00001)
            if alt < 2000:
                tilt_angle = 0.0 # Clear tower vertically
                
            target_att = global_up * np.cos(tilt_angle) + horizontal_dir_to_target * np.sin(tilt_angle)
            target_att = target_att / np.linalg.norm(target_att)
            thrust = MAX_THRUST
            
            if dist_to_target < 20000.0 or alt > self.config['payload_alt'] or self.t > 300.0:
                self.current_phase = Phase.COAST_TO_DEPLOY
                print(f"Transitioning to COAST_TO_DEPLOY at t={self.t}, alt={alt}")

        elif self.current_phase == Phase.COAST_TO_DEPLOY:
            err_pos = self.pos_deploy - self.pos
            if np.linalg.norm(err_pos) < 20000.0 or np.dot(self.vel, err_pos) < 0:
                self.current_phase = Phase.DEPLOY
                
        elif self.current_phase == Phase.DEPLOY:
            self.current_phase = Phase.COAST_DOWN
            
        elif self.current_phase == Phase.COAST_DOWN:
            if np.linalg.norm(self.vel) > 10.0:
                target_att = -self.vel / np.linalg.norm(self.vel) 
            
            g_mag = np.linalg.norm(get_gravity(self.pos))
            a_max = (MAX_THRUST / self.mass) - g_mag
            if a_max > 0:
                burn_dist = (np.linalg.norm(self.vel)**2) / (2 * a_max) * 1.2
                if alt < burn_dist + 2000.0:
                    self.current_phase = Phase.LANDING
                    print(f"Transitioning to LANDING at t={self.t}, alt={alt}")

        elif self.current_phase == Phase.LANDING:
            # 1. Suicide Burn Logic (Vertical)
            v_vertical = np.dot(self.vel, global_up)
            g_mag = np.linalg.norm(get_gravity(self.pos))
            twr_max = MAX_THRUST / self.mass
            a_max_vertical = twr_max - g_mag
            
            # Distance needed to stop vertically
            safe_burn_dist = (v_vertical**2) / (2.0 * max(1.0, a_max_vertical))
            
            # Start burn if alt is close to stopping distance
            if alt < (safe_burn_dist * 1.1) + 50.0:
                # Vertical controller for soft landing
                target_v_vertical = -min(20.0, max(2.0, alt * 0.1))
                v_err = v_vertical - target_v_vertical
                thrust_req = self.mass * (g_mag - v_err * 2.0)
                thrust = clip(thrust_req, 0, MAX_THRUST)
            else:
                thrust = 0.0
                
            # 2. Horizontal Drift Kill (Safety first)
            v_horiz = self.vel - v_vertical * global_up
            v_horiz_mag = np.linalg.norm(v_horiz)
            
            # Target is the vector that cancels drift and slowly leans toward the pad
            err_pos_land = self.pos_land - self.pos
            horizontal_err = err_pos_land - np.dot(err_pos_land, global_up) * global_up
            
            if v_horiz_mag > 1.0:
                # Lean against the drift (Retrograde-ish)
                horiz_corr = -v_horiz * 0.2
            else:
                # Lean toward the pad
                horiz_corr = horizontal_err * 0.001
                
            # Combine Vertical Up with Horizontal Correction, capped at 15 degrees tilt
            target_att = global_up + horiz_corr
            target_att = target_att / np.linalg.norm(target_att)
            
            # Magnitude check for tilt (limit to 15 degrees / 0.26 rad)
            max_tilt = 0.26
            dot_up = np.dot(target_att, global_up)
            if dot_up < np.cos(max_tilt):
                side = target_att - dot_up * global_up
                side = side / np.linalg.norm(side)
                target_att = global_up * np.cos(max_tilt) + side * np.sin(max_tilt)

            if alt < length/2 + 5.0 and np.linalg.norm(self.vel) < 5.0:
                self.current_phase = Phase.LANDED
                thrust = 0.0
                self.vel *= 0.0

        # Out of fuel check
        if self.mass <= DRY_MASS:
            thrust = 0.0

        # --- Attitude Control ---
        I_xx_val, I_yy_val, I_zz_val = self.get_inertia()
        Kp_att = 10.0 * I_xx_val
        Kd_att = 5.0 * I_xx_val

        err_axis = np.cross(self.body_y_world, target_att)
        err_mag = np.arcsin(clip(np.linalg.norm(err_axis), -1.0, 1.0))
        if np.linalg.norm(err_axis) > 1e-6:
            err_axis_norm = err_axis / np.linalg.norm(err_axis)
        else:
            err_axis_norm = np.array([1.0, 0.0, 0.0])
            err_mag = 0.0
            
        torque_req = Kp_att * (err_axis_norm * err_mag) - Kd_att * self.w_world
        
        # Gimbal and RW allocation
        moment_arm_world = -self.body_y_world * (length / 2.0)
        dist_sq = np.linalg.norm(moment_arm_world)**2 + 1e-6
        f_perp_req = np.cross(torque_req, moment_arm_world) / dist_sq
        
        max_f_perp = thrust * np.tan(MAX_GIMBAL)
        f_perp_mag = np.linalg.norm(f_perp_req)
        if f_perp_mag > max_f_perp and max_f_perp > 0:
            f_perp = f_perp_req * (max_f_perp / f_perp_mag)
        else:
            f_perp = f_perp_req
            
        gimbal_torque = np.cross(moment_arm_world, f_perp)
        rw_torque_req = torque_req - gimbal_torque
        rw_torque = np.clip(rw_torque_req, -MAX_RW_TORQUE, MAX_RW_TORQUE)
        torque_applied = gimbal_torque + rw_torque

        # --- Physics Integration ---
        force_gravity = self.mass * get_gravity(self.pos)
        force_thrust = (self.body_y_world * thrust) + f_perp
        
        density = 1.225 * np.exp(-alt / 8500.0)
        drag_coeff = 0.5 
        force_drag = -0.5 * density * np.linalg.norm(self.vel) * self.vel * drag_coeff * (np.pi * radius**2)
        
        total_force = force_gravity + force_thrust + force_drag
        
        accel = total_force / self.mass
        self.vel += accel * dt
        self.pos += self.vel * dt
        
        # Proper Inertia Tensor Math for Angular Acceleration
        I_body_inv = np.diag([1.0/I_xx_val, 1.0/I_yy_val, 1.0/I_zz_val])
        I_world_inv = self.R_world_body @ I_body_inv @ self.R_world_body.T
        alpha_world = I_world_inv @ torque_applied
        self.w_world += alpha_world * dt
        
        # Mass depletion
        if thrust > 0:
            self.mass -= MASS_FLOW_RATE * dt
            if self.mass < DRY_MASS: self.mass = DRY_MASS
        
        # Update body vector mathematically using full rotation matrix
        w_mag = np.linalg.norm(self.w_world)
        if w_mag > 1e-6:
            w_axis = self.w_world / w_mag
            theta = w_mag * dt
            K = np.array([[0, -w_axis[2], w_axis[1]],
                          [w_axis[2], 0, -w_axis[0]],
                          [-w_axis[1], w_axis[0], 0]])
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
            self.R_world_body = np.dot(R, self.R_world_body)
            # Re-orthogonalize
            u, _, vh = np.linalg.svd(self.R_world_body)
            self.R_world_body = np.dot(u, vh)
            self.body_y_world = self.R_world_body[:, 1]

        # --- Handle Logging ---
        lat, lon, log_alt = ecef_to_latlon(self.pos)
        rot = Rotation.from_matrix(self.R_world_body)
        q = rot.as_quat() # [x, y, z, w]
        
        # 1. Write Flight State
        self.state_writer.writerow([
            round(self.t, 2), Phase.to_string(self.current_phase),
            lat, lon, log_alt,
            self.vel[0], self.vel[1], self.vel[2], np.linalg.norm(self.vel),
            q[0], q[1], q[2], q[3]
        ])
        
        # 2. Write Mission Telemetry
        self.telemetry_writer.writerow([
            round(self.t, 2), self.mass, thrust, thrust / MAX_THRUST,
            gimbal_torque[0], gimbal_torque[1], gimbal_torque[2],
            rw_torque[0], rw_torque[1], rw_torque[2],
            accel[0], accel[1], accel[2],
            force_gravity[0], force_gravity[1], force_gravity[2],
            force_thrust[0], force_thrust[1], force_thrust[2],
            force_drag[0], force_drag[1], force_drag[2],
            target_att[0], target_att[1], target_att[2]
        ])
        
        self.state_file.flush()
        self.telemetry_file.flush()

        if alt < length/2.0 - 1.0 and self.current_phase != Phase.LANDED and self.t > 1.0:
            self.current_phase = Phase.LANDED
            self.vel *= 0.0
            
        if self.current_phase == Phase.LANDED:
            if hasattr(self, 'state_file') and self.state_file:
                self.state_file.close()
                self.state_file = None
            if hasattr(self, 'telemetry_file') and self.telemetry_file:
                self.telemetry_file.close()
                self.telemetry_file = None

        self.t += dt
        return self._get_telemetry()

    def _get_telemetry(self):
        lat, lon, alt = ecef_to_latlon(self.pos)
        
        # Scipy quaternion format is [x, y, z, w]
        rot = Rotation.from_matrix(self.R_world_body)
        quat = rot.as_quat().tolist()
        
        return {
            "time": round(self.t, 2),
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "velocity": float(np.linalg.norm(self.vel)),
            "phase": Phase.to_string(self.current_phase),
            "quaternion": quat
        }
