import numpy as np
from vpython import *
import time
import sys

# --- Constants & Simulation Parameters ---
dt = 0.05             # Timestep (s) - larger dt for orbital distances

# Earth Parameters
R_EARTH = 6371000.0   # Radius (m)
M_EARTH = 5.972e24    # Mass (kg)
G = 6.67430e-11       # Gravitational constant

# Rocket (Stick) Properties
mass = 50000.0        # Mass (kg)
length = 50.0         # Length (m)
radius = 2.0          # Radius (m)
I_xx = (1/12) * mass * length**2
I_yy = 0.5 * mass * radius**2
I_zz = I_xx
I_inv = np.diag([1/I_xx, 1/I_yy, 1/I_zz])

# Actuator Limits
MAX_THRUST = 1.5e6    # N (enough to lift 50 tons against 1g)
MAX_RW_TORQUE = 1e6   # N*m
MAX_GIMBAL = 0.1      # radians

def clip(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def latlon_to_ecef(lat_deg, lon_deg, alt=0.0):
    """Convert Lat/Lon/Alt to Earth-Centered Earth-Fixed (ECEF) coordinates."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    r = R_EARTH + alt
    x = r * np.cos(lat) * np.cos(lon)
    y = r * np.cos(lat) * np.sin(lon)
    z = r * np.sin(lat)
    return np.array([x, y, z])

def ecef_to_latlon(pos):
    """Convert ECEF coordinates back to Lat/Lon/Alt."""
    r = np.linalg.norm(pos)
    if r == 0: return 0, 0, 0
    lat = np.arcsin(pos[2] / r)
    lon = np.arctan2(pos[1], pos[0])
    alt = r - R_EARTH
    return np.degrees(lat), np.degrees(lon), alt

def get_gravity(pos):
    """Calculate gravitational force vector at position."""
    r_mag = np.linalg.norm(pos)
    if r_mag < 1.0: return np.zeros(3)
    g_mag = G * M_EARTH / (r_mag**2)
    return -g_mag * (pos / r_mag)

# --- State Machine Enums ---
class Phase:
    ASCENT = 0
    COAST_TO_DEPLOY = 1
    DEPLOY = 2
    COAST_DOWN = 3
    LANDING = 4
    LANDED = 5

# --- Main Program ---
# --- Main Program State ---
class MissionConfig:
    def __init__(self):
        self.launch_lat = 0.0
        self.launch_lon = 0.0
        self.payload_alt = 400000.0
        self.payload_lat = 5.0
        self.payload_lon = 5.0
        self.land_lat = 10.0
        self.land_lon = 10.0
        self.started = False

config = MissionConfig()

def start_simulation(b):
    try:
        config.launch_lat = float(w_l_lat.text)
        config.launch_lon = float(w_l_lon.text)
        config.payload_alt = float(w_p_alt.text)
        config.payload_lat = float(w_p_lat.text)
        config.payload_lon = float(w_p_lon.text)
        config.land_lat = float(w_ld_lat.text)
        config.land_lon = float(w_ld_lon.text)
    except ValueError:
        print("Invalid input! Please enter numbers only. Using safe defaults.")
    config.started = True

# --- Setup Scene ---
sc = canvas(title='Orbital Rocket Stick Simulation', width=800, height=600, background=color.black)

def no_op(b):
    pass

sc.append_to_caption('\\n<b>Mission Parameters:</b>\\n')
sc.append_to_caption('Launch Lat: ')
w_l_lat = winput(bind=no_op, text='0.0', width=50)
sc.append_to_caption(' Lon: ')
w_l_lon = winput(bind=no_op, text='0.0', width=50)

sc.append_to_caption('\\nPayload Alt(m): ')
w_p_alt = winput(bind=no_op, text='400000', width=70)
sc.append_to_caption(' Lat: ')
w_p_lat = winput(bind=no_op, text='5.0', width=50)
sc.append_to_caption(' Lon: ')
w_p_lon = winput(bind=no_op, text='5.0', width=50)

sc.append_to_caption('\\nLanding Lat: ')
w_ld_lat = winput(bind=no_op, text='10.0', width=50)
sc.append_to_caption(' Lon: ')
w_ld_lon = winput(bind=no_op, text='10.0', width=50)

sc.append_to_caption('\\n\\n')
button(bind=start_simulation, text='<b>Start Mission</b>')
sc.append_to_caption('\\n\\n')

def main():
    print("=== Global Rocket Stick Mission Planner ===")
    print("Please use the browser interface to enter parameters and start the simulation.")
    
    # Wait for user to click start
    while not config.started:
        rate(10)
        
    print("\nMission Profile Locked. Initializing Physics...")
    
    # Target vectors
    pos_launch = latlon_to_ecef(config.launch_lat, config.launch_lon, length/2.0)
    pos_deploy = latlon_to_ecef(config.payload_lat, config.payload_lon, config.payload_alt)
    pos_land = latlon_to_ecef(config.land_lat, config.land_lon, length/2.0)

    # --- Setup Physics State ---
    pos = np.copy(pos_launch)
    vel = np.zeros(3)
    w_world = np.zeros(3)
    
    # Body Y axis points "UP" from the center of the earth initially
    body_y_world = pos / np.linalg.norm(pos)
    
    # --- Vpython Rendering Setup ---
    # Visual scaling factor: to see the rocket next to earth, we draw the earth 1/1000th size
    # and the rocket 100x larger relative to the scene. 
    VIS_SCALE = 1e-4
    
    earth_radius_vis = R_EARTH * VIS_SCALE
    earth = sphere(pos=vec(0,0,0), radius=earth_radius_vis, color=color.blue, opacity=0.8)
    
    # Rocket vis (Scale rocket aggressively so it's visible. Say, 200km visual size)
    ROCKET_VIS_LEN = 200000.0 * VIS_SCALE 
    ROCKET_VIS_RAD = ROCKET_VIS_LEN * 0.1
    stick = cylinder(pos=vec(pos[0]*VIS_SCALE, pos[1]*VIS_SCALE, pos[2]*VIS_SCALE), 
                    axis=vec(body_y_world[0], body_y_world[1], body_y_world[2]) * ROCKET_VIS_LEN, 
                    radius=ROCKET_VIS_RAD, color=color.orange)
    
    thrust_arrow = arrow(pos=stick.pos, axis=vec(0,0,0), color=color.cyan, shaftwidth=ROCKET_VIS_RAD*0.5)

    # Markers for targets
    sphere(pos=vec(pos_deploy[0]*VIS_SCALE, pos_deploy[1]*VIS_SCALE, pos_deploy[2]*VIS_SCALE), radius=earth_radius_vis*0.02, color=color.green)
    sphere(pos=vec(pos_land[0]*VIS_SCALE, pos_land[1]*VIS_SCALE, pos_land[2]*VIS_SCALE), radius=earth_radius_vis*0.02, color=color.red)

    current_phase = Phase.ASCENT
    t = 0.0
    
    # Control Gains
    Kp_att = 5.0 * I_xx
    Kd_att = 2.0 * I_xx

    # Simulation loop
    # We run faster than real time to cover global distances
    TIME_WARP = 10 
    
    payload_obj = None

    print("Ignition!")
    while current_phase != Phase.LANDED:
        rate(TIME_WARP / dt)
        
        # 1. Sensors
        alt = np.linalg.norm(pos) - R_EARTH
        global_up = pos / np.linalg.norm(pos)
        
        # 2. State Machine & Guidance logic
        thrust = 0.0
        target_att = global_up # Default: pointing straight up away from earth
        
        if current_phase == Phase.ASCENT:
            # Fly towards deploy target
            err_pos = pos_deploy - pos
            dist_to_target = np.linalg.norm(err_pos)
            
            # Simple gravity turn: pitch towards target vector
            dir_to_target = err_pos / dist_to_target
            # Blend "up" and "towards target"
            target_att = (global_up * 0.5 + dir_to_target * 0.5)
            target_att = target_att / np.linalg.norm(target_att)
            
            # Full thrust
            thrust = MAX_THRUST
            
            # Check transition
            if dist_to_target < 50000.0: # Close enough (50km)
                print(f"T+{t:.1f}s: Reached deploy altitude. Coasting to deploy.")
                current_phase = Phase.COAST_TO_DEPLOY

        elif current_phase == Phase.COAST_TO_DEPLOY:
            err_pos = pos_deploy - pos
            if np.linalg.norm(err_pos) < 10000.0 or np.dot(vel, err_pos) < 0:
                print(f"T+{t:.1f}s: Deploying Payload!")
                current_phase = Phase.DEPLOY
                
        elif current_phase == Phase.DEPLOY:
            # Spawn a little satellite
            payload_obj = box(pos=vec(pos[0]*VIS_SCALE, pos[1]*VIS_SCALE, pos[2]*VIS_SCALE), 
                              size=vec(ROCKET_VIS_RAD*2, ROCKET_VIS_RAD*2, ROCKET_VIS_RAD*2), 
                              color=color.white)
            print(f"T+{t:.1f}s: Payload Deployed. Commencing descent to landing zone.")
            current_phase = Phase.COAST_DOWN
            
        elif current_phase == Phase.COAST_DOWN:
            # Orient retro-grade for landing burn later
            if np.linalg.norm(vel) > 10.0:
                target_att = -vel / np.linalg.norm(vel) 
            
            # Predict if we need to burn
            # Simple suicide burn calculation: v^2 = 2 * a * d
            # a_max = MAX_THRUST/mass - g
            g_mag = np.linalg.norm(get_gravity(pos))
            a_max = (MAX_THRUST / mass) - g_mag
            if a_max > 0:
                burn_dist = (np.linalg.norm(vel)**2) / (2 * a_max) * 1.5 # 1.5 safety factor
                if alt < burn_dist + 5000.0:
                    print(f"T+{t:.1f}s: Landing Burn Start!")
                    current_phase = Phase.LANDING

        elif current_phase == Phase.LANDING:
            # Hover/Landing control
            err_pos_land = pos_land - pos
            dist_to_land = np.linalg.norm(err_pos_land)
            
            # Attitude: Straight up, but tilt slightly towards landing pad
            dir_to_land_horizontal = err_pos_land - np.dot(err_pos_land, global_up)*global_up
            if np.linalg.norm(dir_to_land_horizontal) > 0:
                dir_to_land_horizontal = dir_to_land_horizontal / np.linalg.norm(dir_to_land_horizontal)
                
            tilt_angle = clip(np.linalg.norm(err_pos_land)*0.0001, 0.0, 0.3) # Max tilt
            target_att = global_up * np.cos(tilt_angle) + dir_to_land_horizontal * np.sin(tilt_angle)
            
            # Thrust control
            target_vel = -global_up * min(50.0, alt * 0.1) # max 50m/s down, slow down as we get lower
            vel_err = np.dot(vel, global_up) - np.dot(target_vel, global_up)
            
            ideal_thrust = mass * np.linalg.norm(get_gravity(pos)) - mass * vel_err * 0.5
            thrust = clip(ideal_thrust, 0, MAX_THRUST)
            
            if alt < length/2 + 5.0 and np.linalg.norm(vel) < 5.0:
                print(f"T+{t:.1f}s: Touchdown!")
                current_phase = Phase.LANDED
                thrust = 0.0

        # --- Attitude Control ---
        err_axis = np.cross(body_y_world, target_att)
        err_mag = np.arcsin(clip(np.linalg.norm(err_axis), -1.0, 1.0))
        if np.linalg.norm(err_axis) > 1e-6:
            err_axis_norm = err_axis / np.linalg.norm(err_axis)
        else:
            err_axis_norm = np.array([1.0, 0.0, 0.0])
            err_mag = 0.0
            
        torque_req = Kp_att * (err_axis_norm * err_mag) - Kd_att * w_world
        
        # Gimbal and RW allocation
        moment_arm_world = -body_y_world * (length / 2.0)
        f_perp_req = np.cross(torque_req, moment_arm_world) / (np.linalg.norm(moment_arm_world)**2 + 1e-6)
        
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
        force_gravity = mass * get_gravity(pos)
        force_thrust = (body_y_world * thrust) + f_perp
        
        # Simple atmospheric drag (density drops exponentially)
        density = 1.225 * np.exp(-alt / 8500.0)
        drag_coeff = 0.5
        area = np.pi * radius**2 # Cross sectional roughly
        force_drag = -0.5 * density * np.linalg.norm(vel) * vel * drag_coeff * area
        
        total_force = force_gravity + force_thrust + force_drag
        
        accel = total_force / mass
        vel += accel * dt
        pos += vel * dt
        
        alpha = torque_applied / I_xx 
        w_world += alpha * dt
        
        # Update body vector mathematically
        w_mag = np.linalg.norm(w_world)
        if w_mag > 1e-6:
            # Rotate body_y_world vector around w_axis by w_mag*dt
            w_axis = w_world / w_mag
            theta = w_mag * dt
            # Rodrigues' rotation formula
            K = np.array([[0, -w_axis[2], w_axis[1]],
                          [w_axis[2], 0, -w_axis[0]],
                          [-w_axis[1], w_axis[0], 0]])
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
            body_y_world = np.dot(R, body_y_world)
            body_y_world = body_y_world / np.linalg.norm(body_y_world)

        # Ensure we don't fall into the earth
        if alt < length/2.0 - 1.0 and current_phase != Phase.LANDED and t > 1.0:
            # Crash
            print(f"T+{t:.1f}s: CRITICAL IMPACT! Altitude: {alt:.1f}m, Vel: {np.linalg.norm(vel):.1f}m/s")
            current_phase = Phase.LANDED

        # --- Vpython Updates ---
        # Update physical position
        vis_pos = pos * VIS_SCALE
        
        if current_phase == Phase.LANDED:
            # Snap to surface for visual clean up
            surf_dir = pos / np.linalg.norm(pos)
            vis_pos = (surf_dir * (R_EARTH + length/2)) * VIS_SCALE
            body_y_world = surf_dir
        
        stick.pos = vec(vis_pos[0], vis_pos[1], vis_pos[2]) - vec(body_y_world[0], body_y_world[1], body_y_world[2]) * (ROCKET_VIS_LEN/2)
        stick.axis = vec(body_y_world[0], body_y_world[1], body_y_world[2]) * ROCKET_VIS_LEN
        
        # Camera chase (look at stick relative to earth)
        sc.center = stick.pos
        # To avoid nauseating spinning, we keep the camera 'up' relative to the earth's surface at the stick
        sc.up = vec(global_up[0], global_up[1], global_up[2])
        # Position camera slightly away
        sc.camera.pos = stick.pos + vec(global_up[0], global_up[1], global_up[2]) * ROCKET_VIS_LEN * 3.0 + vec(body_y_world[2], 0, -body_y_world[0]) * ROCKET_VIS_LEN * 4.0

        if thrust > 0:
            thrust_arrow.visible = True
            thrust_arrow.pos = stick.pos
            exhaust_dir = -body_y_world
            thrust_arrow.axis = vec(exhaust_dir[0], exhaust_dir[1], exhaust_dir[2]) * (thrust / MAX_THRUST * ROCKET_VIS_LEN * 0.5)
        else:
            thrust_arrow.visible = False
            
        # Optional: rotate earth slightly or update it? Earth is static in ECEF.
        
        t += dt

if __name__ == "__main__":
    main()
