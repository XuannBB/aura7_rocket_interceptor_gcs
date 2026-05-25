"""
PID Control Loop & Trajectory Simulation Library for Multi-Rotors
Author: UAV Flight Control Systems Developer
Description: Implements Betaflight-style PID rate controllers, an actuator mixer with voltage
             compensation and Throttle PID Attenuation (TPA), and a test bench for high-G maneuvers.
"""

from typing import Dict, List, Any, Tuple
import numpy as np
from propulsion_engine import BatteryConfig, ESCConfig, MotorConfig, PropellerConfig, PropulsionSystem
from flight_dynamics import FlightDynamics


class PIDController:
    """
    Standard PID rate controller with derivative low-pass filtering and anti-windup clamping.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_limit: float = 200.0,         # Maximum integrated error contribution (deg/s)
        filter_rc: float = 0.015        # Low-pass filter time constant for D-term (seconds)
    ):
        """
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            i_limit: Anti-windup clamping limit for the integrator
            filter_rc: Time constant for D-term low-pass filtering (10Hz cutoff by default)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.filter_rc = filter_rc

        self.i_term = 0.0
        self.last_error = 0.0
        self.d_term_filtered = 0.0

    def reset(self) -> None:
        """Resets the internal integrator and derivative history."""
        self.i_term = 0.0
        self.last_error = 0.0
        self.d_term_filtered = 0.0

    def update(self, error: float, dt: float, v_factor: float = 1.0, tpa_factor: float = 1.0) -> float:
        """
        Calculate PID output.

        Args:
            error: Setpoint - Process Variable (deg/s)
            dt: Sample time (seconds)
            v_factor: Gain scaling factor for battery voltage compensation
            tpa_factor: Gain scaling factor for Throttle PID Attenuation (TPA)
        Returns:
            Control command output
        """
        # Apply dynamic gain adjustments
        # Voltage compensation scales P, I, and D. TPA scales only P and D.
        kp_eff = self.kp * v_factor * tpa_factor
        ki_eff = self.ki * v_factor
        kd_eff = self.kd * v_factor * tpa_factor

        # Proportional term
        p_val = kp_eff * error

        # Integral term with anti-windup
        self.i_term += ki_eff * error * dt
        self.i_term = np.clip(self.i_term, -self.i_limit, self.i_limit)

        # Derivative term with low-pass filtering to reduce high-frequency noise
        if dt > 0:
            d_val_raw = kd_eff * (error - self.last_error) / dt
            alpha = dt / (self.filter_rc + dt)
            self.d_term_filtered = alpha * d_val_raw + (1.0 - alpha) * self.d_term_filtered
        else:
            self.d_term_filtered = 0.0

        self.last_error = error

        return p_val + self.i_term + self.d_term_filtered


class FlightController:
    """
    Combines rate controllers, actuator mixing, and voltage/TPA compensations.
    """

    def __init__(
        self,
        kp_roll: float = 0.45,
        ki_roll: float = 1.20,
        kd_roll: float = 0.015,
        kp_pitch: float = 0.48,
        ki_pitch: float = 1.25,
        kd_pitch: float = 0.018,
        kp_yaw: float = 0.85,
        ki_yaw: float = 1.50,
        kd_yaw: float = 0.0,
        tpa_threshold: float = 0.55,    # Throttle threshold where TPA begins to attenuate (55%)
        tpa_rate: float = 0.35          # Attenuation percentage at 100% throttle (35% reduction)
    ):
        """
        Initializes the rate controllers. Gains are in units of throttle command per deg/s error.
        """
        self.pid_roll = PIDController(kp_roll, ki_roll, kd_roll)
        self.pid_pitch = PIDController(kp_pitch, ki_pitch, kd_pitch)
        self.pid_yaw = PIDController(kp_yaw, ki_yaw, kd_yaw)

        self.tpa_threshold = tpa_threshold
        self.tpa_rate = tpa_rate

        # Standard nominal battery pack voltage (for gain compensation)
        # Default configuration is 6S LiPo, so nominal is 6 * 3.7V = 22.2V
        self.v_nominal = 22.2

    def reset(self) -> None:
        """Reset PID controllers."""
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()

    def get_control_outputs(
        self,
        rate_setpoints: Tuple[float, float, float],    # Desired Roll, Pitch, Yaw rates (deg/s)
        rates_measured: Tuple[float, float, float],    # Actual Roll, Pitch, Yaw rates (deg/s)
        throttle: float,                               # Collective throttle (0.0 to 1.0)
        v_battery_terminal: float,                     # Current battery terminal voltage (V)
        dt: float
    ) -> np.ndarray:
        """
        Updates the PID loops and mixes output commands into motor throttle commands (0.0 to 1.0).

        Args:
            rate_setpoints: Desired Roll, Pitch, Yaw rates (deg/s)
            rates_measured: Measured Roll, Pitch, Yaw rates (deg/s)
            throttle: Input throttle command (0.0 to 1.0)
            v_battery_terminal: Battery terminal voltage under load (V)
            dt: Loop time step (seconds)
        Returns:
            Array of 4 motor throttles [M1, M2, M3, M4] clamped to [0.0, 1.0]
        """
        # 1. Calculate voltage compensation factor
        # Gains are scaled up to compensate for lower battery voltage
        if v_battery_terminal > 5.0:
            v_factor = self.v_nominal / v_battery_terminal
            # Clamp compensation factor to prevent infinite loop gain when battery is completely dead
            v_factor = np.clip(v_factor, 0.8, 1.5)
        else:
            v_factor = 1.0

        # 2. Calculate Throttle PID Attenuation (TPA) factor
        # Lowers P and D gains at high throttles to prevent motor-induced feedback oscillations
        if throttle > self.tpa_threshold:
            # Linear attenuation: starts at tpa_threshold and scales down by tpa_rate at 1.0 throttle
            tpa_factor = 1.0 - ((throttle - self.tpa_threshold) / (1.0 - self.tpa_threshold)) * self.tpa_rate
        else:
            tpa_factor = 1.0

        # 3. Update PIDs
        err_roll = rate_setpoints[0] - rates_measured[0]
        err_pitch = rate_setpoints[1] - rates_measured[1]
        err_yaw = rate_setpoints[2] - rates_measured[2]

        r_out = self.pid_roll.update(err_roll, dt, v_factor, tpa_factor)
        p_out = self.pid_pitch.update(err_pitch, dt, v_factor, tpa_factor)
        y_out = self.pid_yaw.update(err_yaw, dt, v_factor, tpa_factor)

        # 4. Actuator mixer for X-configuration (Betaflight mapping)
        # Motor 1: Rear Right  (CCW) -> M1 = Throttle - Roll + Pitch + Yaw
        # Motor 2: Front Right (CW)  -> M2 = Throttle - Roll - Pitch - Yaw
        # Motor 3: Rear Left   (CW)  -> M3 = Throttle + Roll + Pitch - Yaw
        # Motor 4: Front Left  (CCW) -> M4 = Throttle + Roll - Pitch + Yaw
        m1 = throttle - r_out + p_out + y_out
        m2 = throttle - r_out - p_out - y_out
        m3 = throttle + r_out + p_out - y_out
        m4 = throttle + r_out - p_out + y_out

        # Clamp motor signals to valid physical range [0.0, 1.0]
        motor_commands = np.clip(np.array([m1, m2, m3, m4]), 0.0, 1.0)

        return motor_commands


def quaternion_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Converts a unit quaternion [q_w, q_x, q_y, q_z] to Euler angles (Roll, Pitch, Yaw) in degrees.
    """
    qw, qx, qy, qz = q / np.linalg.norm(q)

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx**2 + qy**2)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1.0:
        pitch = np.degrees(np.sign(sinp) * (np.pi / 2.0))
    else:
        pitch = np.degrees(np.arcsin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


def run_maneuver_test(
    dynamics: FlightDynamics,
    controller: FlightController,
    maneuver_preset: str = "sharp_turn",
    total_time: float = 5.0,
    dt: float = 0.001
) -> Dict[str, List[float]]:
    """
    Runs a 5-second simulated flight test tracking a specific dynamic maneuver preset.

    Args:
        dynamics: 6-DOF FlightDynamics instance
        controller: FlightController instance
        maneuver_preset: "sharp_turn" (180-deg flip and pivot) or "punch_out" (straight acceleration)
        total_time: Simulation run time (seconds)
        dt: Timestep (seconds, default 0.001 corresponding to 1kHz)
    Returns:
        A dictionary containing historical telemetry data lists.
    """
    # Reset systems
    controller.reset()
    dynamics.propulsion.reset_thermal()

    # Initial state (position = [0,0,-2] (2 meters height in NED), vel=[0,0,0], q=[1,0,0,0], omega=[0,0,0])
    state = np.array([0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Battery capacity tracking: start fully charged
    capacity_mah = dynamics.propulsion.battery.capacity_mah
    remaining_mah = capacity_mah
    soc = 1.0

    # Telemetry storage
    telemetry: Dict[str, List[float]] = {
        "time": [],
        "pos_x": [], "pos_y": [], "pos_z": [],
        "vel_x": [], "vel_y": [], "vel_z": [],
        "roll": [], "pitch": [], "yaw": [],
        "rate_p": [], "rate_q": [], "rate_r": [],
        "sp_p": [], "sp_q": [], "sp_r": [],
        "motor_1": [], "motor_2": [], "motor_3": [], "motor_4": [],
        "v_bat": [], "soc": [],
        "motor_temp_rise": [],
        "current_total": [],
        "g_force": [],
        "throttle": []
    }

    n_steps = int(total_time / dt)

    for step in range(n_steps):
        t = step * dt

        # Define maneuver command profile (desired rates in deg/s, throttle from 0.0 to 1.0)
        if maneuver_preset == "extreme_maneuver":
            # Sudden 100% throttle punch-out combined with sharp 180-degree roll command at t=1.0s
            if t < 1.0:
                # Hover phase
                throttle_sp = 0.36
                rate_sp = (0.0, 0.0, 0.0)
            elif t < 1.5:
                # 100% throttle punch + sharp 360 deg/s roll (results in 180 deg turn in 0.5s)
                throttle_sp = 1.0
                rate_sp = (360.0, 0.0, 0.0)
            elif t < 2.5:
                # Continue full punch vertical acceleration
                throttle_sp = 1.0
                rate_sp = (0.0, 0.0, 0.0)
            elif t < 3.5:
                # Idle descent
                throttle_sp = 0.10
                rate_sp = (0.0, 0.0, 0.0)
            else:
                # Settle back to hover
                throttle_sp = 0.36
                rate_sp = (0.0, 0.0, 0.0)

        elif maneuver_preset == "sharp_turn":
            # Preset: sharp 180-degree turn
            if t < 0.5:
                # Hover phase
                throttle_sp = 0.36
                rate_sp = (0.0, 0.0, 0.0)
            elif t < 1.0:
                # Accelerate forward by pitching down
                throttle_sp = 0.45
                rate_sp = (0.0, -90.0, 0.0)  # Pitch down at 90 deg/s
            elif t < 1.3:
                # Roll 180 degrees rapidly
                throttle_sp = 0.35
                rate_sp = (360.0, 0.0, 0.0)  # Roll right at 360 deg/s
            elif t < 1.8:
                # Pitch up hard to pivot and reverse velocity
                throttle_sp = 0.80  # Full power punch
                rate_sp = (0.0, 270.0, 180.0)  # Pitch up + Yaw to pivot
            elif t < 2.5:
                # Stabilize rates
                throttle_sp = 0.30
                rate_sp = (0.0, 0.0, 0.0)
            else:
                # Hover and settle
                throttle_sp = 0.36
                rate_sp = (0.0, 0.0, 0.0)

        elif maneuver_preset == "punch_out":
            # Preset: vertical punch out
            if t < 0.5:
                throttle_sp = 0.36
                rate_sp = (0.0, 0.0, 0.0)
            elif t < 2.5:
                throttle_sp = 0.90  # 90% throttle punch
                rate_sp = (0.0, 0.0, 0.0)
            elif t < 3.5:
                throttle_sp = 0.10  # Zero throttle drop
                rate_sp = (0.0, 0.0, 0.0)
            else:
                throttle_sp = 0.36  # Recover hover
                rate_sp = (0.0, 0.0, 0.0)
        else:
            # Default to steady hover
            throttle_sp = 0.36
            rate_sp = (0.0, 0.0, 0.0)

        # Get measured body rates (omega is in rad/s in the dynamics state, convert to deg/s)
        rates_deg_s = np.degrees(state[10:13])

        # Get battery terminal voltage (estimate from previous step or nominal)
        if len(telemetry["v_bat"]) > 0:
            v_bat = telemetry["v_bat"][-1]
        else:
            # OCV at start
            v_bat = dynamics.propulsion.get_cell_ocv(soc) * dynamics.propulsion.battery.cells_s

        # Update controller
        motor_commands = controller.get_control_outputs(
            rate_setpoints=rate_sp,
            rates_measured=(rates_deg_s[0], rates_deg_s[1], rates_deg_s[2]),
            throttle=throttle_sp,
            v_battery_terminal=v_bat,
            dt=dt
        )

        # Run dynamics integration step
        state = dynamics.step_rk4(state, motor_commands, soc, dt)

        # Run a side computation using propulsion to fetch detailed telemetry for logging
        total_current_a = 0.0
        max_motor_temp = 0.0
        bat_terminal_v = v_bat

        for m_idx in range(4):
            m_res = dynamics.propulsion.evaluate(
                throttle=motor_commands[m_idx],
                current_soc=soc,
                dt=dt
            )
            # Accumulate current and find peak motor temperature
            # Under buck converter model, total current is sum of motor currents scaled by duty cycle
            if dynamics.propulsion.use_buck_model:
                total_current_a += dynamics.propulsion.num_rotors/4.0 * motor_commands[m_idx] * m_res["current_motor_a"]
            else:
                total_current_a += m_res["current_motor_a"]
            
            max_motor_temp = max(max_motor_temp, m_res["motor_temp_rise"])
            bat_terminal_v = m_res["voltage_battery_terminal_v"]

        # Update battery SOC
        # dq (mAh) = Current (A) * dt (s) / 3600 * 1000
        dq_mah = (total_current_a * dt / 3.6)
        remaining_mah = max(0.0, remaining_mah - dq_mah)
        soc = remaining_mah / capacity_mah

        # Calculate acceleration and G-force
        dstate = dynamics.state_derivative(state, motor_commands, soc, dt)
        accel_magnitude = np.linalg.norm(dstate[3:6])
        g_force = accel_magnitude / dynamics.g

        # Log telemetry data
        roll, pitch, yaw = quaternion_to_euler(state[6:10])
        
        telemetry["time"].append(t)
        telemetry["pos_x"].append(state[0])
        telemetry["pos_y"].append(state[1])
        telemetry["pos_z"].append(state[2])
        telemetry["vel_x"].append(state[3])
        telemetry["vel_y"].append(state[4])
        telemetry["vel_z"].append(state[5])
        telemetry["roll"].append(roll)
        telemetry["pitch"].append(pitch)
        telemetry["yaw"].append(yaw)
        telemetry["rate_p"].append(rates_deg_s[0])
        telemetry["rate_q"].append(rates_deg_s[1])
        telemetry["rate_r"].append(rates_deg_s[2])
        telemetry["sp_p"].append(rate_sp[0])
        telemetry["sp_q"].append(rate_sp[1])
        telemetry["sp_r"].append(rate_sp[2])
        telemetry["motor_1"].append(motor_commands[0])
        telemetry["motor_2"].append(motor_commands[1])
        telemetry["motor_3"].append(motor_commands[2])
        telemetry["motor_4"].append(motor_commands[3])
        telemetry["v_bat"].append(bat_terminal_v)
        telemetry["soc"].append(soc)
        telemetry["motor_temp_rise"].append(max_motor_temp)
        telemetry["current_total"].append(total_current_a)
        telemetry["g_force"].append(g_force)
        telemetry["throttle"].append(throttle_sp)

    return telemetry



def get_resource_path(relative_path: str) -> str:
    """
    Resolves a resource file path that works both in development and inside
    a PyInstaller --onefile bundle. When running from a frozen .exe, PyInstaller
    extracts bundled data files into a temporary directory referenced by sys._MEIPASS.

    Args:
        relative_path: The relative path to the resource file (e.g. 'rocket_simulation.csv').
    Returns:
        Absolute path to the resource file.
    """
    import sys
    import os
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)


def load_rocket_profile(filepath: str) -> dict:
    """
    Parses an OpenRocket CSV simulation export.
    Filters out comment lines starting with '#' and extracts columns:
    Time, Altitude, Vertical Velocity, Total Acceleration, Position East, Position North.
    """
    import os
    import numpy as np

    # Resolve path for PyInstaller compatibility
    filepath = get_resource_path(filepath)
    
    time_arr = []
    alt_arr = []
    v_vel_arr = []
    accel_arr = []
    east_arr = []
    north_arr = []
    
    if not os.path.exists(filepath):
        # Try finding in the same folder as this script if absolute path doesn't exist
        dir_path = os.path.dirname(os.path.abspath(__file__))
        local_path = os.path.join(dir_path, os.path.basename(filepath))
        if os.path.exists(local_path):
            filepath = local_path
            
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            line_str = line.strip()
            if not line_str or line_str.startswith('#'):
                continue
            parts = line_str.split(',')
            if len(parts) >= 6:
                try:
                    time_arr.append(float(parts[0]))
                    alt_arr.append(float(parts[1]))
                    v_vel_arr.append(float(parts[2]))
                    accel_arr.append(float(parts[3]))
                    east_arr.append(float(parts[4]))
                    north_arr.append(float(parts[5]))
                except ValueError:
                    continue
                    
    return {
        "time": np.array(time_arr),
        "altitude": np.array(alt_arr),
        "vertical_velocity": np.array(v_vel_arr),
        "total_acceleration": np.array(accel_arr),
        "position_east": np.array(east_arr),
        "position_north": np.array(north_arr)
    }


def generate_optimal_drone_path(rocket_data: dict, propulsion_sys=None, override_mass_g: float = None) -> dict:
    """
    Generates the reference drone flight path based on the rocket profile.
    Coordinates horizontal and vertical tracking to follow the rocket,
    and runs the 180s multi-physics mission profile simulation.
    """
    import numpy as np
    
    if propulsion_sys is None:
        propulsion_sys = PropulsionSystem()
        
    time = rocket_data["time"]
    alt_rocket = rocket_data["altitude"]
    east_rocket = rocket_data["position_east"]
    north_rocket = rocket_data["position_north"]
    
    n_points = len(time)
    
    # Coordinates in our coordinate frame
    X_rocket = -east_rocket
    Y_rocket = north_rocket
    Z_rocket = alt_rocket
    
    pos_x = np.zeros(n_points)
    pos_y = np.zeros(n_points)
    pos_z = np.zeros(n_points)
    
    v_x = np.zeros(n_points)
    v_y = np.zeros(n_points)
    v_z = np.zeros(n_points)
    
    a_x = np.zeros(n_points)
    a_y = np.zeros(n_points)
    a_z = np.zeros(n_points)
    
    # Set starting position (Stage 1)
    pos_x[0] = -10.0
    pos_y[0] = 0.0
    pos_z[0] = 250.0
    
    top_speed = 44.444  # m/s
    a_z_max = 49.03325  # 5G m/s^2
    
    # 1. Kinematic Path Planning
    for i in range(1, n_points):
        t = time[i]
        dt = t - time[i-1]
        if dt <= 0:
            pos_x[i] = pos_x[i-1]
            pos_y[i] = pos_y[i-1]
            pos_z[i] = pos_z[i-1]
            continue
            
        if t <= 0.696:
            # Stage 1: Pre-stage Hover
            pos_x[i] = -10.0
            pos_y[i] = 0.0
            pos_z[i] = 250.0
            v_x[i] = 0.0
            v_y[i] = 0.0
            v_z[i] = 0.0
        elif t <= 16.643:
            # Stage 2: High-G Intercept
            # Vertical velocity increases with max 5G vertical acceleration
            v_z_raw = v_z[i-1] + a_z_max * dt
            
            # Horizontal tracking of rocket
            dx = X_rocket[i] - pos_x[i-1]
            dy = Y_rocket[i] - pos_y[i-1]
            v_x_raw = dx / dt
            v_y_raw = dy / dt
            
            # Global speed limit
            v_speed = np.sqrt(v_x_raw**2 + v_y_raw**2 + v_z_raw**2)
            if v_speed > top_speed:
                v_x[i] = v_x_raw * (top_speed / v_speed)
                v_y[i] = v_y_raw * (top_speed / v_speed)
                v_z[i] = v_z_raw * (top_speed / v_speed)
            else:
                v_x[i] = v_x_raw
                v_y[i] = v_y_raw
                v_z[i] = v_z_raw
                
            pos_x[i] = pos_x[i-1] + v_x[i] * dt
            pos_y[i] = pos_y[i-1] + v_y[i] * dt
            pos_z[i] = pos_z[i-1] + v_z[i] * dt
        else:
            # Stage 3: Parachute Companion Descent
            pos_x[i] = X_rocket[i] - 20.0
            pos_y[i] = Y_rocket[i] + 20.0
            pos_z[i] = Z_rocket[i] + 20.0
            
            v_x[i] = (pos_x[i] - pos_x[i-1]) / dt
            v_y[i] = (pos_y[i] - pos_y[i-1]) / dt
            v_z[i] = (pos_z[i] - pos_z[i-1]) / dt
            
        a_x[i] = (v_x[i] - v_x[i-1]) / dt
        a_y[i] = (v_y[i] - v_y[i-1]) / dt
        a_z[i] = (v_z[i] - v_z[i-1]) / dt
        
    # 2. Multi-Physics Simulation
    # Calculate drone All-Up Weight (AUW) dynamically
    if override_mass_g is not None:
        auw_g = override_mass_g
    else:
        m_hub = 556.4
        m_arm = 20.0
        m_prop = 8.9
        m_motor = propulsion_sys.motor.weight_g
        m_batt = getattr(propulsion_sys.battery, 'weight_g', 908.0)
        auw_g = m_hub + 4.0 * m_arm + 4.0 * (m_motor + m_prop) + m_batt
    
    capacity_mah = propulsion_sys.battery.capacity_mah
    remaining_mah = capacity_mah
    soc = 1.0
    propulsion_sys.reset_thermal()
    
    # Setup analytical hover throttle calculation helper
    def get_hover_throttle(s):
        g_grav = 9.80665
        w_total = (auw_g / 1000.0) * g_grav
        t_hover_per_motor = w_total / 4.0
        d_m = propulsion_sys.prop_diameter_m
        n_hover = np.sqrt(max(0.0, t_hover_per_motor / (propulsion_sys.propeller.ct * propulsion_sys.rho * (d_m ** 4))))
        rpm_hover = n_hover * 60.0
        A_factor = (propulsion_sys.motor.kv * propulsion_sys.propeller.cp * propulsion_sys.rho * (d_m ** 5)) / 216000.0
        i_motor_hover = propulsion_sys.motor.i_no_load + A_factor * (rpm_hover ** 2)
        v_ocv = propulsion_sys.get_cell_ocv(s) * propulsion_sys.battery.cells_s
        r_batt = propulsion_sys.battery.r_cell * propulsion_sys.battery.cells_s
        denom = v_ocv - 4.0 * i_motor_hover * r_batt
        r_esc_motor = propulsion_sys.esc.r_esc + propulsion_sys.motor.r_motor
        if denom > 0:
            d_hover = (rpm_hover / propulsion_sys.motor.kv + i_motor_hover * r_esc_motor) / denom
        else:
            d_hover = 1.0
        return float(np.clip(d_hover, 0.0, 1.0))
        
    d_hover = get_hover_throttle(1.0)
    
    v_bat = np.zeros(n_points)
    soc_arr = np.zeros(n_points)
    motor_temp_rise = np.zeros(n_points)
    current_total = np.zeros(n_points)
    g_force = np.zeros(n_points)
    throttle = np.zeros(n_points)
    tracking_error = np.zeros(n_points)
    gimbal_pitch = np.zeros(n_points)
    
    # Store initial state physics
    v_bat[0] = propulsion_sys.get_cell_ocv(1.0) * propulsion_sys.battery.cells_s
    soc_arr[0] = 1.0
    motor_temp_rise[0] = 0.0
    current_total[0] = 0.0
    g_force[0] = 1.0
    throttle[0] = d_hover
    tracking_error[0] = np.sqrt((X_rocket[0] - pos_x[0])**2 + (Y_rocket[0] - pos_y[0])**2 + (Z_rocket[0] - pos_z[0])**2)
    dist_horiz0 = np.sqrt((X_rocket[0] - pos_x[0])**2 + (Y_rocket[0] - pos_y[0])**2)
    gimbal_pitch[0] = np.degrees(np.arctan2(Z_rocket[0] - pos_z[0], dist_horiz0))
    
    for i in range(1, n_points):
        t = time[i]
        dt = t - time[i-1]
        
        # 1. Required throttle
        if t <= 0.696:
            thr = get_hover_throttle(soc)
        elif t <= 16.643:
            thr = 1.0
        else:
            thr = 0.22
            
        throttle[i] = thr
        
        # 2. Evaluate propulsion system
        m_res = propulsion_sys.evaluate(throttle=thr, current_soc=soc, dt=dt)
        
        total_curr = m_res["current_total_a"]
        current_total[i] = total_curr
        v_bat[i] = m_res["voltage_battery_terminal_v"]
        motor_temp_rise[i] = m_res["motor_temp_rise"]
        
        # Draining battery
        dq_mah = (total_curr * dt) / 3.6
        remaining_mah = max(0.0, remaining_mah - dq_mah)
        soc = remaining_mah / capacity_mah
        soc_arr[i] = soc
        
        # 3. G-Force calculation
        a_mag = np.sqrt(a_x[i]**2 + a_y[i]**2 + a_z[i]**2)
        g_force[i] = float(np.clip(a_mag / 9.80665, 1.0, 5.34))
        
        # 4. Geometry analytics
        tracking_error[i] = np.sqrt((X_rocket[i] - pos_x[i])**2 + (Y_rocket[i] - pos_y[i])**2 + (Z_rocket[i] - pos_z[i])**2)
        dist_h = np.sqrt((X_rocket[i] - pos_x[i])**2 + (Y_rocket[i] - pos_y[i])**2)
        gimbal_pitch[i] = np.degrees(np.arctan2(Z_rocket[i] - pos_z[i], dist_h))
        
    # Calculate additional dynamic and attitude parameters
    velocity = np.sqrt(v_x**2 + v_y**2 + v_z**2)
    span_horizontal = -pos_x
    span_longitudinal = pos_y
    
    # Attitude angles estimation based on acceleration and gravity
    g_const = 9.80665
    pitch = np.degrees(np.arctan2(-a_y, a_z + g_const))
    roll = np.degrees(np.arctan2(a_x, a_z + g_const))
    
    yaw = np.zeros(n_points)
    for i in range(1, n_points):
        if np.sqrt(v_x[i]**2 + v_y[i]**2) > 0.1:
            yaw[i] = np.degrees(np.arctan2(-v_x[i], v_y[i]))
        else:
            yaw[i] = yaw[i-1]
            
    return {
        "pos_x": pos_x,
        "pos_y": pos_y,
        "pos_z": pos_z,
        "gimbal_pitch": gimbal_pitch,
        "tracking_error": tracking_error,
        "throttle": throttle,
        "v_bat": v_bat,
        "soc": soc_arr,
        "motor_temp_rise": motor_temp_rise,
        "current_total": current_total,
        "g_force": g_force,
        "velocity": velocity,
        "span_horizontal": span_horizontal,
        "span_longitudinal": span_longitudinal,
        "pitch": pitch,
        "roll": roll,
        "yaw": yaw
    }


# Diagnostic runner to test flight controller and maneuver test
if __name__ == "__main__":
    sys = PropulsionSystem()
    dyn = FlightDynamics(propulsion_sys=sys)
    ctrl = FlightController()
    
    print("=== Flight Controller Maneuver Test Diagnostic ===")
    print("Simulating sharp turn maneuver...")
    tel = run_maneuver_test(dyn, ctrl, maneuver_preset="sharp_turn")
    
    print("\nTelemetry Results summary (after 5.0 seconds):")
    print(f"Final Position:   [{tel['pos_x'][-1]:.3f}, {tel['pos_y'][-1]:.3f}, {tel['pos_z'][-1]:.3f}] m")
    print(f"Max G-Force:      {max(tel['g_force']):.2f} G")
    print(f"Max Motor Temp:   {max(tel['motor_temp_rise']):.2f} K rise")
    print(f"Min Bat Voltage:  {min(tel['v_bat']):.2f} V")
    print(f"Final Battery SOC: {tel['soc'][-1]*100:.1f} %")
