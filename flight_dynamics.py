"""
6-DOF Rigid Body Flight Dynamics Simulation Library for Multi-Rotors
Author: Aerospace Simulation Engineer & Flight Dynamics Expert
Description: Implements rigid body dynamics equations of motion for a quadcopter,
             with dynamic Moment of Inertia calculations, 3D aerodynamic drag,
             gyroscopic propeller coupling, and Runge-Kutta 4th Order (RK4) integration.
"""

from typing import Dict, Any, Tuple
import numpy as np
from propulsion_engine import BatteryConfig, ESCConfig, MotorConfig, PropellerConfig, PropulsionSystem


class FlightDynamics:
    """
    Implements 6-Degree-of-Freedom (6-DOF) rigid body equations of motion for a quadcopter.
    Coordinates inertial and body reference frames using quaternions.
    """

    def __init__(
        self,
        diagonal_size_m: float = 0.300,       # Motor-to-motor diagonal distance (m)
        m_hub_g: float = 556.4,               # Central hub mass (g) (Foxeer Aura plates + payload + avionics)
        m_arm_g: float = 20.0,                # Individual arm mass (g)
        m_prop_g: float = 8.9,                # Propeller mass (g) (Gemfan 7050)
        battery_mount: str = "top",           # "top" or "bottom" battery mounting
        battery_dim_mm: Tuple[float, float, float] = (46.0, 160.0, 51.0), # W, L, H in mm
        drag_area_m2: Tuple[float, float, float] = (0.076, 0.080, 0.065), # Ax, Ay, Az (frontal, side, planform) (160km/h cap)
        drag_coeff: Tuple[float, float, float] = (1.2, 1.2, 1.3),        # Cdx, Cdy, Cdz
        rotational_drag: Tuple[float, float, float] = (0.01, 0.01, 0.015), # Roll, Pitch, Yaw damping coefficients
        propulsion_sys: PropulsionSystem = PropulsionSystem()
    ):
        """
        Initialize the 6-DOF dynamics model.

        Args:
            diagonal_size_m: Diagonal motor-to-motor distance (m)
            m_hub_g: Mass of central frame body (g)
            m_arm_g: Mass of a single arm structure (g)
            m_prop_g: Mass of a single propeller (g)
            battery_mount: Placement of battery ("top" or "bottom")
            battery_dim_mm: Battery physical dimensions (Width, Length, Height) in mm
            drag_area_m2: Cross-sectional drag areas (Ax, Ay, Az) in body frame (m^2)
            drag_coeff: Drag coefficients (Cdx, Cdy, Cdz) for each axis
            rotational_drag: Rotational damping coefficients for body-axis damping
            propulsion_sys: Instance of the PropulsionSystem module
        """
        self.diagonal_size_m = diagonal_size_m
        self.m_hub_g = m_hub_g
        self.m_arm_g = m_arm_g
        self.m_prop_g = m_prop_g
        self.battery_mount = battery_mount
        self.battery_dim_mm = battery_dim_mm
        self.drag_area_m2 = np.array(drag_area_m2)
        self.drag_coeff = np.array(drag_coeff)
        self.rotational_drag = np.array(rotational_drag)
        self.propulsion = propulsion_sys

        # Environmental constants
        self.g = 9.80665  # Gravity m/s^2

        # Motor geometry configuration (Betaflight X frame)
        # Motor distance along body x and y axes: arm length L = diagonal / 2
        # For a true symmetric X frame: L_x = L_y = L * cos(45 deg)
        self.L = diagonal_size_m / 2.0
        self.L_x = self.L * np.cos(np.radians(45))
        self.L_y = self.L * np.sin(np.radians(45))

        # Motor layout positions in body frame (x: forward, y: right, z: down)
        # Motor 1: Rear Right  (CCW)
        # Motor 2: Front Right (CW)
        # Motor 3: Rear Left   (CW)
        # Motor 4: Front Left  (CCW)
        self.motor_pos = np.array([
            [-self.L_x,  self.L_y, 0.0],  # Motor 1
            [ self.L_x,  self.L_y, 0.0],  # Motor 2
            [-self.L_x, -self.L_y, 0.0],  # Motor 3
            [ self.L_x, -self.L_y, 0.0]   # Motor 4
        ])

        # Motor rotation spin signs (CW = +1, CCW = -1)
        self.motor_spins = np.array([-1, 1, 1, -1])

        # Propeller polar moment of inertia (J_prop): modeled as thin rod
        self.J_prop = (1.0 / 12.0) * (m_prop_g / 1000.0) * (self.propulsion.prop_diameter_m ** 2)

        # Compute total mass and Inertia Tensor
        self.mass, self.inertia = self._calculate_mass_properties()

    def _calculate_mass_properties(self) -> Tuple[float, np.ndarray]:
        """
        Calculates the quadcopter's total All-Up-Weight (AUW) and Moment of Inertia Tensor (I)
        around the center of mass using the Parallel Axis Theorem.

        Returns:
            total_mass: Total mass (kg)
            inertia_tensor: 3x3 diagonal Inertia Tensor (kg-m^2)
        """
        # Convert masses to kg
        m_hub = self.m_hub_g / 1000.0
        m_arm = self.m_arm_g / 1000.0
        m_motor = self.propulsion.motor.weight_g / 1000.0
        m_prop = self.m_prop_g / 1000.0
        m_batt = getattr(self.propulsion.battery, 'weight_g', 908.0) / 1000.0
        
        # Physical dimensions in meters
        w_hub, l_hub, h_hub = 0.06, 0.06, 0.03
        w_batt = self.battery_dim_mm[0] / 1000.0
        l_batt = self.battery_dim_mm[1] / 1000.0
        h_batt = self.battery_dim_mm[2] / 1000.0

        # 1. Calculate battery vertical offset (Z axis in NED frame: positive is down, negative is up)
        # Configure battery center of mass with top-mount offset of 25mm vertically up (-25mm in NED)
        z_batt_offset = -0.025 if self.battery_mount == "top" else 0.025

        # 2. Compute Total Mass (kg)
        total_mass = m_hub + 4.0 * m_arm + 4.0 * (m_motor + m_prop) + m_batt

        # 3. Compute System Center of Mass (CM) along Z axis (X and Y are symmetric at 0)
        # Z_cm = sum(m_i * z_i) / sum(m_i)
        z_cm = (m_batt * z_batt_offset) / total_mass

        # 4. Calculate Moments of Inertia around the system Center of Mass
        # Z-shift distances for Parallel Axis Theorem
        dz_hub = -z_cm
        dz_arms = -z_cm
        dz_motors = -z_cm
        dz_batt = z_batt_offset - z_cm

        # Central Hub Inertia (Rectangular solid)
        Ixx_hub = (1.0 / 12.0) * m_hub * (l_hub**2 + h_hub**2) + m_hub * dz_hub**2
        Iyy_hub = (1.0 / 12.0) * m_hub * (w_hub**2 + h_hub**2) + m_hub * dz_hub**2
        Izz_hub = (1.0 / 12.0) * m_hub * (w_hub**2 + l_hub**2)

        # Arms Inertia (Modeled as 4 diagonal rods of length L)
        # Summed rotated local inertias + parallel axis translation
        Ixx_arms = (2.0 / 3.0) * m_arm * (self.L ** 2) + 4.0 * m_arm * dz_arms**2
        Iyy_arms = (2.0 / 3.0) * m_arm * (self.L ** 2) + 4.0 * m_arm * dz_arms**2
        Izz_arms = (4.0 / 3.0) * m_arm * (self.L ** 2)

        # Motors & Propellers (Modeled as point masses at arm tips)
        m_rotor_group = m_motor + m_prop
        Ixx_motors = 2.0 * m_rotor_group * (self.L ** 2) + 4.0 * m_rotor_group * dz_motors**2
        Iyy_motors = 2.0 * m_rotor_group * (self.L ** 2) + 4.0 * m_rotor_group * dz_motors**2
        Izz_motors = 4.0 * m_rotor_group * (self.L ** 2)

        # Battery Inertia (Rectangular solid offset along Z)
        Ixx_batt = (1.0 / 12.0) * m_batt * (l_batt**2 + h_batt**2) + m_batt * dz_batt**2
        Iyy_batt = (1.0 / 12.0) * m_batt * (w_batt**2 + h_batt**2) + m_batt * dz_batt**2
        Izz_batt = (1.0 / 12.0) * m_batt * (w_batt**2 + l_batt**2)

        # Combined Total Inertias
        Ixx = Ixx_hub + Ixx_arms + Ixx_motors + Ixx_batt
        Iyy = Iyy_hub + Iyy_arms + Iyy_motors + Iyy_batt
        Izz = Izz_hub + Izz_arms + Izz_motors + Izz_batt

        inertia_tensor = np.array([
            [Ixx, 0.0, 0.0],
            [0.0, Iyy, 0.0],
            [0.0, 0.0, Izz]
        ])

        return total_mass, inertia_tensor

    @staticmethod
    def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        Converts a unit quaternion [q_w, q_x, q_y, q_z] to a 3x3 rotation matrix R (body -> inertial).
        """
        qw, qx, qy, qz = q / np.linalg.norm(q)
        return np.array([
            [1.0 - 2.0 * (qy**2 + qz**2), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
            [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx**2 + qz**2), 2.0 * (qy * qz - qw * qx)],
            [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx**2 + qy**2)]
        ])

    @staticmethod
    def quaternion_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
        """
        Calculates the time derivative of the unit quaternion.
        dq/dt = 0.5 * q * [0, omega]
        """
        qw, qx, qy, qz = q
        p, q_pitch, r_yaw = omega

        dq = 0.5 * np.array([
            -qx * p - qy * q_pitch - qz * r_yaw,
             qw * p + qy * r_yaw - qz * q_pitch,
             qw * q_pitch - qx * r_yaw + qz * p,
             qw * r_yaw + qx * q_pitch - qy * p
        ])
        return dq

    def state_derivative(self, state: np.ndarray, throttles: np.ndarray, soc: float, dt: float = 0.001) -> np.ndarray:
        """
        Calculates the derivative of the rigid body state vector.
        State vector: x_state = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, qw, qx, qy, qz, p, q, r]
        
        Args:
            state: Current 13-element state vector
            throttles: Motor throttles (4-element array, 0.0 to 1.0)
            soc: Current battery state of charge (0.0 to 1.0)
            dt: Timestep (used to propagate the propulsion engine's thermal state)
        """
        # Unpack state
        # pos = state[0:3], vel = state[3:6], q = state[6:10], omega = state[10:13]
        vel_i = state[3:6]
        q = state[6:10]
        omega = state[10:13]

        # Normalize quaternion to avoid numerical integration drift
        q_norm = q / np.linalg.norm(q)

        # 1. Transform inertial velocity to body frame
        R = self.quaternion_to_rotation_matrix(q_norm)
        vel_b = R.T @ vel_i

        # 2. Evaluate propulsion forces and torques for each motor
        thrusts = np.zeros(4)
        prop_torques = np.zeros(4)
        prop_rpms = np.zeros(4)

        for i in range(4):
            motor_telemetry = self.propulsion.evaluate(
                throttle=throttles[i],
                current_soc=soc,
                dt=dt
            )
            thrusts[i] = motor_telemetry["thrust_n"]
            prop_rpms[i] = motor_telemetry["motor_rpm"]
            # Aerodynamic drag torque of the propeller (Q = P / omega)
            omega_prop = prop_rpms[i] * (2.0 * np.pi / 60.0)
            if omega_prop > 0:
                prop_torques[i] = motor_telemetry["mechanical_power_w"] / omega_prop
            else:
                prop_torques[i] = 0.0

        # Total upward thrust in body frame (acts along -Z body axis)
        total_thrust_z_b = -np.sum(thrusts)
        F_thrust_b = np.array([0.0, 0.0, total_thrust_z_b])

        # Control Torques (mixer torque outputs)
        # Roll torque: left motors (2, 4) - right motors (1, 3)
        tau_ctrl_x = (thrusts[1] + thrusts[3] - thrusts[0] - thrusts[2]) * self.L_y  # Wait! Index 0, 1, 2, 3 correspond to motors 1, 2, 3, 4
        # Let's map indexes correctly:
        # Motor 1 (Rear Right) -> index 0
        # Motor 2 (Front Right) -> index 1
        # Motor 3 (Rear Left) -> index 2
        # Motor 4 (Front Left) -> index 3
        # Left motors are 3 and 4 (index 2 and 3). Right motors are 1 and 2 (index 0 and 1).
        # Roll Right (+x rotation): left motors push up (negative z force), right motors push down
        # So tau_roll = (T_left - T_right) * L_y = (T3 + T4 - T1 - T2) * L_y
        tau_ctrl_x = (thrusts[2] + thrusts[3] - thrusts[0] - thrusts[1]) * self.L_y

        # Pitch torque: rear motors (1, 3) - front motors (2, 4)
        # Pitch Up (+y rotation): front motors push up, rear motors push down
        # So tau_pitch = (T_front - T_rear) * L_x = (T2 + T4 - T1 - T3) * L_x
        # Let's check indices: Front motors are 2 and 4 (index 1 and 3). Rear motors are 1 and 3 (index 0 and 2).
        # So tau_ctrl_y = (T_front - T_rear) * L_x = (thrusts[1] + thrusts[3] - thrusts[0] - thrusts[2]) * self.L_x
        tau_ctrl_y = (thrusts[1] + thrusts[3] - thrusts[0] - thrusts[2]) * self.L_x

        # Yaw torque: sum of propeller drag reaction moments
        # CCW spin creates CW (+yaw) reaction torque. CW spin creates CCW (-yaw) reaction torque.
        # Motor 1 (0): CCW (+), Motor 2 (1): CW (-), Motor 3 (2): CW (-), Motor 4 (3): CCW (+)
        tau_ctrl_z = np.sum(-self.motor_spins * prop_torques)  # CCW motors create +Z reaction torque

        tau_ctrl = np.array([tau_ctrl_x, tau_ctrl_y, tau_ctrl_z])

        # 3. 3D Aerodynamic Drag
        # Project velocity direction relative to body axis to scale Cd and Area
        vel_mag = np.linalg.norm(vel_b)
        if vel_mag > 1e-4:
            vel_unit_b = vel_b / vel_mag
            # Effective area and drag coefficient via linear blending
            eff_area = np.sum(np.abs(vel_unit_b) * self.drag_area_m2)
            eff_cd = np.sum(np.abs(vel_unit_b) * self.drag_coeff)
            # Drag force in body frame (F_drag = -0.5 * rho * v^2 * A * Cd)
            F_drag_b = -0.5 * self.propulsion.rho * (vel_mag ** 2) * eff_area * eff_cd * vel_unit_b
        else:
            F_drag_b = np.zeros(3)

        # Aerodynamic rotational damping torques
        tau_drag_b = -self.rotational_drag * omega * vel_mag

        # 4. Gyroscopic Propeller Coupling Torque
        # H_z = J_prop * sum(spin_i * omega_prop_i)
        H_z = self.J_prop * np.sum(self.motor_spins * (prop_rpms * (2.0 * np.pi / 60.0)))
        # tau_gyro = -omega x [0, 0, H_z]^T = [-q * Hz, p * Hz, 0]^T
        tau_gyro = np.array([
            -omega[1] * H_z,
             omega[0] * H_z,
             0.0
        ])

        # 5. Combine forces & torques
        # Total force in body frame
        F_total_b = F_thrust_b + F_drag_b
        # Total force in inertial frame (rotate + add gravity)
        F_total_i = R @ F_total_b
        # Gravity force: mass * [0, 0, g] (downwards positive in NED)
        gravity_i = np.array([0.0, 0.0, self.mass * self.g])
        accel_i = (F_total_i + gravity_i) / self.mass

        # Total torques in body frame
        tau_total_b = tau_ctrl + tau_gyro + tau_drag_b

        # 6. Rotational Acceleration (Euler's Equations)
        # d_omega/dt = I^-1 * (tau - omega x (I * omega))
        I_inv = np.linalg.inv(self.inertia)
        omega_cross_Iomega = np.cross(omega, self.inertia @ omega)
        alpha_b = I_inv @ (tau_total_b - omega_cross_Iomega)

        # 7. Coordinate Kinematics Derivatives
        # Position derivative is velocity
        dpos_dt = vel_i
        # Quaternion derivative
        dq_dt = self.quaternion_derivative(q_norm, omega)

        # Assemble state derivative vector
        dstate_dt = np.zeros(13)
        dstate_dt[0:3] = dpos_dt    # x_dot, y_dot, z_dot
        dstate_dt[3:6] = accel_i    # vx_dot, vy_dot, vz_dot
        dstate_dt[6:10] = dq_dt     # qw_dot, qx_dot, qy_dot, qz_dot
        dstate_dt[10:13] = alpha_b  # p_dot, q_dot, r_dot

        return dstate_dt

    def step_rk4(self, state: np.ndarray, throttles: np.ndarray, soc: float, dt: float = 0.001) -> np.ndarray:
        """
        Integrates the rigid body state by one timestep dt using Runge-Kutta 4th Order.

        Args:
            state: Current state vector (13-element)
            throttles: Motor throttles (4-element array, 0.0 to 1.0)
            soc: Battery state of charge (0.0 to 1.0)
            dt: Time step in seconds
        Returns:
            Next state vector (13-element)
        """
        k1 = self.state_derivative(state, throttles, soc, dt)
        k2 = self.state_derivative(state + 0.5 * dt * k1, throttles, soc, dt)
        k3 = self.state_derivative(state + 0.5 * dt * k2, throttles, soc, dt)
        k4 = self.state_derivative(state + dt * k3, throttles, soc, dt)

        next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Re-normalize quaternion to prevent accumulation of numerical error
        q = next_state[6:10]
        next_state[6:10] = q / np.linalg.norm(q)

        return next_state


# Diagnostic runner to test flight dynamics calculations when executed directly
if __name__ == "__main__":
    # Create flight dynamics system
    dyn = FlightDynamics()
    print("=== 6-DOF Flight Dynamics Diagnostic ===")
    print(f"Quadcopter AUW: {dyn.mass:.4f} kg ({dyn.mass*1000:.1f} g)")
    print(f"Inertia Tensor: \nI_xx: {dyn.inertia[0,0]:.6f} kg-m^2\nI_yy: {dyn.inertia[1,1]:.6f} kg-m^2\nI_zz: {dyn.inertia[2,2]:.6f} kg-m^2")

    # Initial state: hovering, zero velocity, identity rotation
    # State: pos=[0,0,0], vel=[0,0,0], q=[1,0,0,0], omega=[0,0,0]
    init_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Find approximate hover throttle: weight Force / (4 * max thrust Force)
    hover_res = dyn.propulsion.evaluate(throttle=0.5, current_soc=1.0)
    print(f"Thrust per motor at 50%: {hover_res['thrust_g']:.1f} g")

    # Step RK4 with hover throttle (e.g. 36% throttle to balance ~1900g total weight)
    # Let's verify derivative under hover
    dstate = dyn.state_derivative(init_state, np.array([0.36, 0.36, 0.36, 0.36]), soc=1.0)
    print(f"Acceleration at ~36% throttle: {dstate[3:6]} m/s^2 (should be close to 0)")

    # Pitch command (higher rear throttles, lower front)
    pitch_throttles = np.array([0.3, 0.2, 0.3, 0.2])
    dstate_pitch = dyn.state_derivative(init_state, pitch_throttles, soc=1.0)
    print(f"Angular acceleration under pitch command: {dstate_pitch[10:13]} rad/s^2")
