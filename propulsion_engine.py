"""
Propulsion Systems Simulation Library for Multi-Rotors
Author: Senior Propulsion Systems & Mathematical Simulation Engineer
Description: Implements a high-fidelity static propulsion performance calculator for a quadcopter,
             incorporating battery state-of-charge (SOC) dynamics, voltage sag, circuit losses,
             coupled motor-propeller equilibrium solver, and motor transient thermal dynamics.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
import numpy as np


@dataclass
class BatteryConfig:
    """Configuration parameters for the battery pack."""
    cells_s: int = 6                  # Number of cells in series (S)
    capacity_mah: float = 6500.0      # Nominal capacity in mAh (dual parallel 3250mAh)
    r_cell: float = 0.003             # Internal resistance per cell (ohms) (0.018 ohms per pack)
    v_cell_nominal: float = 3.7       # Nominal voltage per cell (V)
    weight_g: float = 908.0           # Total battery pack weight (grams)


@dataclass
class ESCConfig:
    """Configuration parameters for the Electronic Speed Controller (ESC)."""
    r_esc: float = 0.0015             # ESC internal MOSFET resistance (ohms)
    max_current: float = 80.0         # Max continuous current limit (A) (APD 80F3)
    burst_current: float = 140.0      # Burst peak current limit (A)


@dataclass
class MotorConfig:
    """Configuration parameters for the Brushless DC Motor (BLDC)."""
    kv: float = 1300.0                # Motor KV rating (RPM/V) (Xnova 2812 Heavy Lift)
    i_no_load: float = 2.1            # No-load current (A) @ 10V
    r_motor: float = 0.021            # Phase winding resistance (ohms)
    weight_g: float = 80.0            # Motor mass (grams)
    max_current: float = 54.5         # Max safe current limit (A)
    specific_heat: float = 0.85       # Specific heat capacity (J/(g*K)), default copper/aluminum blend
    r_thermal_static: float = 2.5     # Thermal resistance to ambient under static conditions (K/W)
    thermal_time_constant: float = 45.0  # Thermal time constant tau (seconds)


@dataclass
class PropellerConfig:
    """Configuration parameters for the Propeller."""
    diameter_in: float = 7.0          # Diameter in inches (Gemfan 7050)
    pitch_in: float = 5.0             # Pitch in inches
    blade_count: int = 3              # Number of blades (Tri-blade)
    ct: float = 0.1115                # Thrust coefficient (dimensionless, calibrated for 2860g @ 22.2V)
    cp: float = 0.0565                # Power coefficient (dimensionless, calibrated for 57A @ 22.2V)


class PropulsionSystem:
    """
    Simulates a single rotor-motor-ESC-battery system of a quadcopter.
    Solves the coupled electro-aerodynamic equations to find the operating point.
    """

    # 21-point empirical LiPo cell Open Circuit Voltage (OCV) vs State-of-Charge (SOC)
    # SOC ranges from 0.0 (0%) to 1.0 (100%)
    _LIPO_SOC_TABLE = np.linspace(0.0, 1.0, 21)
    _LIPO_OCV_TABLE = np.array([
        3.27, 3.55, 3.62, 3.66, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79,
        3.82, 3.84, 3.87, 3.90, 3.94, 3.98, 4.02, 4.07, 4.11, 4.16, 4.20
    ])

    def __init__(
        self,
        battery: BatteryConfig = BatteryConfig(),
        esc: ESCConfig = ESCConfig(),
        motor: MotorConfig = MotorConfig(),
        propeller: PropellerConfig = PropellerConfig(),
        num_rotors: int = 4,
        rho: float = 1.225,
        use_buck_model: bool = True
    ):
        """
        Initialize the propulsion system model.

        Args:
            battery: Configuration for the battery pack.
            esc: Configuration for the ESC.
            motor: Configuration for the BLDC motor.
            propeller: Configuration for the propeller.
            num_rotors: Number of rotors (default 4 for quadcopter, scales battery current).
            rho: Air density in kg/m^3 (default: 1.225, sea level standard).
            use_buck_model: If True, uses the buck converter model for ESC current scaling.
                            If False, uses the linear current scaling model.
        """
        self.battery = battery
        self.esc = esc
        self.motor = motor
        self.propeller = propeller
        self.num_rotors = num_rotors
        self.rho = rho
        self.use_buck_model = use_buck_model

        # Calculate propeller diameter in meters
        self.prop_diameter_m = propeller.diameter_in * 0.0254

        # Track motor temperature rise (Delta T above ambient)
        self.motor_delta_t = 0.0

    def get_cell_ocv(self, soc: float) -> float:
        """
        Calculates the Open Circuit Voltage of a single cell using linear interpolation.

        Args:
            soc: State of Charge (0.0 to 1.0)
        Returns:
            Cell open circuit voltage (V)
        """
        # Clamp SOC to valid range
        soc_clamped = np.clip(soc, 0.0, 1.0)
        return float(np.interp(soc_clamped, self._LIPO_SOC_TABLE, self._LIPO_OCV_TABLE))

    def evaluate(
        self,
        throttle: float,
        current_soc: float,
        execution_time: float = 0.0,
        dt: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Solves the steady-state motor-propeller operating point at the specified throttle and SOC.
        Optionally updates the transient thermal model of the motor.

        Args:
            throttle: Input throttle command (0.0 to 1.0)
            current_soc: Current Battery State-of-Charge (0.0 to 1.0)
            execution_time: Continuous duration in seconds for thermal model update
            dt: Optional time step for iterative simulation. If provided, updates thermal state
                using dt instead of execution_time.
        Returns:
            A dictionary containing key telemetry metrics:
                - thrust_g: Static thrust per motor (g)
                - thrust_n: Static thrust per motor (N)
                - current_motor_a: Current drawn by the motor (A)
                - current_total_a: Total current drawn from the battery (A)
                - voltage_battery_terminal_v: Voltage at the battery terminals (V)
                - voltage_motor_v: Effective voltage across motor terminals (V)
                - motor_rpm: Rotational speed of the motor (RPM)
                - mechanical_power_w: Shaft power delivered to the propeller (W)
                - electrical_power_w: Electrical power delivered to the motor (W)
                - power_loss_w: Total power dissipated as heat in the single motor/ESC path (W)
                - efficiency_g_w: Propulsive efficiency (g/W)
                - motor_temp_rise: Motor temperature rise above ambient (K)
        """
        # 1. Calculate Open Circuit Voltage of the Battery pack
        cell_ocv = self.get_cell_ocv(current_soc)
        v_battery_ocv = cell_ocv * self.battery.cells_s
        r_battery = self.battery.r_cell * self.battery.cells_s

        # Clamp throttle to avoid boundary errors
        d = np.clip(throttle, 0.0, 1.0)

        # 2. Establish equivalent resistance based on selected ESC-battery model
        # Linear: R_equiv = 4 * R_battery + R_esc + R_motor (for 4 motors)
        # Buck:   R_equiv = 4 * d * R_battery + R_esc + R_motor (referred to motor current)
        if self.use_buck_model:
            # Under buck model, average battery current is I_total = num_rotors * d * I_motor
            # Yielding circuit voltage loss referred to I_motor: num_rotors * d * R_battery
            r_equiv_circuit = self.num_rotors * d * r_battery + self.esc.r_esc
        else:
            # Under linear model, average battery current is I_total = num_rotors * I_motor
            r_equiv_circuit = self.num_rotors * r_battery + self.esc.r_esc

        r_total = r_equiv_circuit + self.motor.r_motor
        v_eff = d * v_battery_ocv

        # 3. Solve the coupled electro-aerodynamic torque equations:
        # Prop Torque:  Q_prop = (Cp * rho * D^5 / (7200 * pi)) * RPM^2
        # Motor Torque: Q_motor = Kt * (I_motor - I_no_load)
        # Yields: I_motor = I_no_load + A * RPM^2
        # With RPM = KV * (V_eff - I_motor * R_total)
        # Combining gives the quadratic equation: a * I_motor^2 + b * I_motor + c = 0
        
        # Propeller torque scale factor: A = (KV * Cp * rho * D^5) / 216000
        A = (self.motor.kv * self.propeller.cp * self.rho * (self.prop_diameter_m ** 5)) / 216000.0
        K = A * (self.motor.kv ** 2)

        # Coefficients for a * I_motor^2 + b * I_motor + c = 0
        a_coeff = K * (r_total ** 2)
        b_coeff = -(2.0 * K * v_eff * r_total + 1.0)
        c_coeff = K * (v_eff ** 2) + self.motor.i_no_load

        # Guard against zero throttle / low voltage conditions where the motor cannot spin
        if d < 1e-4 or v_eff <= r_total * self.motor.i_no_load:
            # Static stalled/unpowered condition
            i_motor = max(0.0, v_eff / r_total) if r_total > 0 else 0.0
            i_motor = min(i_motor, self.motor.i_no_load)  # Clip to no-load
            motor_rpm = 0.0
        else:
            # Solve quadratic formula using the negative root (physically stable branch)
            discriminant = (b_coeff ** 2) - 4.0 * a_coeff * c_coeff
            if discriminant < 0:
                # Fallback to stalled if math yields imaginary roots under extreme sag
                i_motor = v_eff / r_total
                motor_rpm = 0.0
            else:
                i_motor = (-b_coeff - np.sqrt(discriminant)) / (2.0 * a_coeff)
                # Compute resulting RPM
                motor_rpm = self.motor.kv * (v_eff - i_motor * r_total)
                motor_rpm = max(0.0, motor_rpm)

        # 4. Calculate resultant forces, powers, and currents
        # Rotational speed in revs per second
        rps = motor_rpm / 60.0

        # Thrust: T = Ct * rho * n^2 * D^4 (N)
        thrust_n = self.propeller.ct * self.rho * (rps ** 2) * (self.prop_diameter_m ** 4)
        g_gravity = 9.80665
        thrust_g = (thrust_n / g_gravity) * 1000.0

        # Total battery current based on the ESC model
        if self.use_buck_model:
            current_total_a = self.num_rotors * d * i_motor
        else:
            current_total_a = self.num_rotors * i_motor

        # Battery terminal voltage under load (sagged voltage)
        v_battery_terminal = max(0.0, v_battery_ocv - current_total_a * r_battery)
        
        # Effective average voltage seen by the motor windings
        v_motor = max(0.0, d * v_battery_terminal - i_motor * self.esc.r_esc)

        # Mechanical shaft power (W): P_mech = Cp * rho * n^3 * D^5
        mechanical_power_w = self.propeller.cp * self.rho * (rps ** 3) * (self.prop_diameter_m ** 5)

        # Electrical power input to the ESC/Motor path
        electrical_power_w = v_motor * i_motor

        # Copper loss in windings (W)
        copper_loss_w = (i_motor ** 2) * self.motor.r_motor

        # Iron and friction losses (no-load loss)
        # Estimated using back-EMF voltage * no-load current
        v_back_emf = max(0.0, v_motor - i_motor * self.motor.r_motor)
        iron_loss_w = v_back_emf * self.motor.i_no_load

        # ESC loss (W)
        esc_loss_w = (i_motor ** 2) * self.esc.r_esc

        # Battery loss shared per motor path (W)
        battery_loss_w = ((current_total_a ** 2) * r_battery) / self.num_rotors

        # Combined heat loss per motor-ESC pathway
        power_loss_w = copper_loss_w + iron_loss_w + esc_loss_w + battery_loss_w

        # Propulsive system efficiency (grams of thrust per watt of input electrical power)
        p_in_per_motor = (v_battery_terminal * current_total_a) / self.num_rotors
        efficiency_g_w = thrust_g / p_in_per_motor if p_in_per_motor > 0 else 0.0

        # 5. Thermal Dynamics Model
        # Specific Heat Capacity (J/K): C_th = mass * specific_heat
        c_thermal = self.motor.weight_g * self.motor.specific_heat
        # Thermal resistance (K/W)
        r_th = self.motor.r_thermal_static

        # Copper loss acts as the heat source in the windings
        heat_power_w = copper_loss_w

        # Update the continuous/transient thermal state
        step_time = dt if dt is not None else execution_time
        if step_time > 0:
            # Analytical update equation for 1st order linear system:
            # Delta_T_new = Delta_T_old * exp(-t/tau) + P_loss * R_th * (1 - exp(-t/tau))
            exp_term = np.exp(-step_time / self.motor.thermal_time_constant)
            self.motor_delta_t = self.motor_delta_t * exp_term + (heat_power_w * r_th) * (1.0 - exp_term)

        return {
            "thrust_g": float(thrust_g),
            "thrust_n": float(thrust_n),
            "current_motor_a": float(i_motor),
            "current_total_a": float(current_total_a),
            "voltage_battery_terminal_v": float(v_battery_terminal),
            "voltage_motor_v": float(v_motor),
            "motor_rpm": float(motor_rpm),
            "mechanical_power_w": float(mechanical_power_w),
            "electrical_power_w": float(electrical_power_w),
            "power_loss_w": float(power_loss_w),
            "efficiency_g_w": float(efficiency_g_w),
            "motor_temp_rise": float(self.motor_delta_t)
        }

    def reset_thermal(self) -> None:
        """Resets the motor temperature rise back to ambient (0 K rise)."""
        self.motor_delta_t = 0.0


# A quick diagnostic execution to verify basic performance when run directly
if __name__ == "__main__":
    # Initialize the default 7-inch setup
    sys = PropulsionSystem()
    print("=== 7-inch Quadcopter Propulsion Engine Diagnostic ===")
    print(f"Propeller Diameter: {sys.prop_diameter_m:.4f} m")
    
    # Evaluate at 100% Throttle, 100% SOC
    res_100 = sys.evaluate(throttle=1.0, current_soc=1.0, execution_time=10.0)
    print("\n--- 100% Throttle, Fully Charged ---")
    print(f"Motor RPM:       {res_100['motor_rpm']:.1f}")
    print(f"Thrust:          {res_100['thrust_g']:.1f} g")
    print(f"Motor Current:   {res_100['current_motor_a']:.2f} A")
    print(f"Total Current:   {res_100['current_total_a']:.2f} A")
    print(f"Battery Voltage: {res_100['voltage_battery_terminal_v']:.2f} V")
    print(f"System Loss:     {res_100['power_loss_w']:.1f} W")
    print(f"Efficiency:      {res_100['efficiency_g_w']:.2f} g/W")
    print(f"Temp Rise (10s): {res_100['motor_temp_rise']:.2f} K")

    # Evaluate at 50% Throttle, 50% SOC
    res_50 = sys.evaluate(throttle=0.5, current_soc=0.5, execution_time=10.0)
    print("\n--- 50% Throttle, 50% SOC (Cruise) ---")
    print(f"Motor RPM:       {res_50['motor_rpm']:.1f}")
    print(f"Thrust:          {res_50['thrust_g']:.1f} g")
    print(f"Motor Current:   {res_50['current_motor_a']:.2f} A")
    print(f"Total Current:   {res_50['current_total_a']:.2f} A")
    print(f"Battery Voltage: {res_50['voltage_battery_terminal_v']:.2f} V")
    print(f"Efficiency:      {res_50['efficiency_g_w']:.2f} g/W")
    print(f"Temp Rise (20s): {res_50['motor_temp_rise']:.2f} K")
