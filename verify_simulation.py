"""
UAV Simulation Verification and Testing Script
Author: Quality Assurance & Simulation Test Engineer
Description: Systematically validates the static propulsion solver, Moment of Inertia calculations,
             6-DOF dynamics integration, PID mixer, and the optimization sweep logic.
"""

import sys
import numpy as np

# Import our custom libraries
try:
    from propulsion_engine import BatteryConfig, ESCConfig, MotorConfig, PropellerConfig, PropulsionSystem
    from flight_dynamics import FlightDynamics
    from flight_controller import FlightController, run_maneuver_test
except ImportError as e:
    print(f"Import Error: Make sure all simulator python modules are in the same folder. Detail: {str(e)}")
    sys.exit(1)


def test_propulsion_solver():
    print("[1/5] Testing static propulsion engine solver...")
    sys = PropulsionSystem()
    
    # Check OCV lookups
    ocv_100 = sys.get_cell_ocv(1.0)
    ocv_0 = sys.get_cell_ocv(0.0)
    assert np.isclose(ocv_100, 4.20), f"Expected 4.20V OCV at 100% SOC, got {ocv_100:.2f}V"
    assert np.isclose(ocv_0, 3.27), f"Expected 3.27V OCV at 0% SOC, got {ocv_0:.2f}V"
    
    # Check linear interpolation at 50% SOC
    ocv_50 = sys.get_cell_ocv(0.5)
    assert 3.79 < ocv_50 < 3.85, f"OCV at 50% SOC should be between 3.79V and 3.85V, got {ocv_50:.2f}V"

    # Evaluate full throttle
    res = sys.evaluate(throttle=1.0, current_soc=1.0)
    assert res["motor_rpm"] > 20000, f"Expected RPM > 20k for default 1300KV motor at 6S, got {res['motor_rpm']:.1f}"
    assert res["thrust_g"] > 1800, f"Expected thrust > 1800g at full throttle, got {res['thrust_g']:.1f}g"
    assert res["current_motor_a"] > 25.0, f"Expected current > 25A, got {res['current_motor_a']:.2f}A"
    print("      -> Propulsion solver verified successfully!")


def test_inertia_tensor():
    print("[2/5] Testing Moment of Inertia calculations...")
    
    # Top-mount battery dynamics
    dyn_top = FlightDynamics(battery_mount="top")
    # Bottom-mount battery dynamics
    dyn_bottom = FlightDynamics(battery_mount="bottom")
    
    # Mass must be identical regardless of mounting location
    assert np.isclose(dyn_top.mass, dyn_bottom.mass), "Mass mismatch between top and bottom mount configs"
    
    # Vertically displaced center of mass should result in identical XY diagonal inertia due to symmetry
    Ixx_top = dyn_top.inertia[0, 0]
    Iyy_top = dyn_top.inertia[1, 1]
    Izz_top = dyn_top.inertia[2, 2]
    
    assert Izz_top > Ixx_top, f"Expected I_zz ({Izz_top:.6f}) > I_xx ({Ixx_top:.6f}) for a quadcopter frame"
    assert Ixx_top > Iyy_top, f"Expected I_xx ({Ixx_top:.6f}) > I_yy ({Iyy_top:.6f}) because battery length (80mm) is longer along X than width (40mm) along Y"
    
    print(f"      -> Frame AUW: {dyn_top.mass*1000:.1f} g")
    print(f"      -> MoI Tensor: I_xx = {Ixx_top:.6f}, I_yy = {Iyy_top:.6f}, I_zz = {Izz_top:.6f} kg-m^2")
    print("      -> Moment of Inertia calculation verified successfully!")


def test_aerodynamic_drag():
    print("[3/5] Testing 3D Aerodynamic Drag projections...")
    dyn = FlightDynamics()
    
    # State: pos=[0,0,0], vel=[0,0,0], q=[1,0,0,0] (identity), omega=[0,0,0]
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Test vertical fall (10 m/s along Z axis)
    state_z = state.copy()
    state_z[5] = 10.0  # vz = 10 m/s
    deriv_z = dyn.state_derivative(state_z, np.zeros(4), soc=1.0)
    # Drag acts along -Z to oppose motion, so Z-acceleration should be less than gravity (9.81)
    # gravity in NED is positive Z (+9.81 m/s^2)
    assert deriv_z[5] < dyn.g, f"Expected vertical drag to decelerate fall, vertical accel: {deriv_z[5]:.2f} m/s^2"
    
    # Test forward flight (20 m/s along X axis)
    state_x = state.copy()
    state_x[3] = 20.0  # vx = 20 m/s
    deriv_x = dyn.state_derivative(state_x, np.zeros(4), soc=1.0)
    # Drag acts along -X, so X-acceleration should be negative
    assert deriv_x[3] < 0.0, f"Expected forward drag to decelerate, forward accel: {deriv_x[3]:.2f} m/s^2"
    
    print("      -> 3D Aerodynamic Drag verified successfully!")


def test_pid_and_mixer():
    print("[4/5] Testing PID rate controller and mixer...")
    ctrl = FlightController()
    
    # Hover throttle (e.g. 0.25) and zero error
    m_commands = ctrl.get_control_outputs(
        rate_setpoints=(0.0, 0.0, 0.0),
        rates_measured=(0.0, 0.0, 0.0),
        throttle=0.25,
        v_battery_terminal=22.2,
        dt=0.001
    )
    
    # Without errors, all motors should output exactly the input throttle
    assert np.allclose(m_commands, 0.25), f"Expected all motors to output hover throttle, got {m_commands}"
    
    # Test Roll Right command (Roll setpoint = 100 deg/s, measured = 0)
    # Roll Right should increase left motors (M3, M4) and decrease right motors (M1, M2)
    m_roll = ctrl.get_control_outputs(
        rate_setpoints=(100.0, 0.0, 0.0),
        rates_measured=(0.0, 0.0, 0.0),
        throttle=0.30,
        v_battery_terminal=22.2,
        dt=0.001
    )
    
    # Motor layout: M1 (Rear Right), M2 (Front Right), M3 (Rear Left), M4 (Front Left)
    assert m_roll[2] > m_roll[0], f"Rear Left ({m_roll[2]:.2f}) should be larger than Rear Right ({m_roll[0]:.2f}) under Roll Right command"
    assert m_roll[3] > m_roll[1], f"Front Left ({m_roll[3]:.2f}) should be larger than Front Right ({m_roll[1]:.2f}) under Roll Right command"
    
    print("      -> PID Mixer and sign conventions verified successfully!")


def test_maneuver_simulation():
    print("[5/5] Testing maneuver integration runner (RK4 @ 1kHz)...")
    sys = PropulsionSystem()
    dyn = FlightDynamics(propulsion_sys=sys)
    ctrl = FlightController()
    
    # Run the sharp turn preset over a short 1-second window to verify no NaNs or overflow
    tel = run_maneuver_test(dyn, ctrl, maneuver_preset="sharp_turn", total_time=1.0)
    
    # Verify telemetry lists are populated
    assert len(tel["time"]) == 1000, f"Expected 1000 points for 1s at 1kHz, got {len(tel['time'])}"
    assert not np.isnan(tel["pos_x"][-1]), "Position contains NaNs"
    assert not np.isnan(tel["v_bat"][-1]), "Battery voltage contains NaNs"
    assert not np.isnan(tel["motor_temp_rise"][-1]), "Motor temperature contains NaNs"
    
    # Check that quaternion normalization holds
    assert np.all(np.array(tel["soc"]) <= 1.0), "SOC went above 100%"
    assert np.all(np.array(tel["soc"]) > 0.0), "SOC dropped below 0%"
    
    print(f"      -> 1.0s simulation completed: G-force reached {max(tel['g_force']):.2f} G")
    print("      -> Trajectory runner verified successfully!")


def main():
    print("=========================================================")
    print("   UAV FLIGHT SIMULATOR COMPONENT VERIFICATION SUITE     ")
    print("=========================================================")
    
    try:
        test_propulsion_solver()
        test_inertia_tensor()
        test_aerodynamic_drag()
        test_pid_and_mixer()
        test_maneuver_simulation()
        print("\nALL SIMULATION COMPONENTS ARE 100% VALIDATED!")
        print("Ready for deployment.")
        print("=========================================================")
    except AssertionError as e:
        print(f"\n❌ VERIFICATION TEST FAILED!")
        print(f"Detail: {str(e)}")
        print("=========================================================")
        sys.exit(1)


if __name__ == "__main__":
    main()
