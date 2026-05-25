import urllib.request
import json

def test_home_page():
    print("Testing GET / ...")
    try:
        response = urllib.request.urlopen("http://127.0.0.1:8000/")
        html = response.read().decode('utf-8')
        print(f"  -> Success: HTML fetched, length: {len(html)}")
        
        has_tab = "UAV Component Architect" in html and "switchTab" in html
        has_inputs = "input-frame-weight" in html and "input-battery-capacity" in html and "input-vtx-power" in html
        
        if has_tab and has_inputs:
            print("  -> Success: UAV Component Architect tab and parameter inputs are correctly present in HTML.")
            return True
        else:
            print("  -> ERROR: Tab or inputs are missing in HTML!")
            return False
    except Exception as e:
        print("  -> ERROR fetching home page:", str(e))
        return False

def test_simulation(name, payload_overrides):
    print(f"\nTesting POST /api/simulate ({name}) ...")
    
    # 22 core parameters default dictionary
    payload = {
        "frame_weight": 184.0,
        "arm_length": 280.0,
        "landing_gear_cd": 0.15,
        "motor_kv": 1300.0,
        "motor_r": 0.021,
        "esc_current": 80.0,
        "esc_r": 0.0015,
        "prop_diameter": 7.0,
        "prop_pitch": 5.0,
        "prop_blades": 3,
        "battery_capacity": 6500.0,
        "battery_cells": 6,
        "battery_r": 0.003,
        "pdb_current": 360.0,
        "bec_voltage": 5.0,
        "fc_refresh": 8.0,
        "imu_delay": 1.5,
        "gps_refresh": 10.0,
        "lidar_weight": 45.0,
        "vision_power": 3.5,
        "tof_range": 15.0,
        "rx_latency": 4.0,
        "telemetry_freq": 915.0,
        "vtx_power": 800.0,
        "camera_weight": 215.0,
        "gimbal_axes": 3,
        "radio_range": 15.0,
        "gcs_latency": 28.0
    }
    
    payload.update(payload_overrides)
    req_data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(
        "http://127.0.0.1:8000/api/simulate",
        data=req_data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    
    try:
        with urllib.request.urlopen(req) as res:
            response = json.loads(res.read().decode('utf-8'))
            status = response.get("status")
            steps = len(response.get("time", []))
            max_g = max(response.get("g_force", [0]))
            final_soc = response.get("soc", [0])[-1]
            max_error = max(response.get("tracking_error", [0]))
            
            print(f"  -> Status: {status}")
            print(f"  -> Steps: {steps}")
            print(f"  -> Max G-Force: {max_g:.2f} G")
            print(f"  -> Final SOC: {final_soc:.2f} %")
            print(f"  -> Max Tracking Error: {max_error:.2f} m")
            
            if status == "success" and steps > 0:
                return {
                    "success": True,
                    "max_g": max_g,
                    "final_soc": final_soc,
                    "max_error": max_error
                }
            else:
                print("  -> ERROR: Simulation response failed.")
                return {"success": False}
    except Exception as e:
        print("  -> ERROR calling simulate:", str(e))
        return {"success": False}

def main():
    print("=== UAV Component Architect Verification Script ===")
    if not test_home_page():
        exit(1)
        
    # Case 1: Defaults
    res_def = test_simulation("Default 1.93kg Config", {})
    if not res_def["success"]:
        print("Default simulation test failed!")
        exit(1)
        
    # Case 2: Extreme Heavy payload
    res_heavy = test_simulation("Extreme Heavy Config (+10kg Payload)", {
        "camera_weight": 5215.0, # Camera lens mass 215g -> 5215g
        "lidar_weight": 5045.0   # LiDAR weight 45g -> 5045g
    })
    if not res_heavy["success"]:
        print("Heavy simulation test failed!")
        exit(1)
        
    # Compare
    print("\n=== Physics Comparison ===")
    print(f"Default: Final SOC = {res_def['final_soc']:.2f}%, Max G = {res_def['max_g']:.2f}G, Max Error = {res_def['max_error']:.2f}m")
    print(f"Heavy:   Final SOC = {res_heavy['final_soc']:.2f}%, Max G = {res_heavy['max_g']:.2f}G, Max Error = {res_heavy['max_error']:.2f}m")
    
    # Assert that heavy config burns more battery or tracking fails
    # A heavier drone will burn more battery in hover phases, so final SOC will be significantly lower!
    if res_heavy['final_soc'] < res_def['final_soc']:
        print("\nSUCCESS: Heavier configuration burns battery faster (dynamic mass loop closed successfully!).")
        print("ALL VERIFICATION CHECKS PASSED!")
    else:
        print("\nERROR: Heavier configuration did not reduce final SOC! Dynamic mass calculation not linked correctly.")
        exit(1)

if __name__ == '__main__':
    main()
