import ast
import math
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon

def parse_raw_path(raw_input):
    # to clean up messy text tags like ```python that the AI might include
    clean_input = (
        raw_input.replace("```python", "")
        .replace("```json", "")
        .replace("```", "")
        .strip()
    )

    # to convert the cleaned string into a real Python list of coordinate numbers, stored as a tuple format
    parsed = ast.literal_eval(clean_input)
    return [tuple(pt) for pt in parsed]

def parse_coord(prompt_text, default_val):
    user_input = input(prompt_text).strip()
    if not user_input:
        return default_val
    # Step 3: Split the typed text by commas and convert each number string into a float
    return tuple(float(val.strip()) for val in user_input.split(","))

def calculate_segment_distance(p1, p2):
    # Check if points have a Z axis (3D space with altitude)
    if len(p1) >= 3 and len(p2) >= 3:
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

#SCENARIO AUDITS
def audit_scenario_1(path, start, goal):
    start_ok = abs(path[0][0] - start[0]) < 0.01 and abs(path[0][1] - start[1]) < 0.01
    goal_ok = abs(path[-1][0] - goal[0]) < 0.01 and abs(path[-1][1] - goal[1]) < 0.01

    # finding the biggest jump distance between any two connected waypoints
    if len(path) < 2:
        max_step = 0.0
    else:
        max_step = max(
            calculate_segment_distance(path[i], path[i + 1])
            for i in range(len(path) - 1)
        )
    # Check if the biggest jump distance stays under the 50m max step limit
    smooth_path = max_step < 50
    is_valid = start_ok and goal_ok and smooth_path

    # Return the diagnostic dictionary back to the main runner
    return {
        "is_valid": is_valid,
        "start_ok": start_ok,
        "goal_ok": goal_ok,
        "smooth_path": smooth_path,
        "max_step": round(max_step, 2),
        "obstacles": [],
    }

def audit_scenario_2(path, start, goal, obstacles):
    flight_line = LineString([(p[0], p[1]) for p in path])
    collisions = [
        idx + 1 for idx, obs in enumerate(obstacles) if flight_line.intersects(obs)
    ]

    start_ok = abs(path[0][0] - start[0]) < 0.01 and abs(path[0][1] - start[1]) < 0.01
    goal_ok = abs(path[-1][0] - goal[0]) < 0.01 and abs(path[-1][1] - goal[1]) < 0.01

    has_collision = len(collisions) > 0
    is_valid = start_ok and goal_ok and (not has_collision)

    return {
        "is_valid": is_valid,
        "start_ok": start_ok,
        "goal_ok": goal_ok,
        "collision": has_collision,
        "collided_obstacles": collisions,
        "obstacles": obstacles,
    }

def audit_scenario_3(path, start, goal, emergency_base, initial_battery, burn_rate, threshold):
    current_bat = initial_battery
    battery_breached = False

    # calculating battery drain for each step in the path
    for i in range(len(path) - 1):
        dist = calculate_segment_distance(path[i], path[i + 1])
        current_bat -= (dist / 10.0) * burn_rate

        if current_bat < threshold:
            battery_breached = True

    final_point = path[-1][:2]
    reached_goal = abs(final_point[0] - goal[0]) < 1.0 and abs(final_point[1] - goal[1]) < 1.0
    reached_emergency = (
        abs(final_point[0] - emergency_base[0]) < 1.0
        and abs(final_point[1] - emergency_base[1]) < 1.0
    )

    if battery_breached and reached_emergency:
        is_valid = True
    elif not battery_breached and reached_goal:
        is_valid = True
    else:
        is_valid = False

    return {
        "is_valid": is_valid,
        "final_battery": round(current_bat, 2),
        "battery_breached": battery_breached,
        "diverted_to_base": reached_emergency,
        "reached_goal": reached_goal,
        "obstacles": [],
    }

def audit_scenario_4(path, start, goal, min_alt, max_alt):
    alt_violations = []

    for idx, p in enumerate(path):
        if len(p) < 3:
            # Mark error if AI forgot to include Z coordinate in a 3D test
            alt_violations.append((idx, "missing altitude (2D point in 3D path)"))
        elif p[2] < min_alt or p[2] > max_alt:
            # Mark error if altitude goes below min or above max
            alt_violations.append((idx, p[2]))

    # calculating battery drain with altitude wind penalties
    current_bat = 100.0
    for i in range(len(path) - 1):
        dist = calculate_segment_distance(path[i], path[i + 1])

        if len(path[i]) >= 3 and len(path[i + 1]) >= 3:
            avg_alt = (path[i][2] + path[i + 1][2]) / 2.0
        else:
            avg_alt = 10.0

        burn_multiplier = 2.0 if avg_alt > 40.0 else 1.0
        current_bat -= (dist / 10.0) * 0.5 * burn_multiplier

    is_valid = len(alt_violations) == 0 and current_bat > 0

    return {
        "is_valid": is_valid,
        "alt_violations": alt_violations,
        "final_battery": round(current_bat, 2),
        "obstacles": [],
    }

def audit_scenario_5(path, start, goal, airbase, obstacles, min_alt, threshold, max_step_limit=50.0):
    unit_mismatch = False
    for i in range(len(path) - 1):
        step = calculate_segment_distance(path[i], path[i + 1])
        if 0 < step < 0.05 or abs(path[i][0]) > 5000:
            unit_mismatch = True

    step_violations = []
    max_observed_step = 0.0
    for i in range(len(path) - 1):
        step_dist = calculate_segment_distance(path[i], path[i + 1])
        max_observed_step = max(max_observed_step, step_dist)
        if step_dist > max_step_limit:
            step_violations.append((i, round(step_dist, 2)))
    step_ok = len(step_violations) == 0

    # obstacle collision check against all no-fly zones
    flight_line = LineString([(p[0], p[1]) for p in path])
    collision = any(flight_line.intersects(obs) for obs in obstacles)

    alt_fail = any(len(p) < 3 or p[2] < min_alt for p in path)

    # Rule 5: Battery tracking & emergency airbase diversion check
    current_bat = 100.0
    for i in range(len(path) - 1):
        dist = calculate_segment_distance(path[i], path[i + 1])
        current_bat -= (dist / 10.0) * 0.8

    final_pt = path[-1]
    diverted_to_airbase = (abs(final_pt[0] - airbase[0]) < 2.0 and abs(final_pt[1] - airbase[1]) < 2.0)
    battery_ok = (current_bat >= threshold) or diverted_to_airbase

    # Overall Pass Verdict: ALL 5 rules must pass simultaneously
    is_valid = (
        (not unit_mismatch)
        and step_ok
        and (not collision)
        and (not alt_fail)
        and battery_ok
    )

    return {
        "is_valid": is_valid,
        "unit_mismatch": unit_mismatch,
        "step_ok": step_ok,
        "step_violations": step_violations,
        "max_observed_step": round(max_observed_step, 2),
        "collision": collision,
        "alt_fail": alt_fail,
        "final_battery": round(current_bat, 2),
        "diverted_to_airbase": diverted_to_airbase,
        "obstacles": obstacles,
    }

#AUDIT PASS/FAIL
def print_audit_summary(scenario_id, results):
    print(f"\n      AUDIT REPORT: SCENARIO {scenario_id}      ")
    # print out each test detail key and value pair
    for key, value in results.items():
        if key != "obstacles":
            print(f"{key.replace('_', ' ').title():<25} : {value}")

    print(f"OVERALL SYSTEM VERDICT    : {'PASS (SAFE)' if results['is_valid'] else 'FAIL (UNSAFE)'}")


# input contraints 
def run_pipeline():
    print(" UAV AI Path Validator: ")
    # Main loop keeps running until user chooses to exit
    while True:
        try:
            scenario = input("\nSelect Scenario to Audit (1-5): ").strip()
            if scenario not in ["1", "2", "3", "4", "5"]:
                print("Invalid scenario selection.")
                continue

            # Ask user if they want to override default numbers with custom values
            use_custom = (
                input("Customize environment inputs for this test? (y/n, default=n): ")
                .strip()
                .lower()
                == "y"
            )

            if scenario == "1":
                start = (
                    parse_coord("Enter Start (x,y) [default 0,0]: ", (0.0, 0.0))
                    if use_custom
                    else (0, 0)
                )
                goal = (
                    parse_coord("Enter Goal (x,y) [default 100,100]: ", (100.0, 100.0))
                    if use_custom
                    else (100, 100)
                )

                raw_path = input("\nPaste AI Path List: ")
                path = parse_raw_path(raw_path)
                results = audit_scenario_1(path, start, goal)

            elif scenario == "2":
                start = (
                    parse_coord("Enter Start (x,y) [default 0,0]: ", (0.0, 0.0))
                    if use_custom
                    else (0, 0)
                )
                goal = (
                    parse_coord("Enter Goal (x,y) [default 100,100]: ", (100.0, 100.0))
                    if use_custom
                    else (100, 100)
                )

                if use_custom:
                    obs1_center = parse_coord("Obstacle 1 Center (x,y): ", (30.0, 30.0))
                    obs1_r = float(
                        input("Obstacle 1 Radius [default 10]: ").strip() or 10.0
                    )
                    obs2_center = parse_coord("Obstacle 2 Center (x,y): ", (70.0, 60.0))
                    obs2_r = float(
                        input("Obstacle 2 Radius [default 20]: ").strip() or 20.0
                    )
                    obstacles = [
                        Point(obs1_center).buffer(obs1_r),
                        Point(obs2_center).buffer(obs2_r),
                    ]
                else:
                    # Default circular and rectangular no-fly zone obstacles
                    obstacles = [
                        Point(30, 30).buffer(10),
                        Point(70, 60).buffer(20),
                        Polygon([(40, 70), (60, 70), (60, 90), (40, 90)]),
                    ]

                raw_path = input("\nPaste AI Path List: ")
                path = parse_raw_path(raw_path)
                results = audit_scenario_2(path, start, goal, obstacles)

            elif scenario == "3":
                start = (
                    parse_coord("Enter Start (x,y) [default 0,0]: ", (0.0, 0.0))
                    if use_custom
                    else (0, 0)
                )
                goal = (
                    parse_coord("Enter Goal (x,y) [default 500,500]: ", (500.0, 500.0))
                    if use_custom
                    else (500, 500)
                )
                emergency_base = (
                    parse_coord(
                        "Enter Emergency Base (x,y) [default 150,200]: ", (150.0, 200.0)
                    )
                    if use_custom
                    else (150, 200)
                )
                initial_bat = (
                    float(input("Initial Battery % [default 100]: ").strip() or 100.0)
                    if use_custom
                    else 100.0
                )
                burn_rate = (
                    float(
                        input("Burn rate % per 10m [default 1.0]: ").strip() or 1.0
                    )
                    if use_custom
                    else 1.0
                )
                threshold = (
                    float(
                        input("Min Battery Threshold % [default 15.0]: ").strip()
                        or 15.0
                    )
                    if use_custom
                    else 15.0
                )

                raw_path = input("\nPaste AI Path List: ")
                path = parse_raw_path(raw_path)
                results = audit_scenario_3(
                    path, start, goal, emergency_base, initial_bat, burn_rate, threshold
                )

            elif scenario == "4":
                start = (
                    parse_coord(
                        "Enter Start (x,y,z) [default 0,0,10]: ", (0.0, 0.0, 10.0)
                    )
                    if use_custom
                    else (0, 0, 10)
                )
                goal = (
                    parse_coord(
                        "Enter Goal (x,y,z) [default 200,200,50]: ",
                        (200.0, 200.0, 50.0),
                    )
                    if use_custom
                    else (200, 200, 50)
                )
                min_alt = (
                    float(input("Min Altitude (m) [default 15.0]: ").strip() or 15.0)
                    if use_custom
                    else 15.0
                )
                max_alt = (
                    float(input("Max Altitude (m) [default 120.0]: ").strip() or 120.0)
                    if use_custom
                    else 120.0
                )

                raw_path = input("\nPaste AI Path List: ")
                path = parse_raw_path(raw_path)
                results = audit_scenario_4(path, start, goal, min_alt, max_alt)

            elif scenario == "5":
                start = (
                    parse_coord(
                        "Enter Start (x,y,z) [default 0,0,20]: ",
                        (0.0, 0.0, 20.0),
                    )
                    if use_custom
                    else (0, 0, 20)
                )
                goal = (
                    parse_coord(
                        "Enter Goal (x,y,z) [default 500,500,30]: ",
                        (500.0, 500.0, 30.0),
                    )
                    if use_custom
                    else (500, 500, 30)
                )
                airbase = (
                    parse_coord(
                        "Enter Airbase (x,y,z) [default 250,100,20]: ",
                        (250.0, 100.0, 20.0),
                    )
                    if use_custom
                    else (250, 100, 20)
                )
                min_alt = (
                    float(
                        input("Min Altitude (m) [default 10.0]: ").strip()
                        or 10.0
                    )
                    if use_custom
                    else 10.0
                )
                threshold = (
                    float(
                        input(
                            "Min Battery Threshold % [default 20.0]: "
                        ).strip()
                        or 20.0
                    )
                    if use_custom
                    else 20.0
                )
                max_step = (
                    float(
                        input("Max Step Distance (m) [default 50.0]: ").strip()
                        or 50.0
                    )
                    if use_custom
                    else 50.0
                )

                if use_custom:
                    obs1_center = parse_coord(
                        "Obstacle 1 Center (x,y) [default 150,150]: ",
                        (150.0, 150.0),
                    )
                    obs1_r = float(
                        input("Obstacle 1 Radius [default 30]: ").strip()
                        or 30.0
                    )
                    obs2_center = parse_coord(
                        "Obstacle 2 Center (x,y) [default 350,350]: ",
                        (350.0, 350.0),
                    )
                    obs2_r = float(
                        input("Obstacle 2 Radius [default 40]: ").strip()
                        or 40.0
                    )
                    obstacles = [
                        Point(obs1_center).buffer(obs1_r),
                        Point(obs2_center).buffer(obs2_r),
                    ]
                else:
                    obstacles = [
                        Point(150, 150).buffer(30),
                        Point(350, 350).buffer(40),
                    ]

                raw_path = input("\nPaste AI Path List: ")
                path = parse_raw_path(raw_path)
                results = audit_scenario_5(
                    path,
                    start,
                    goal,
                    airbase,
                    obstacles,
                    min_alt,
                    threshold,
                    max_step_limit=max_step,
                )

            print_audit_summary(scenario, results)
        except Exception as e:
            print(f"\n[ERROR] Failed to execute validation audit: {e}\n")

        # Ask user if they want to run another test or close script
        again = input("Do you want to test another run? (y/n): ").strip().lower()
        if again != "y":
            print("Exiting validator engine.")
            break

if __name__ == "__main__":
    run_pipeline()
    
