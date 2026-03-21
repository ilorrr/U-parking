# ============================================================
# main.py
# Entry point - orchestrates the full campus parking scene.
# ============================================================

import os
# Fix: Updated path for jairD's machine
os.environ['RTMODELS_DIR'] = r"C:\Users\jairD\Downloads\Quanser_Academic_Resources-dev-windows\Quanser_Academic_Resources-dev-windows\0_libraries\resources\rt_models"
import sys
import time
import math
import random

# Fix: Updated QLabs library path for jairD
QLABS_LIB_PATH = r"C:\Users\jairD\OneDrive\Documents\U-Parking\0_libraries\python"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)

# -- QLabs library imports ----------------------------------------
from qvl.crosswalk import QLabsCrosswalk
from qvl.environment_outdoors import QLabsEnvironmentOutdoors
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime
from qvl.qdrone2 import QLabsQDrone2
import pal.resources.rtmodels as rtmodels

# Local modules
from qlabs_connect  import connect_or_launch_qlabs
from scene_elements import spawn_building, spawn_grass, spawn_sidewalk, spawn_tree
from vehicles       import spawn_truck, spawn_suv, spawn_motorcycle
from parking_lot    import build_parking_lot
from occupancy      import (get_vehicle_positions, check_parking_spot_occupancy,
                             get_empty_parking_spots, print_occupancy_report)
from drone_scanner  import DroneScanner
from qcar_spawner   import spawn_random_qcars_async, get_qcar_actor_list


# ============================================================
# SETUP
# ============================================================

def setup(initialPosition=[1.325, 9.5, 2], initialOrientation=[0, 0, 0]):
    os.system("cls")

    qlabs = connect_or_launch_qlabs()
    env   = QLabsEnvironmentOutdoors(qlabs, True)

    env.set_time_of_day(12)
    env.set_weather_preset(env.LIGHT_RAIN)
    env.set_outdoor_lighting(1)
    print("Environment: Noon, Light rain")

    QLabsRealTime().terminate_all_real_time_models()
    time.sleep(2)
    qlabs.destroy_all_spawned_actors()
    time.sleep(2)
    qlabs.destroy_all_spawned_actors()  # second pass clears stragglers
    time.sleep(2)
    QLabsQDrone2(qlabs).destroy_all_actors_of_class()
    time.sleep(1)

    actor_counter = 100

    # -- 1-4. Buildings -----------------------------------------
    print("  Buildings...", end=" ", flush=True)
    building_color = (0.2, 0.1, 0.05) # Solid Dark Brown
    
    spawn_building(qlabs, actor_counter,
                   position=[64.564, 25.459, 8.1], size=[20, 30, 16],
                   color=building_color, rotation_deg=0)
    actor_counter += 1

    spawn_building(qlabs, actor_counter,
                   position=[43.331, 68.917, 20.1], size=[4, 4, 40],
                   color=building_color, rotation_deg=0)
    actor_counter += 1

    spawn_building(qlabs, actor_counter,
                   position=[64.564, 55.0, 8.1], size=[20, 50, 16],   
                   color=building_color, rotation_deg=0)
    actor_counter += 1
    
    spawn_building(qlabs, actor_counter,
                   position=[49, 89.937, 8.1], size=[20, 50, 16],
                   color=building_color, rotation_deg=90)
    actor_counter += 1

    # -- 5. Sidewalks -------------------------------------------
    print("  Sidewalks / grass / trees...", end=" ", flush=True)
    sidewalk_color = (0.05, 0.05, 0.05)  # Solid Black
    sidewalk_configs = [
        ([54, 25, 0.001],         [3.1, 50, 0.15], 0),  # Main Front (Vertical)
        ([74, 50, 0.001],         [43, 3.1, 0.15], 0),  # Top Path (Horizontal)
        ([28.87, 50.25, 0.001],   [47.5, 3.1, 0.15], 0), # Connected Sidewalk
        ([94, 25, 0.001],         [3.1, 53, 0.15], 0),  # Back Path (Vertical)
        ([74, 0, 0.001],          [43, 3.1, 0.15], 0),  # Bottom Path (Horizontal)
        ([-27.33, 40.98, 0.001],  [8.96, 108.53, 0.15], 0),  # Aisle 1
        ([-62.52, 40.75, 0.001],  [17.46, 109.0, 0.15], 0),  # Central Plaza
        ([-97.77, 40.88, 0.001],  [9.02, 108.25, 0.15], 0), # NEW THIRD PLATFORM
        ([9.328, 4.28, 0.001],    [8, 39.06, 0.15], 0, (1.0, 0.0, 0.0)),   # RED STAGING PLATFORM
        ([60, 10, 0.001],         [8, 3.1,  0.15], 0)
    ]
    for config in sidewalk_configs:
        pos, size, rot = config[0], config[1], config[2]
        color = config[3] if len(config) > 3 else sidewalk_color
        spawn_sidewalk(qlabs, actor_number=actor_counter, position=pos, size=size, color=color, rotation_deg=rot)
        actor_counter += 1
        
    # -- 5. Grass -------------------------------------------
    print("  Grass...", end=" ", flush=True)
    spawn_grass(qlabs, actor_counter, [75, 40, 0.0005] , [140, 140, 0.01])
    actor_counter += 1

    # -- 5.5 Tree Planters (Circular Grass) -----------------------
    y_steps = [6.166, 16.727, 33.172, 49.311, 65.738, 83.648]
    x_platforms = [-26.82, -62.82, -97.77]
    from basic_shape import QLabsBasicShape
    for px in x_platforms:
        for py in y_steps:
            hPlanter = QLabsBasicShape(qlabs)
            hPlanter.spawn_id_degrees(actorNumber=actor_counter, location=[px, py, 0.16], 
                                      rotation=[0,0,0], scale=[2.5, 2.5, 0.05], 
                                      configuration=hPlanter.SHAPE_SPHERE)
            hPlanter.set_material_properties(color=(0.0, 0.8, 0.0), roughness=0.9)
            actor_counter += 1

    # -- 6. Trees -----------------------------------------------
    tree_positions = [
        [52, 15, 0], [52, 25, 0], [52, 35, 0], [52, 45, 0],
        [60, 5, 0], [68, 5, 0], [76, 5, 0],
        [51.918, 58.843, 0], [51.667, 66.837, 0],
    ]
    for px in [-26.9, -63.0, -97.7]:
        for py in y_steps:
            tree_positions.append([px, py, 0])

    for pos in tree_positions:
        spawn_tree(qlabs, actor_counter, pos, size=1.0)
        actor_counter += 2

    # -- 7. Parking lot -----------------------------------------
    sections = [
        {"start": [-4, -11.68, 0.005], "rows": 4, "cols": 40},
        {"start": [-74.403, -11.68, 0.005], "rows": 4, "cols": 40},
    ]
    parking_spots, actor_counter = build_parking_lot(qlabs, actor_counter, sections)

    # -- 8. Crosswalk ------------------------
    hCrosswalk = QLabsCrosswalk(qlabs, True)
    hCrosswalk.spawn_id_degrees(
        actorNumber=50, location=[1.734, 50.18, 0], rotation=[1, 0, 0],
        scale=[2.379, 1.125, 1], configuration=0, waitForConfirmation=True
    )
    
    # -- 9. Vehicles --------------------------------------------
    vehicle_actors = []
    def spawn_perfectly_centered_truck(spot_label, color, name):
        nonlocal actor_counter
        spot = next((s for s in parking_spots if s["label"] == spot_label), None)
        if spot:
            cx, cy = spot["center"][0], spot["center"][1]
            yaw_deg = 90 if spot["row"] % 2 == 0 else 270
            px = cx - 1.6 * math.sin(math.radians(yaw_deg))
            py = cy - 1.6 * math.cos(math.radians(yaw_deg))
            spawn_truck(qlabs, actor_counter, position=[px, py, 0.1], rotation_deg=yaw_deg, color=color)
            vehicle_actors.append((actor_counter, name, cx, cy))
            actor_counter += 2

    spawn_perfectly_centered_truck("S1-A0", (0.05, 0.05, 0.05), "Truck_Black")
    spawn_perfectly_centered_truck("S1-B0", (0.8, 0.8, 0.8), "Truck_Gray")

    # Spawn 25 RANDOM TRUCKS across the entire lot to test occupancy detection
    all_spots_except_static = [s for s in parking_spots if s["label"] not in ["S1-A0", "S1-B0"]]
    num_random_trucks = 25
    if len(all_spots_except_static) >= num_random_trucks:
        chosen_truck_spots = random.sample(all_spots_except_static, num_random_trucks)
        for i, spot in enumerate(chosen_truck_spots):
            t_color = (random.random(), random.random(), random.random())
            spawn_perfectly_centered_truck(spot["label"], t_color, f"RandomTruck_{i}")

    # DISTRIBUTE QCARS: 5 in Section 1, 5 in Section 2 (excluding taken spots)
    taken_labels = set([v[1] for v in vehicle_actors])
    s1_free = [s for s in parking_spots if s["section"] == 1 and s["label"] not in taken_labels]
    s2_free = [s for s in parking_spots if s["section"] == 2 and s["label"] not in taken_labels]
    
    # Selection: Ensure 5 from each section if possible
    chosen_spots = random.sample(s1_free, min(5, len(s1_free))) + \
                   random.sample(s2_free, min(5, len(s2_free)))

    raw_qcar_actors, qcar_targets = spawn_random_qcars_async(
        qlabs, chosen_spots, count=10, vehicle_actors=vehicle_actors)
    qcar_actor_list = get_qcar_actor_list(raw_qcar_actors)

    # -- 10. Drones ----------------------------------------------
    drone_positions = [([-40, 100, 25], 0), ([10, -20, 25], 0), ([-90, -20, 25], 0)]
    drones = []
    for i, (pos, rot) in enumerate(drone_positions):
        hDrone = QLabsQDrone2(qlabs, True)
        hDrone.spawn_id_degrees(actorNumber=i, location=pos, rotation=[0, 0, rot], scale=[5, 5, 5], configuration=0)
        hDrone.possess()
        hDrone.set_velocity_and_request_state(motorsEnabled=False)
        hDrone.set_transform_and_dynamics(pos, [0, 0, math.radians(rot)], enableDynamics=False)
        drones.append(hDrone)

    QLabsRealTime().start_real_time_model(modelName=rtmodels.QDRONE2, actorNumber=0)
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn([0, 0, 30], [0, 0, 0])

    return qlabs, parking_spots, drones[0], vehicle_actors, qcar_actor_list, qcar_targets


if __name__ == "__main__":
    qlabs, spots, drone, vehicle_actors, qcar_actor_list, qcar_targets = setup()
    time.sleep(2)
    vehicle_positions = get_vehicle_positions(qlabs, vehicle_actors)
    occupancy = check_parking_spot_occupancy(qlabs, spots, vehicle_positions=vehicle_positions)
    print(f"Occupancy check -- {sum(1 for v in occupancy.values() if v)} occupied")

    scanner = DroneScanner(drone, spots, drone_altitude=8.0)
    try:
        scanner.run(qlabs, vehicle_actors, qcar_actors=qcar_actor_list, qcar_target_labels=qcar_targets)
    except Exception:
        import traceback
        traceback.print_exc()

    # -- YOLOv11 vision-based scan logic ----------------------
    import asyncio as _asyncio
    from uparking_detection.drone_scanner import DroneScanner as VisionScanner
    if scanner.waypoints:
        vision_waypoints = [(wp["x"], wp["y"], wp["z"]) for wp in scanner.waypoints]
        vision_scanner = VisionScanner(qlabs_drone=drone, ws_server=None, model_path=None, hover_time=0.5)
        _asyncio.run(vision_scanner.full_lot_scan(vision_waypoints))
