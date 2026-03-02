# ============================================================
# main.py
# Entry point — orchestrates the full campus parking scene.
#
# Module layout:
#   qlabs_connect.py  — connect / launch QLabs
#   scene_elements.py — buildings, sidewalks, trees
#   vehicles.py       — truck, SUV, motorcycle
#   parking_lot.py    — grid generation + line spawning
#   occupancy.py      — proximity detection + reporting
#   main.py           — setup() + __main__ (this file)
# ============================================================

import os
import sys
import time
import math

# ── QLabs library path (Universal Detection) ──────────────────
def setup_quanser_paths():
    possible_bases = [
        os.path.join(os.path.expanduser("~"), "Downloads", "Quanser_Academic_Resources-dev-windows", "Quanser_Academic_Resources-dev-windows"),
        os.path.join(os.path.expanduser("~"), "Documents", "Quanser"),
        r"C:\Users\jairD\Downloads\Quanser_Academic_Resources-dev-windows\Quanser_Academic_Resources-dev-windows"
    ]
    
    for base in possible_bases:
        lib_path = os.path.join(base, "0_libraries", "python")
        if os.path.exists(lib_path):
            if lib_path not in sys.path:
                sys.path.insert(0, lib_path)
            os.environ['QAL_DIR'] = base
            os.environ['RTMODELS_DIR'] = os.path.join(base, "0_libraries", "resources", "rt_models")
            return True
    return False

if not setup_quanser_paths():
    print("WARNING: Quanser libraries not found. Please ensure they are in Downloads or Documents/Quanser")
# ─────────────────────────────────────────────────────────────

# ── QLabs library imports ────────────────────────────────────────

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
    time.sleep(1)

    actor_counter = 100

    # ── 1–4. Buildings ─────────────────────────────────────────
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

    # ── 5. Sidewalks ───────────────────────────────────────────
    print("  Sidewalks / grass / trees...", end=" ", flush=True)
    sidewalk_color = (0.05, 0.05, 0.05)  # Solid Black
    sidewalk_configs = [
        ([54, 25, 0.001],         [3.1, 50, 0.15], 0),  # Main Front (Vertical)
        ([74, 50, 0.001],         [43, 3.1, 0.15], 0),  # Top Path (Horizontal)
        ([28.87, 50.25, 0.001],   [47.5, 3.1, 0.15], 0), # Connected Sidewalk
        ([94, 25, 0.001],         [3.1, 53, 0.15], 0),  # Back Path (Vertical)
        ([74, 0, 0.001],          [43, 3.1, 0.15], 0),  # Bottom Path (Horizontal)
        ([-27.33, 40.98, 0.001],  [8.96, 108.53, 0.15], 0),  # NEW BLACK AISLE PLATFORM (Aisle 1)
        ([-62.52, 40.75, 0.001],  [17.46, 109.0, 0.15], 0),  # EXTENDED TREE PLAZA
        ([9.328, 4.28, 0.001],    [8, 39.06, 0.15], 0, (1.0, 0.0, 0.0)),   # FURTHER EXTENDED RED STAGING PLATFORM
        ([60, 10, 0.001],         [8, 3.1,  0.15], 0)
    ]
    for config in sidewalk_configs:
        pos, size, rot = config[0], config[1], config[2]
        color = config[3] if len(config) > 3 else sidewalk_color
        spawn_sidewalk(qlabs, actor_number=actor_counter, position=pos, size=size, color=color, rotation_deg=rot)
        actor_counter += 1
        
    # ── 5. Grass ───────────────────────────────────────────
    print("  Grass...", end=" ", flush=True)
    grass_configs = [
        ([75, 40, 0.0005] , [140, 140, 0.01]), 
    ]
    for pos, size in grass_configs:
        spawn_grass(qlabs, actor_counter, pos, size)
        actor_counter += 1

    # ── 5.5 Tree Planters (Circular Grass) ───────────────────────
    planter_positions = [
        [-26.82, 6.166, 0.16], [-27.489, 16.727, 0.16], [-26.866, 33.172, 0.16],
        [-26.831, 49.311, 0.16], [-26.938, 65.738, 0.16], [-26.867, 83.648, 0.16],
        [-62.822, 83.689, 0.16], [-62.916, 65.999, 0.16], [-62.907, 49.12, 0.16],
        [-62.854, 33.327, 0.16], [-62.853, 16.843, 0.16], [-63.009, 5.724, 0.16]
    ]
    from basic_shape import QLabsBasicShape
    for pos in planter_positions:
        hPlanter = QLabsBasicShape(qlabs)
        hPlanter.spawn_id_degrees(actorNumber=actor_counter, location=pos, 
                                  rotation=[0,0,0], scale=[2.5, 2.5, 0.05], 
                                  configuration=hPlanter.SHAPE_SPHERE)
        hPlanter.set_material_properties(color=(0.0, 0.8, 0.0), roughness=0.9)
        actor_counter += 1

    # ── 6. Trees ───────────────────────────────────────────────
    tree_positions = [
        [52, 15, 0], [52, 25, 0], [52, 35, 0], [52, 45, 0],
        [60, 5, 0], [68, 5, 0], [76, 5, 0],
        [51.918, 58.843, 0], [51.667, 66.837, 0],
        # TREES ON AISLE 1 (-27 range)
        [-26.969, 6.172, 0], [-27.444, 16.869, 0], [-26.954, 33.292, 0],
        [-26.939, 49.415, 0], [-26.998, 65.875, 0], [-26.916, 83.789, 0],
        # TREES ON CENTRAL PLAZA (-63 range)
        [-63.0, 6.172, 0], [-63.0, 16.869, 0], [-63.0, 33.292, 0],
        [-63.0, 49.415, 0], [-63.0, 65.875, 0], [-63.0, 83.789, 0]
    ]
    for pos in tree_positions:
        spawn_tree(qlabs, actor_counter, pos, size=1.0)
        actor_counter += 2  # trunk + foliage

    # ── 7. Parking lot ─────────────────────────────────────────
    print("done")
    print("  Parking lot...", end=" ", flush=True)

    sections = [
        {"start": [-4, -11.68, 0.005], "rows": 4, "cols": 40},
        {"start": [-74.403, -11.68, 0.005], "rows": 4, "cols": 40},
    ]
    parking_spots, actor_counter = build_parking_lot(qlabs, actor_counter, sections)

    # ── 8. Crosswalk / vehicles / drone ────────────────────────
    print("done")
    print("  Vehicles / drone...", end=" ", flush=True)
    hCrosswalk = QLabsCrosswalk(qlabs, True)
    hCrosswalk.spawn_id_degrees(
        actorNumber=50,
        location=[1.734, 50.18, 0],
        rotation=[1, 0, 0],
        scale=[2.379, 1.125, 1],
        configuration=0,
        waitForConfirmation=True
    )
    
    # ── 9. Vehicles ────────────────────────────────────────────
    vehicle_actors = []

    # Helper for mathematically centered truck spawning
    def spawn_perfectly_centered_truck(spot_label, color, name):
        nonlocal actor_counter
        spot = next((s for s in parking_spots if s["label"] == spot_label), None)
        if spot:
            cx, cy = spot["center"][0], spot["center"][1]
            yaw_deg = 90 if spot["row"] % 2 == 0 else 270
            yaw_rad = math.radians(yaw_deg)
            
            # Use geometric center logic
            px = cx - 1.6 * math.sin(yaw_rad)
            py = cy - 1.6 * math.cos(yaw_rad)
            
            spawn_truck(qlabs, actor_counter, position=[px, py, 0.1], 
                        rotation_deg=yaw_deg, color=color)
            vehicle_actors.append((actor_counter, name, cx, cy))
            actor_counter += 2

    # Spawn Original Trucks (Geometric Center)
    spawn_perfectly_centered_truck("S1-A0", (0.05, 0.05, 0.05), "Truck_Black")
    spawn_perfectly_centered_truck("S1-B0", (0.8, 0.8, 0.8), "Truck_Gray")

    # Spawn 10 Random Additional Trucks in Section 1
    import random
    available_s1 = [s for s in parking_spots if s["section"] == 1 and s["label"] not in ["S1-A0", "S1-B0"]] 
    chosen_s1 = []
    if len(available_s1) >= 10:
        chosen_s1 = random.sample(available_s1, 10)
        for i, spot in enumerate(chosen_s1):
            t_color = (random.random(), random.random(), random.random()) 
            spawn_perfectly_centered_truck(spot["label"], t_color, f"S1ExtraTruck_{i}")

    # Spawn 15 Additional Trucks in Section 2 (New Lot)
    sec2_spots = [s for s in parking_spots if s["section"] == 2]
    chosen_s2 = []
    if len(sec2_spots) >= 15:
        chosen_s2 = random.sample(sec2_spots, 15)
        for i, spot in enumerate(chosen_s2):
            t_color = (random.random(), random.random(), random.random()) 
            spawn_perfectly_centered_truck(spot["label"], t_color, f"S2ExtraTruck_{i}")

    # Combined list of all vehicles
    all_static_vehicles = vehicle_actors

    # Spawn QCars - Explicitly targeting empty spots
    taken_labels = [s["label"] for s in chosen_s1] + [s["label"] for s in chosen_s2] + ["S1-A0", "S1-B0"]
    available_s1 = [s for s in parking_spots if s["section"] == 1 and s["label"] not in taken_labels]
    available_s2 = [s for s in parking_spots if s["section"] == 2 and s["label"] not in taken_labels]
    
    # Select exactly 5 from each section
    final_target_spots = []
    if len(available_s1) >= 5: final_target_spots.extend(random.sample(available_s1, 5))
    if len(available_s2) >= 5: final_target_spots.extend(random.sample(available_s2, 5))

    # Spawn 10 total QCars on Red Platform and drive to assigned sections
    raw_qcar_actors, qcar_targets = spawn_random_qcars_async(
        qlabs, final_target_spots, count=10, vehicle_actors=all_static_vehicles)
    qcar_actor_list = get_qcar_actor_list(raw_qcar_actors)

    # ── 10. Drones ──────────────────────────────────────────────
    drone_positions = [
        ([-40, 100, 25], 0), # Drone 0 (Top Center - Above far aisles)
        ([10, -20, 25], 0),  # Drone 1 (Bottom Left - Above Lot 1)
        ([-90, -20, 25], 0), # Drone 2 (Bottom Right - Above Lot 2)
    ]
    
    drones = []
    for i, (pos, rot) in enumerate(drone_positions):
        hDrone = QLabsQDrone2(qlabs, True)
        hDrone.spawn_id_degrees(
            actorNumber=i,
            location=pos,
            rotation=[0, 0, rot],
            scale=[5, 5, 5],
            configuration=0
        )
        drones.append(hDrone)

    # Use the center drone for reference
    hQDrone = drones[0]

    # ── 11. Camera ─────────────────────────────────────────────
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn([0, 0, 30], [0, 0, 0])

    print("done")
    print(f"  Scene ready — {len(parking_spots)} spots, ~{actor_counter} actors\n")

    return qlabs, parking_spots, hQDrone, vehicle_actors


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    qlabs, spots, drone, vehicle_actors = setup()

    time.sleep(2)
    # Registry and reporting logic only
    vehicle_positions = get_vehicle_positions(qlabs, vehicle_actors)
    occupancy   = check_parking_spot_occupancy(qlabs, spots,
                      vehicle_positions=vehicle_positions)
    empty_spots = get_empty_parking_spots(spots, occupancy)
    occ_count   = sum(1 for v in occupancy.values() if v)
    print(f"Occupancy check — {occ_count} occupied, {len(spots)-occ_count} available")

    # Drone scanning logic disabled for static formation testing
    # scanner = DroneScanner(drone, spots, drone_altitude=8.0)
    # scanner.run(qlabs, vehicle_actors,
    #             qcar_actors=qcar_actor_list,
    #             qcar_target_labels=qcar_targets)
