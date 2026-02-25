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

# ── QLabs library path ────────────────────────────────────────
# The QLabs helper files (crosswalk.py, basic_shape.py, etc.)
# live in a separate folder from this project.
# Update QLABS_LIB_PATH if your installation is in a different location.
QLABS_LIB_PATH = r"E:\uparking"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)
# ─────────────────────────────────────────────────────────────

from crosswalk import QLabsCrosswalk
from environment_outdoors import QLabsEnvironmentOutdoors
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

    env.set_time_of_day(20)
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
    spawn_building(qlabs, actor_counter,
                   position=[64.564, 25.459, 8], size=[20, 30, 16],
                   color=(0.85, 0.75, 0.65), rotation_deg=0)          # ← change only this building's rotation here
    actor_counter += 1

    spawn_building(qlabs, actor_counter,
                   position=[43.331, 68.917, 0.075], size=[4, 4, 40],
                   color=(0.8, 0.7, 0.6), rotation_deg=0)             # ← change only this building's rotation here
    actor_counter += 1

    spawn_building(qlabs, actor_counter,
                   position=[64.564, 55.0, 8], size=[20, 50, 16],   # ← unique Y position
                   color=(0.85, 0.75, 0.65), rotation_deg=0)         # ← change only this building's rotation here
    actor_counter += 1
    
    spawn_building(qlabs, actor_counter,
                   position=[49, 89.937, 8], size=[20, 50, 16],
                   color=(0.82, 0.72, 0.62), rotation_deg=90)          # ← change only this building's rotation here
    actor_counter += 1

    # ── 5. Sidewalks ───────────────────────────────────────────
    print("  Sidewalks / grass / trees...", end=" ", flush=True)
    sidewalk_configs = [
    ([54, 25.459, 0.05],     [3, 40, 0.15],  0),   #[],  # no rotation
    ([64.564, 25.459, 0.05], [25, 3, 0.15],  90),   # rotated 90°
    ([50, 40, 0.05],         [3, 40, 0.15],  45), # rotated 45°
    ([35.577, 46.677, 0.15], [3, 70, 0.15],  0),
    ([60, 10, 0.05],         [8, 3,  0.15],  0),
    ([18.606, -10.273, 0.979], [25, 3, 0.15], 0)
]
    for pos, size, rot in sidewalk_configs:
        spawn_sidewalk(qlabs, actor_counter, pos, size, rotation_deg=rot)
        actor_counter += 1
        
    # ── 5. Grass ───────────────────────────────────────────
    grass_configs = [
        ([54.564, 46.601, 0.05] , [60, 90, 0.05]), # position [] , size []
    ]
    for pos, size in grass_configs:
        spawn_grass(qlabs, actor_counter, pos, size)
        actor_counter += 1

    # ── 6. Trees ───────────────────────────────────────────────
    tree_positions = [
        [52, 15, 0], [52, 25, 0], [52, 35, 0], [52, 45, 0],
        [60, 5, 0], [68, 5, 0], [76, 5, 0],
        [51.918, 58.843, 0], [51.667, 66.837, 0],
        [-29.903, 24.388, 0], [-28.102, 53.113, 0], [-29.296, 74.328, 0],
    ]
    for pos in tree_positions:
        spawn_tree(qlabs, actor_counter, pos, size=1.0)
        actor_counter += 2  # trunk + foliage

    # ── 7. Parking lot ─────────────────────────────────────────
    print("done")
    print("  Parking lot...", end=" ", flush=True)

    sections = [
        {"start": [-4, -10, 0.005], "rows": 4, "cols": 39},
    ]
    parking_spots, actor_counter = build_parking_lot(qlabs, actor_counter, sections)

    # ── 8. Crosswalk ───────────────────────────────────────────
    print("done")
    print("  Crosswalk / vehicles / drone...", end=" ", flush=True)
    hCrosswalk = QLabsCrosswalk(qlabs, True)
    hCrosswalk.spawn_id_degrees(
        actorNumber=50,
        location=[51.676, -41.043, 0],
        rotation=[1, 0, 0],
        scale=[3, 2.5, 1],
        configuration=0,
        waitForConfirmation=True
    )
    # ── 9. Vehicles ────────────────────────────────────────────
    vehicle_actors = []

    truck_pos = [-7.528, 56.522]
    spawn_truck(qlabs, actor_counter,
                position=[truck_pos[0], truck_pos[1], 0],
                rotation_deg=90, color=(0.2, 0.3, 0.8))
    vehicle_actors.append((actor_counter, "Truck", truck_pos[0], truck_pos[1]))
    actor_counter += 2

    suv_pos = [-20.512, 46.901]
    spawn_suv(qlabs, actor_counter,
              position=[suv_pos[0], suv_pos[1], 0],
              rotation_deg=90, color=(0.6, 0.1, 0.1))
    vehicle_actors.append((actor_counter, "SUV", suv_pos[0], suv_pos[1]))
    actor_counter += 2

    moto_pos = [-51.587, 57.62]
    spawn_motorcycle(qlabs, actor_counter,
                     position=[moto_pos[0], moto_pos[1], 0],
                     rotation_deg=90, color=(0.1, 0.1, 0.1))
    vehicle_actors.append((actor_counter, "Motorcycle", moto_pos[0], moto_pos[1]))
    actor_counter += 2

    # ── 10. Drone ──────────────────────────────────────────────
    hQDrone = QLabsQDrone2(qlabs, True)
    hQDrone.actorNumber = 0
    hQDrone.spawn_id_degrees(
        actorNumber=0,
        location=initialPosition,
        rotation=initialOrientation,
        scale=[5, 5, 5],
        configuration=0
    )
    QLabsRealTime().start_real_time_model(modelName=rtmodels.QDRONE2, actorNumber=0)

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
    vehicle_positions = get_vehicle_positions(qlabs, vehicle_actors)
    occupancy   = check_parking_spot_occupancy(qlabs, spots,
                      vehicle_positions=vehicle_positions)
    empty_spots = get_empty_parking_spots(spots, occupancy)
    occ_count   = sum(1 for v in occupancy.values() if v)
    print(f"Occupancy check — {occ_count} occupied, {len(spots)-occ_count} available")

    # Spawn QCars (count= controls how many, seed= for reproducible selection)
    raw_qcar_actors, qcar_targets = spawn_random_qcars_async(
        qlabs, spots, count=3, vehicle_actors=vehicle_actors)
    qcar_actor_list = get_qcar_actor_list(raw_qcar_actors)

    # Drone scan
    scanner = DroneScanner(drone, spots, drone_altitude=8.0)
    scanner.run(qlabs, vehicle_actors,
                qcar_actors=qcar_actor_list,
                qcar_target_labels=qcar_targets)
