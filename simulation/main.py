# main.py
# Entry point - orchestrates the full campus parking scene.
import os
import sys
import time
import math

#hard coded paths 
os.environ['RTMODELS_DIR'] = r"C:\Users\Roli\Documents\Quanser_Academic_Resources-dev-windows\0_libraries\resources\rt_models"

QLABS_LIB_PATH = r"C:\Users\user\Desktop\U-parking-main\U-parking"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)

QUANSER_LIB_PATH = r"C:\Users\user\Documents\Quanser\0_libraries\python"
if QUANSER_LIB_PATH not in sys.path:
    sys.path.insert(0, QUANSER_LIB_PATH)

# QLabs library imports 

from crosswalk import QLabsCrosswalk
from environment_outdoors import QLabsEnvironmentOutdoors
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime
from qvl.qdrone2 import QLabsQDrone2 # type: ignore
import pal.resources.rtmodels as rtmodels

# Local modules
from qlabs_connect  import connect_or_launch_qlabs
from scene_elements import spawn_building, spawn_grass, spawn_sidewalk, spawn_tree
from vehicles       import spawn_truck
from parking_lot    import build_parking_lot
from occupancy      import (get_vehicle_positions, check_parking_spot_occupancy,
                             get_empty_parking_spots, print_occupancy_report)
from drone_scanner  import QCarPoller
from qcar_spawner   import spawn_random_qcars_async, get_qcar_actor_list


# SETUP
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

    # Pre-spawn FreeCamera for vision capture 
    print("  Pre-spawning vision camera...", end=" ", flush=True)
    vision_cam = QLabsFreeCamera(qlabs)
    vision_cam.spawn_id_degrees(actorNumber=99,
                        location=[0, 0, 30],
                        rotation=[0, 90, 0])
    time.sleep(3.0)
    vision_cam.set_image_capture_resolution(width=640, height=480)
    time.sleep(3.0)
    status, test_frame = vision_cam.get_image()
    if status and test_frame is not None:
        print(f"OK (frame: {test_frame.shape})")
    else:
        print("WARNING: camera not responding")
        vision_cam = None

    actor_counter = 100

    #  1-4. Buildings 
    print("  Buildings...", end=" ", flush=True)
    building_color = (0.2, 0.1, 0.05) # Solid Dark Brown
    
    spawn_building(qlabs, actor_counter, position=[64.564, 25.459, 8.1], size=[20, 30, 16], color=building_color, rotation_deg=0)
    actor_counter += 1
    spawn_building(qlabs, actor_counter, position=[43.331, 68.917, 20.1], size=[4, 4, 40], color=building_color, rotation_deg=0)
    actor_counter += 1
    spawn_building(qlabs, actor_counter, position=[64.564, 55.0, 8.1], size=[20, 50, 16], color=building_color, rotation_deg=0)
    actor_counter += 1
    spawn_building(qlabs, actor_counter, position=[49, 89.937, 8.1], size=[20, 50, 16], color=building_color, rotation_deg=90)
    actor_counter += 1

    #  Sidewalks 
    print("  Sidewalks / grass / trees...", end=" ", flush=True)
    sidewalk_color = (0.05, 0.05, 0.05)
    sidewalk_configs = [
        ([54, 25, 0.001],         [3.1, 50, 0.15], 0),  
        ([74, 50, 0.001],         [43, 3.1, 0.15], 0),  
        ([28.87, 50.25, 0.001],   [47.5, 3.1, 0.15], 0), 
        ([94, 25, 0.001],         [3.1, 53, 0.15], 0),  
        ([74, 0, 0.001],          [43, 3.1, 0.15], 0),  
        ([-27.33, 40.98, 0.001],  [8.96, 108.53, 0.15], 0),  
        ([-62.52, 40.75, 0.001],  [17.46, 109.0, 0.15], 0),  
        ([9.328, 4.28, 0.001],    [8, 39.06, 0.15], 0, (1.0, 0.0, 0.0)),   
        ([60, 10, 0.001],         [8, 3.1,  0.15], 0)
    ]
    for config in sidewalk_configs:
        pos, size, rot = config[0], config[1], config[2]
        color = config[3] if len(config) > 3 else sidewalk_color
        spawn_sidewalk(qlabs, actor_number=actor_counter, position=pos, size=size, color=color, rotation_deg=rot)
        actor_counter += 1
        
    #  Grass 
    print("  Grass...", end=" ", flush=True)
    spawn_grass(qlabs, actor_counter, [75, 40, 0.0005], [140, 140, 0.01])
    actor_counter += 1

    planter_positions = [
        [-26.82, 6.166, 0.16], [-27.489, 16.727, 0.16], [-26.866, 33.172, 0.16],
        [-26.831, 49.311, 0.16], [-26.938, 65.738, 0.16], [-26.867, 83.648, 0.16],
        [-62.822, 83.689, 0.16], [-62.916, 65.999, 0.16], [-62.907, 49.12, 0.16],
        [-62.854, 33.327, 0.16], [-62.853, 16.843, 0.16], [-63.009, 5.724, 0.16]
    ]
    from basic_shape import QLabsBasicShape
    for pos in planter_positions:
        hPlanter = QLabsBasicShape(qlabs)
        hPlanter.spawn_id_degrees(actorNumber=actor_counter, location=pos, rotation=[0,0,0], scale=[2.5, 2.5, 0.05], configuration=hPlanter.SHAPE_SPHERE)
        hPlanter.set_material_properties(color=(0.0, 0.8, 0.0), roughness=0.9)
        actor_counter += 1

    # trees
    tree_positions = [
        [52, 15, 0], [52, 25, 0], [52, 35, 0], [52, 45, 0],
        [60, 5, 0], [68, 5, 0], [76, 5, 0],
        [51.918, 58.843, 0], [51.667, 66.837, 0],
        [-26.969, 6.172, 0], [-27.444, 16.869, 0], [-26.954, 33.292, 0],
        [-26.939, 49.415, 0], [-26.998, 65.875, 0], [-26.916, 83.789, 0],
        [-63.0, 6.172, 0], [-63.0, 16.869, 0], [-63.0, 33.292, 0],
        [-63.0, 49.415, 0], [-63.0, 65.875, 0], [-63.0, 83.789, 0]
    ]
    for pos in tree_positions:
        spawn_tree(qlabs, actor_counter, pos, size=1.0)
        actor_counter += 2

    # Parking lot
    print("done")
    print("  Parking lot...", end=" ", flush=True)

    sections = [
        {"start": [-4, -11.68, 0.005], "rows": 4, "cols": 40},
        {"start": [-74.403, -11.68, 0.005], "rows": 4, "cols": 40},
    ]
    parking_spots, actor_counter = build_parking_lot(qlabs, actor_counter, sections)

    # Crosswalk / vehicles / drone 
    print("done")
    print("  Vehicles / drone...", end=" ", flush=True)
    hCrosswalk = QLabsCrosswalk(qlabs, True)
    hCrosswalk.spawn_id_degrees(actorNumber=50, location=[1.734, 50.18, 0], rotation=[1, 0, 0], scale=[2.379, 1.125, 1], configuration=0, waitForConfirmation=True)
    
    #Vehicles
    vehicle_actors = []

    def spawn_perfectly_centered_truck(spot_label, color, name):
        nonlocal actor_counter
        spot = next((s for s in parking_spots if s["label"] == spot_label), None)
        if spot:
            cx, cy = spot["center"][0], spot["center"][1]
            yaw_deg = 90 if spot["row"] % 2 == 0 else 270
            yaw_rad = math.radians(yaw_deg)
            px = cx - 1.6 * math.sin(yaw_rad)
            py = cy - 1.6 * math.cos(yaw_rad)
            spawn_truck(qlabs, actor_counter, position=[px, py, 0.1], rotation_deg=yaw_deg, color=color)
            vehicle_actors.append((actor_counter, name, cx, cy))
            actor_counter += 2

    import random
    total_trucks = random.randint(15, 35)
    all_available = list(parking_spots)
    random.shuffle(all_available)
    chosen_truck_spots = all_available[:total_trucks]

    for i, spot in enumerate(chosen_truck_spots):
        t_color = (random.random(), random.random(), random.random())
        spawn_perfectly_centered_truck(spot["label"], t_color, f"Truck_{i}")

    print(f"  {total_trucks} trucks spawned randomly across both sections")

    all_static_vehicles = vehicle_actors
    taken_labels = set(s["label"] for s in chosen_truck_spots)
    available_for_qcars = [s for s in parking_spots if s["label"] not in taken_labels]

    raw_qcar_actors, qcar_targets = spawn_random_qcars_async(qlabs, available_for_qcars, count=10, vehicle_actors=all_static_vehicles)
    qcar_actor_list = get_qcar_actor_list(raw_qcar_actors)

    #  Drones 
    drone_positions = [
        ([-40, 100, 0], 0),  
        ([10, -20, 0], 0),   
        ([-90, -20, 0], 0),  
    ]

    drones = []
    for i, (pos, rot) in enumerate(drone_positions):
        hDrone = QLabsQDrone2(qlabs, True)
        hDrone.spawn_id_degrees(actorNumber=i, location=pos, rotation=[0, 0, rot], scale=[5, 5, 5], configuration=0)
        drones.append(hDrone)

    print("  [RT Model] Skipped — FreeCamera used for image capture")
    hQDrone = drones[0]
    drone_spawn_positions = [pos for pos, _ in drone_positions]

    # -- 11. Camera ---------------------------------------------
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn([0, 0, 30], [0, 0, 0])

    print("done")
    print(f"  Scene ready - {len(parking_spots)} spots, ~{actor_counter} actors\n")

    # FIX: Ensure 9 items are returned, appending vision_cam
    return qlabs, parking_spots, hQDrone, vehicle_actors, qcar_actor_list, qcar_targets, drones, drone_spawn_positions, vision_cam


# ENTRY POINT

if __name__ == "__main__":
    # STEP 1: SETUP QLABS FIRST 
    qlabs, spots, drone, vehicle_actors, qcar_actor_list, qcar_targets, \
        all_drones, drone_spawns, vision_cam = setup()

    
    # STEP 2: LAUNCH LIVE STREAMER
    try:
        from live_streamer import launch_streamer_thread
        stream_queue, stream_updater = launch_streamer_thread()
        print ("\n" + "="*60)
        print("Live Dashboard: http://10.26.0.141:5000/")
        print("="*60 + "\n")   
        
    except ImportError:
        print("[Warning] live_streamer.py not found. Running without live web feed.")
        stream_updater = lambda _: None

    # CAMERA TEST
    time.sleep(1)

    # STEP 3: PRE-SCAN (SAFE FETCH)
    from occupancy import is_spot_occupied
    from drone_scanner import QCarPoller

    if qcar_actor_list:
        temp_poller = QCarPoller(qlabs, qcar_actor_list)
        temp_poller.start()
        time.sleep(1.0) 
        live_positions = temp_poller.get_positions()
        temp_poller.stop() 
        time.sleep(0.5)
    else:
        live_positions = []

    vehicle_positions = get_vehicle_positions(qlabs, vehicle_actors)
    pre_scan_log = []
    
    for spot in spots:
        all_vehicles = [[x, y, name] for x, y, name in vehicle_positions] + \
                       [[vx, vy, "qcar"] for vx, vy in live_positions]
        
        occupied, _, _ = is_spot_occupied(spot, all_vehicles)
        pre_scan_log.append({
            "label": spot.get("label", ""),
            "x": spot["center"][0],
            "y": spot["center"][1],
            "occupied": occupied
        })

    occ_count = sum(1 for e in pre_scan_log if e["occupied"])
    print(f"[Vision] Pre-scan: {occ_count} occupied, {len(spots)-occ_count} available")

    #send the inital occupancy data to the web dashboard before flight
    stream_updater(pre_scan_log)
    
    from occupancy_learning import OccupancyLearner
    import random
    
    print("\n[AI] Consulting historical database for smart routing...")
    learner = OccupancyLearner()

    smart_spots = []
    skipped_count = 0

    for spot in spots:
        prob = learner.get_occupancy_probability(spot["label"])
        
        # EXPLOITATION: Always scan spots with > 10% historical occupancy
        if prob > 0.10:
            smart_spots.append(spot)
        # EXPLORATION: 5% chance to double-check a "Low Risk" spot just in case
        elif random.random() < 0.05:
            smart_spots.append(spot)
        else:
            skipped_count += 1   
        
    # STEP 4: VISION PASS
    
    from multi_drone_scanner import vision_collection_pass

    if vision_cam is None:
        print("[Vision] ERROR: No working camera available")
        raise SystemExit(1)

    vision_collection_pass(
        drone=vision_cam,
        parking_spots= smart_spots,
        scan_log=pre_scan_log,
        altitude=8.0,      
        sample_empty=0.3,
        use_free_camera=True,
        update_callback=stream_updater
    )
    # STEP 5: BACKGROUND POLLER RESTART
    if qcar_actor_list:
        poller = QCarPoller(qlabs, qcar_actor_list)
        poller.start()
        time.sleep(1.0)
    else:
        class _NoOpPoller:
            def get_positions(self): return []
            def stop(self): pass
        poller = _NoOpPoller()

    # STEP 6: PARALLEL DRONE SCAN
    from multi_drone_scanner import parallel_scan, print_parallel_summary
    
    scan_log = parallel_scan(
        drones           = all_drones,
        spawn_positions  = drone_spawns,
        parking_spots    = smart_spots,
        altitude         = 15.0,    
        vehicle_actors   = vehicle_actors,
        poller           = poller,
        target_labels    = qcar_targets,
        qcar_hover       = 3.0,
        update_callback  = stream_updater
    )
    
    #SEND LATEST DATA TO WEB DASHBOARD 
    stream_updater(scan_log)

    print_parallel_summary(scan_log)
    poller.stop()

    #Record scan results and update learning model 
    from occupancy_learning import OccupancyLearner
    import random
    learner = OccupancyLearner()
            
    learner.record_scan(scan_log, scan_metadata={
        "weather"    : "light_rain",
        "drones"     : len(all_drones),
        "total_spots": len(spots)
    })
    learner.generate_report()

    # Push results to Django backend if available
    try:
        from live_scanner import push_to_backend
        push_to_backend(scan_log)
    except Exception as e:
        pass
    
    print("\n[Server] Scan complete. Live dashboard is still running at http://10.26.0.141:5000/")
    print("[Server] Press Ctrl+C in this terminal to shut down.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down server...")