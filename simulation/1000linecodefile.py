# ============================================================
# Quanser QLabs - Campus Parking Scene with Buildings
# Enhanced version with realistic parking lot, buildings, and sidewalks
# ============================================================

# region: imports
import os
import sys
import time
import subprocess
import numpy as np
import random
import math


from generic_sensor import QLabsGenericSensor
from crosswalk import QLabsCrosswalk
from environment_outdoors import QLabsEnvironmentOutdoors
from qvl.qlabs import QuanserInteractiveLabs
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime
from qvl.qdrone2 import QLabsQDrone2
from qvl.person import QLabsPerson
from spline_line import QLabsSplineLine
from basic_shape import QLabsBasicShape
from reference_frame import QLabsReferenceFrame  # For marking parking spots


import pal.resources.rtmodels as rtmodels
# endregion


# ------------------------------------------------------------
# Path to QLabs executable
# ------------------------------------------------------------
QLABS_EXE = r"C:\Program Files\Quanser\Quanser Interactive Labs\Quanser Interactive Labs.exe"


# ------------------------------------------------------------
# Connect to existing QLabs OR launch it if needed
# ------------------------------------------------------------
def connect_or_launch_qlabs():
    qlabs = QuanserInteractiveLabs()

    print("Attempting to connect to existing QLabs...")
    if qlabs.open("localhost"):
        print("Connected to existing QLabs instance")
        return qlabs
    print("No running QLabs detected. Launching QLabs with Cityscape...")
    subprocess.Popen([
        QLABS_EXE,
        "-loadmodule",
        "-Plane"
    ])

    # Allow QLabs to fully initialize
    time.sleep(12)

    print("Re-attempting connection...")
    if not qlabs.open("localhost"):
        print("Unable to connect to QLabs after launch")
        sys.exit()

    print("Connected to newly launched QLabs")
    return qlabs


# ============================================================
# BUILDING AND SCENE ELEMENT FUNCTIONS
# ============================================================

def spawn_building(qlabs, actor_number, position, size, color=(0.8, 0.7, 0.6), is_glass=False):
    """
    Spawn a rectangular building using basic shapes.
    
    Args:
        position: [x, y, z] - center position
        size: [width, depth, height]
        color: RGB tuple for building color
        is_glass: If True, makes building semi-transparent like glass
    """
    building = QLabsBasicShape(qlabs)
    building.actorNumber = actor_number  # Set actor number first
    
    # Spawn a cube scaled to building dimensions
    ok = building.spawn_degrees(
        location=position,
        rotation=[0, 0, 0],
        scale=size,
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok:
        # Set material properties
        if is_glass:
            # Glass building - semi-transparent
            building.set_material_properties(
                color=color,
                roughness=0.2,
                metallic=False,  # Must be boolean
                waitForConfirmation=True
            )
        else:
            # Solid building
            building.set_material_properties(
                color=color,
                roughness=0.7,
                metallic=False,  # Must be boolean
                waitForConfirmation=True
            )
        
        # Disable dynamics (static building)
        building.set_enable_dynamics(False, waitForConfirmation=True)
        
        print(f"Building spawned at {position} with size {size}")
        return True
    
    return False


def spawn_sidewalk(qlabs, actor_number, position, size, color=(0.7, 0.7, 0.7)):
    """
    Spawn a sidewalk/pavement section.
    
    Args:
        position: [x, y, z]
        size: [width, depth, height] - height should be small (0.1-0.2)
    """
    sidewalk = QLabsBasicShape(qlabs)
    sidewalk.actorNumber = actor_number  # Set actor number first
    
    ok = sidewalk.spawn_degrees(
        location=position,
        rotation=[0, 0, 0],
        scale=size,
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok:
        sidewalk.set_material_properties(
            color=color,
            roughness=0.8,
            metallic=False,  # Must be boolean
            waitForConfirmation=True
        )
        sidewalk.set_enable_dynamics(False, waitForConfirmation=True)
        return True
    
    return False


def spawn_tree(qlabs, actor_number, position, size=1.0):
    """
    Spawn a simple tree using cylinders and spheres.
    """
    # Tree trunk (brown cylinder)
    trunk = QLabsBasicShape(qlabs)
    trunk.actorNumber = actor_number  # Set actor number first
    trunk_height = 4.0 * size
    trunk_pos = [position[0], position[1], position[2] + trunk_height/2]
    
    ok_trunk = trunk.spawn_degrees(
        location=trunk_pos,
        rotation=[0, 0, 0],
        scale=[0.3 * size, 0.3 * size, trunk_height],
        configuration=QLabsBasicShape.SHAPE_CYLINDER,
        waitForConfirmation=True
    )
    
    if ok_trunk:
        trunk.set_material_properties(
            color=(0.4, 0.3, 0.2),  # Brown
            roughness=0.9,
            metallic=False,  # Must be boolean
            waitForConfirmation=True
        )
        trunk.set_enable_dynamics(False, waitForConfirmation=True)
    
    # Tree foliage (green sphere)
    foliage = QLabsBasicShape(qlabs)
    foliage.actorNumber = actor_number + 1  # Set actor number first
    foliage_pos = [position[0], position[1], position[2] + trunk_height + 1.0wwwww * size]
    
    ok_foliage = foliage.spawn_degrees(
        location=foliage_pos,
        rotation=[0, 0, 0],
        scale=[2.5 * size, 2.5 * size, 2.5 * size],
        configuration=QLabsBasicShape.SHAPE_SPHERE,
        waitForConfirmation=True
    )
    
    if ok_foliage:
        foliage.set_material_properties(
            color=(0.2, 0.6, 0.3),  # Green
            roughness=0.9,
            metallic=False,  # Must be boolean
            waitForConfirmation=True
        )
        foliage.set_enable_dynamics(False, waitForConfirmation=True)
    
    return ok_trunk and ok_foliage

def spawn_truck(qlabs, actor_number, position, rotation_deg=0, color=(0.2, 0.3, 0.8)):
    """
    Spawn a truck that fits in a parking spot (5.5m × 2.7m).
    Truck dimensions: 5.2m long × 2.3m wide × 2.5m tall
    
    Args:
        position: [x, y, z] center position
        rotation_deg: Rotation in degrees (0, 90, 180, 270)
        color: RGB color tuple
    """
    import math
    
    rot_rad = math.radians(rotation_deg)
    
    # Main cab (front part) - 2.3m wide × 2.0m long × 2.5m tall
    cab = QLabsBasicShape(qlabs)
    cab.actorNumber = actor_number
    cab_pos = [position[0], position[1], position[2] + 1.25]
    
    ok_cab = cab.spawn_degrees(
        location=cab_pos,
        rotation=[0, 0, rotation_deg],
        scale=[1.7, 2.0, 2.5],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok_cab:
        cab.set_material_properties(
            color=color,
            roughness=0.3,
            metallic=False,
            waitForConfirmation=True
        )
        cab.set_enable_dynamics(False, waitForConfirmation=True)
    
    # Cargo bed (back part) - 2.3m wide × 3.2m long × 1.8m tall
    # Calculate offset based on rotation
    offset_distance = 2.6
    bed_offset_x = offset_distance * math.sin(rot_rad)
    bed_offset_y = offset_distance * math.cos(rot_rad)
    
    bed = QLabsBasicShape(qlabs)
    bed.actorNumber = actor_number + 1
    bed_pos = [position[0] + bed_offset_x, position[1] + bed_offset_y, position[2] + 0.9]
    
    ok_bed = bed.spawn_degrees(
        location=bed_pos,
        rotation=[0, 0, rotation_deg],
        scale=[1.7, 3.2, 1.8],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok_bed:
        bed.set_material_properties(
            color=color,
            roughness=0.3,
            metallic=False,
            waitForConfirmation=True
        )
        bed.set_enable_dynamics(False, waitForConfirmation=True)
    
    print(f"Truck spawned at {position}, rotation={rotation_deg}° (5.2m × 2.3m - fits in spot)")
    return ok_cab and ok_bed


def spawn_suv(qlabs, actor_number, position, rotation_deg=0, color=(0.6, 0.1, 0.1)):
    """
    Spawn an SUV that fits in a parking spot (5.5m × 2.7m).
    SUV dimensions: 4.5m long × 1.9m wide × 2.6m tall
    
    Args:
        position: [x, y, z] center position
        rotation_deg: Rotation in degrees (0, 90, 180, 270)
        color: RGB color tuple
    """
    # Main body - 1.9m wide × 4.5m long × 1.8m tall
    body = QLabsBasicShape(qlabs)
    body.actorNumber = actor_number
    body_pos = [position[0], position[1], position[2] + 0.9]
    
    ok_body = body.spawn_degrees(
        location=body_pos,
        rotation=[0, 0, rotation_deg],
        scale=[1.5, 4.5, 1.8],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok_body:
        body.set_material_properties(
            color=color,
            roughness=0.2,
            metallic=False,
            waitForConfirmation=True
        )
        body.set_enable_dynamics(False, waitForConfirmation=True)
    
    # Roof/cabin - 1.7m wide × 2.5m long × 0.8m tall
    roof = QLabsBasicShape(qlabs)
    roof.actorNumber = actor_number + 1
    roof_pos = [position[0], position[1], position[2] + 2.2]
    
    ok_roof = roof.spawn_degrees(
        location=roof_pos,
        rotation=[0, 0, rotation_deg],
        scale=[1.6, 2.5, 0.8],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok_roof:
        roof.set_material_properties(
            color=color,
            roughness=0.2,
            metallic=False,
            waitForConfirmation=True
        )
        roof.set_enable_dynamics(False, waitForConfirmation=True)
    
    print(f"SUV spawned at {position}, rotation={rotation_deg}° (4.5m × 1.9m - fits in spot)")
    return ok_body and ok_roof


def spawn_motorcycle(qlabs, actor_number, position, rotation_deg=0, color=(0.1, 0.1, 0.1)):
    """
    Spawn a motorcycle that fits in a parking spot (5.5m × 2.7m).
    Motorcycle dimensions: 2.2m long × 0.8m wide × 1.2m tall
    
    Args:
        rotation_deg: Rotation in degrees (0, 90, 180, 270)
        color: RGB color tuple
    """
    # Main body/frame - 0.8m wide × 2.2m long × 0.8m tall
    body = QLabsBasicShape(qlabs)
    body.actorNumber = actor_number
    body_pos = [position[0], position[1], position[2] + 0.6]
    
    ok_body = body.spawn_degrees(
        location=body_pos,
        rotation=[0, 0, rotation_deg],
        scale=[0.8, 2.2, 0.9],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok_body:
        body.set_material_properties(
            color=color,
            roughness=0.3,
            metallic=True,
            waitForConfirmation=True
        )
        body.set_enable_dynamics(False, waitForConfirmation=True)
    
    # Seat - 0.6m wide × 0.8m long × 0.3m tall
    seat = QLabsBasicShape(qlabs)
    seat.actorNumber = actor_number + 1
    seat_pos = [position[0], position[1], position[2] + 1.05]
    
    ok_seat = seat.spawn_degrees(
        location=seat_pos,
        rotation=[0, 0, rotation_deg],
        scale=[0.6, 0.8, 0.3],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    
    if ok_seat:
        seat.set_material_properties(
            color=(0.2, 0.2, 0.2),
            roughness=0.5,
            metallic=False,
            waitForConfirmation=True
        )
        seat.set_enable_dynamics(False, waitForConfirmation=True)
    
    print(f"Motorcycle spawned at {position}, rotation={rotation_deg}° (2.2m × 0.8m - fits in spot)")
    return ok_body and ok_seat

# ============================================================
# PARKING SPOT DETECTION FUNCTIONS
# ============================================================

def get_vehicle_positions(qlabs, vehicle_actors):
    """Returns spawn-time positions stored in vehicle_actors tuples."""
    positions = []
    for entry in vehicle_actors:
        actor_num, name, x, y = entry
        print(f"  {name} (actor {actor_num}): position = ({x:.3f}, {y:.3f})")
        positions.append([x, y])
    return positions


def check_parking_spot_occupancy(qlabs, parking_spots, vehicle_positions=None):
    """
    Check parking spot occupancy using proximity detection.
    Compares each spot center against known vehicle positions.
    """
    occupancy_status = {}
    occupied_labels = []

    print("Checking occupancy.. ", end="", flush=True)

    SPOT_LENGTH = 5.5
    SPOT_WIDTH  = 2.7
    HALF_X = (SPOT_LENGTH / 2.0) * 0.90   # 2.475m along spot depth
    HALF_Y = (SPOT_WIDTH  / 2.0) * 0.45   # 0.608m along column direction

    for spot in parking_spots:
        spot_label = spot.get("label", str(spot.get("spot_id", -1)))
        cx = spot["center"][0]
        cy = spot["center"][1]
        occupied = False

        if vehicle_positions:
            for vx, vy in vehicle_positions:
                if abs(vx - cx) <= HALF_X and abs(vy - cy) <= HALF_Y:
                    occupied = True
                    break

        occupancy_status[spot_label] = occupied
        if occupied:
            occupied_labels.append(spot_label)

    print("Done!")

    if occupied_labels:
        from collections import defaultdict
        by_row = defaultdict(list)
        for lbl in sorted(occupied_labels):
            by_row[lbl[0]].append(lbl)
        print(f"\nOccupied spots ({len(occupied_labels)} total):")
        for row_letter in sorted(by_row.keys()):
            print(f"  Row {row_letter}: {', '.join(by_row[row_letter])}")
    else:
        print("\nNo occupied spots detected")

    return occupancy_status

def get_empty_parking_spots(parking_spots, occupancy_status):
    """
    Get list of empty parking spots.

    Args:
        parking_spots: List of parking spot dictionaries
        occupancy_status: Dictionary mapping spot label -> bool

    Returns:
        List of empty spot dictionaries
    """
    empty_spots = []
    for spot in parking_spots:
        spot_label = spot.get("label", str(spot.get("spot_id", -1)))
        if spot_label in occupancy_status and not occupancy_status[spot_label]:
            empty_spots.append(spot)
    return empty_spots


# ============================================================
# PARKING SPOT GENERATION (from previous version)
# ============================================================

def generate_parking_grid(start_xyz, rows=3, cols=4, spot_length=5.5, spot_width=2.7, 
                         row_spacing=6.0, col_spacing=3.0, orientation_deg=0):
    """Generate a grid of parking spot center positions."""
    positions = []
    theta = math.radians(orientation_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    for row in range(rows):
        for col in range(cols):
            local_x = col * (spot_width + col_spacing)
            local_y = row * (spot_length + row_spacing)
            
            x = start_xyz[0] + local_x * cos_t - local_y * sin_t
            y = start_xyz[1] + local_x * sin_t + local_y * cos_t
            z = start_xyz[2]
            
            positions.append([x, y, z])
    
    return positions


def spawn_perpendicular_parking_lines(qlabs, actor_number_left, actor_number_right,
                                      center_xyz, yaw_deg, length_m=5.5, width_m=2.7,
                                      line_w=0.10, color=(0.9, 0.9, 0.9),
                                      configuration=QLabsSplineLine.LINEAR):
    """Spawn two vertical parking lines (left and right)."""
    import math
    
    L = length_m / 2.0
    W = width_m / 2.0
    
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    
    # Left line position
    left_offset_x = -W * cos_yaw
    left_offset_y = -W * sin_yaw
    left_pos = [
        center_xyz[0] + left_offset_x,
        center_xyz[1] + left_offset_y,
        center_xyz[2]
    ]
    
    # Right line position
    right_offset_x = W * cos_yaw
    right_offset_y = W * sin_yaw
    right_pos = [
        center_xyz[0] + right_offset_x,
        center_xyz[1] + right_offset_y,
        center_xyz[2]
    ]
    
    spots = []
    
    # Spawn left line
    spline_left = QLabsSplineLine(qlabs, verbose=False)
    ok_spawn_left = spline_left.spawn(
        location=left_pos,
        rotation=[0, 0, yaw_rad],
        scale=[1, 1, 1],
        configuration=configuration,
        waitForConfirmation=True
    )
    
    if ok_spawn_left:
        pts_left = [
            [0, -L, center_xyz[2], line_w],
            [0,  L, center_xyz[2], line_w],
        ]
        ok_pts_left = spline_left.set_points(
            list(color), pts_left, alignEndPointTangents=False, waitForConfirmation=True
        )
        if ok_pts_left:
            spots.append({"id": actor_number_left, "side": "left"})
    
    # Spawn right line
    spline_right = QLabsSplineLine(qlabs, verbose=False)
    ok_spawn_right = spline_right.spawn(
        location=right_pos,
        rotation=[0, 0, yaw_rad],
        scale=[1, 1, 1],
        configuration=configuration,
        waitForConfirmation=True
    )
    
    if ok_spawn_right:
        pts_right = [
            [0, -L, center_xyz[2], line_w],
            [0,  L, center_xyz[2], line_w],
        ]
        ok_pts_right = spline_right.set_points(
            list(color), pts_right, alignEndPointTangents=False, waitForConfirmation=True
        )
        if ok_pts_right:
            spots.append({"id": actor_number_right, "side": "right"})
    
    return spots


def spawn_parking_row_back_line(qlabs, actor_number, row_positions, yaw_deg,
                                spot_length=5.5, spot_width=2.7, line_w=0.10,
                                color=(0.9, 0.9, 0.9), configuration=QLabsSplineLine.LINEAR):
    """Spawn horizontal line at FRONT of parking spots (where cars enter)."""
    import math
    
    if len(row_positions) < 2:
        return False
    
    yaw_rad = math.radians(yaw_deg)
    # CHANGED: Use negative offset to place line at FRONT instead of back
    front_offset = -spot_length / -2.0  # Negative for front
    back_offset = spot_length / 2.0
    
       
    spot_forward_x = math.sin(yaw_rad)
    spot_forward_y = -math.cos(yaw_rad)
    
    first_spot = row_positions[0]
    last_spot = row_positions[-1]
    
    front_center_first = [
        first_spot[0] + spot_forward_x * front_offset,
        first_spot[1] + spot_forward_y * front_offset,
        first_spot[2]
    ]
    
    front_center_last = [
        last_spot[0] + spot_forward_x * front_offset,
        last_spot[1] + spot_forward_y * front_offset,
        last_spot[2]
    ]
    
    line_center = [
        (front_center_first[0] + front_center_last[0]) / 2,
        (front_center_first[1] + front_center_last[1]) / 2,
        first_spot[2]
    ]
    
    line_length = math.sqrt(
        (front_center_last[0] - front_center_first[0])**2 + 
        (front_center_last[1] - front_center_first[1])**2
    )
    line_length += spot_width
    
    line_angle = math.atan2(
        front_center_last[1] - front_center_first[1],
        front_center_last[0] - front_center_first[0]
    )
    
    spline = QLabsSplineLine(qlabs, verbose=False)
    ok_spawn = spline.spawn(
        location=line_center,
        rotation=[0, 0, line_angle],
        scale=[1, 1, 1],
        configuration=configuration,
        waitForConfirmation=True
    )
    
    if ok_spawn:
        half_length = line_length / 2
        pts = [
            [-half_length, 0, line_center[2], line_w],
            [ half_length, 0, line_center[2], line_w],
        ]
        ok_pts = spline.set_points(
            list(color), pts, alignEndPointTangents=False, waitForConfirmation=True
        )
        return ok_pts
    
    return False


# ============================================================
# MAIN SETUP FUNCTION
# ============================================================

def setup(initialPosition=[1.325, 9.5, 2], initialOrientation=[0, 0, 0]):
    os.system("cls")

    # Connect or launch QLabs
    qlabs = connect_or_launch_qlabs()
    env = QLabsEnvironmentOutdoors(qlabs, True)
    
    # Set outdoor environment - bright day for campus scene
    ok1 = env.set_time_of_day(20)  # Noon
    ok2 = env.set_weather_preset(env.LIGHT_RAIN)  # Clear weather
    ok3 = env.set_outdoor_lighting(1)  # Bright lighting (must be integer)
    print("Environment: Noon, Clear weather")
    print(f"set_time_of_day: {ok1}, set_weather_preset: {ok2}, set_outdoor_lighting: {ok3}")

    # Clean previous actors
    QLabsRealTime().terminate_all_real_time_models()
    time.sleep(2)
    qlabs.destroy_all_spawned_actors()
    time.sleep(1)

    actor_counter = 100  # Start actor numbering
    
    # ============================================================
    # BUILD CAMPUS SCENE
    # ============================================================
    
    print("\n" + "="*60)
    print("BUILDING CAMPUS SCENE")
    print("="*60)
    
    # Main building (at specified coordinates)
    print("\n1. Creating main academic building...")
    spawn_building(
        qlabs, actor_counter, 
        position=[64.564, 25.459, 8],  # Your specified X, Y coordinates
        size=[20, 30, 16],  # width, depth, height
        color=(0.85, 0.75, 0.65),  # Tan/beige like the photo
        is_glass=False
    )
    actor_counter += 1
    
    # Glass entrance section (adjacent to main building)
    print("2. Creating glass entrance...")
    spawn_building(
        qlabs, actor_counter,
        position=[79, 25.459, 6],  # Next to main building
        size=[8, 15, 12],
        color=(0.6, 0.7, 0.8),
        is_glass=True
    )
    actor_counter += 1
    
    # Clock tower (near main building)
    print("3. Creating clock tower...")
    spawn_building(
        qlabs, actor_counter,
        position=[35.319, 42.462, -0],
        size=[4, 4, 40],
        color=(0.8, 0.7, 0.6)
    )
    actor_counter += 1
    
    # Secondary building
    print("4. Creating secondary building...")
    spawn_building(
        qlabs, actor_counter,
        position=[64.564, 55, 7],  # Behind main building
        size=[18, 25, 14],
        color=(0.82, 0.72, 0.62)
    )
    actor_counter += 1
    
    # ============================================================
    # CREATE SIDEWALKS (overlapping/under buildings)
    # ============================================================
    
    print("\n5. Creating sidewalks...")
    
    # Main sidewalk along building front (extends under building)
    spawn_sidewalk(qlabs, actor_counter, [54, 25.459, 0.05], [3, 40, 0.15])
    actor_counter += 1
    
    # Sidewalk under/along main building
    spawn_sidewalk(qlabs, actor_counter, [64.564, 25.459, 0.05], [25, 3, 0.15])
    actor_counter += 1
    
    # Perpendicular sidewalk connecting to parking
    spawn_sidewalk(qlabs, actor_counter, [50, 10, 0.05], [3, 30, 0.15])
    actor_counter += 1
    
    # Another connecting sidewalk
    spawn_sidewalk(qlabs, actor_counter, [50, 40, 0.05], [3, 20, 0.15])
    actor_counter += 1
    
    # Wide sidewalk entrance to building
    spawn_sidewalk(qlabs, actor_counter, [60, 10, 0.05], [8, 3, 0.15])
    actor_counter += 1
    
    # ============================================================
    # ADD TREES/LANDSCAPING
    # ============================================================
    
    print("6. Adding landscaping...")
    
    # Trees moved far from parking lot - positioned near buildings
    tree_positions = [
        [52, 15, 0], [52, 25, 0], [52, 35, 0], [52, 45, 0],  # Along sidewalk to building
        [60, 12, 0], [68, 12, 0], [76, 12, 0],  # In front of buildings
        [51.918, 58.843, -0], [51.667, 66.837, 0], # Behind/sides of buildings - away from parking
        [-29.903, 24.388, 0], [-28.102, 53.113, 0], [-29.296, 74.328, -0], #parking lot trees
        
    ]
    
    for i, pos in enumerate(tree_positions):
        spawn_tree(qlabs, actor_counter, pos, size=1.0)
        actor_counter += 2  # Tree uses 2 actors (trunk + foliage)
    
    # ============================================================
    # CREATE PARKING LOT
    # ============================================================
    
    print("\n7. Creating TWO-WAY parking lot...")
    print("="*60)
    
    parking_spots = []
    spot_actor_start = actor_counter
    
    # Create multiple parking sections - adjusted positions
    # Every other row faces opposite direction for two-way traffic
    sections = [
        {"start": [-5, -10, 0.005], "rows": 4, "cols": 39},   # Section 1
        
    ]
    
    for section_idx, section in enumerate(sections):
        print(f"\nSection {section_idx + 1}: {section['rows']}x{section['cols']} spots (TWO-WAY)")
        
        spot_positions = generate_parking_grid(
            start_xyz=section["start"],
            rows=section["rows"],
            cols=section["cols"],
            spot_length=5.5,
            spot_width=2.7,
            row_spacing=10,
            col_spacing=0,
            orientation_deg=90
        )
        
        # Group by rows
        cols = section["cols"]
        rows = section["rows"]
        row_groups = []
        for row in range(rows):
            row_start = row * cols
            row_end = row_start + cols
            row_groups.append(spot_positions[row_start:row_end])
        
        # Create spots - ALTERNATE DIRECTION EVERY OTHER ROW
        for i, center_pos in enumerate(spot_positions):
            # Determine which row (0, 1, 2, 3)
            row_idx = i // cols
            
            # ALTERNATE: Even rows (0,2) face 90°, Odd rows (1,3) face 270° (opposite)
            if row_idx % 2 == 0:
                yaw = 90   # Even rows: face right
            else:
                yaw = 270  # Odd rows: face left (MIRRORED)
            
            color = (0.9, 0.9, 0.9)  # White lines
            
            spot_lines = spawn_perpendicular_parking_lines(
                qlabs=qlabs,
                actor_number_left=actor_counter,
                actor_number_right=actor_counter + 1,
                center_xyz=center_pos,
                yaw_deg=yaw,  # ALTERNATING DIRECTION
                length_m=5.5,
                width_m=2.7,
                line_w=0.10,
                color=color,
                configuration=QLabsSplineLine.LINEAR
            )
            
            if len(spot_lines) == 2:
                # Create spot ID
                spot_id = len(parking_spots)
                
                # Spawn reference frame at parking spot center (for drone detection)
                ref_frame = QLabsReferenceFrame(qlabs)
                ref_frame.spawn_id(
                    actorNumber=actor_counter,
                    location=center_pos,
                    rotation=[0, 0, math.radians(yaw)],  # Match spot orientation
                    scale=[1, 1, 1],
                    configuration=0,  # 0 = invisible, 1 = visible axes
                    waitForConfirmation=True
                )
                
                col_idx = i % cols
                row_letter = chr(ord('A') + row_idx)  # 0→A, 1→B, 2→C, 3→D
                spot_label = f"{row_letter}{col_idx}"  # e.g. "A0", "C19"

                parking_spots.append({
                    "center": list(center_pos),
                    "is_occupied": False,
                    "row": row_idx,
                    "col": col_idx,
                    "row_label": row_letter,
                    "label": spot_label,
                    "direction": "right" if yaw == 90 else "left",
                    "spot_id": spot_id,
                    "frame_actor": actor_counter,  # Store reference frame actor number
                    "section": section_idx + 1
                })
                actor_counter += 3  # 2 for lines + 1 for reference frame
        
        # Add back lines for each row - WITH ALTERNATING DIRECTION
        for row_idx, row_positions in enumerate(row_groups):
            # Even rows: 90°, Odd rows: 270°
            row_yaw = 90 if row_idx % 2 == 0 else 270
            
            spawn_parking_row_back_line(
                qlabs=qlabs,
                actor_number=actor_counter,
                row_positions=row_positions,
                yaw_deg=row_yaw,  # ALTERNATING DIRECTION
                spot_length=5.5,
                spot_width=3.84,
                line_w=0.14,
                color=(0.9, 0.9, 0.9)
            )
            actor_counter += 1
            print(f"  Row {row_idx}: {'→→→ (right)' if row_yaw == 90 else '←←← (left)'}")
    
    print(f"\n{'='*60}")
    print(f"Total parking spots: {len(parking_spots)}")
    print(f"Layout: ALTERNATING rows (Row 0→, Row 1←, Row 2→, Row 3←)")
    print(f"Result: TWO-WAY traffic between each pair of rows")
    print(f"{'='*60}")
    
    # ============================================================
    # SPAWN CROSSWALK (at specified location)
    # ============================================================
    
    print("\n8. Adding crosswalk at specified location...")
    hCrosswalk = QLabsCrosswalk(qlabs, True)
    hCrosswalk.spawn_id_degrees(
        actorNumber=50,
        location=[51.676, -41.043, 0],  # Your specified coordinates
        rotation=[1, 0, 0],
        scale=[3, 2.5, 1],
        configuration=0,
        waitForConfirmation=True
    )
    print(f"Crosswalk spawned at [-28.818, -34.541, 0.005]")
    
    # ============================================================
    # SPAWN DRONE
    # ============================================================
    
    # ============================================================
    # SPAWN VEHICLES
    # ============================================================
    
    print("\n9. Spawning vehicles...")
    print("="*60)
    
    # Store spawn positions — update these when you move a vehicle
    vehicle_actors = []

    truck_pos = [-7.528, 56.522, 0]
    print("Creating truck...")
    spawn_truck(qlabs=qlabs, actor_number=actor_counter,
        position=[truck_pos[0], truck_pos[1], 0], rotation_deg=90, color=(0.2, 0.3, 0.8))
    vehicle_actors.append((actor_counter, "Truck", truck_pos[0], truck_pos[1]))
    actor_counter += 2

    suv_pos = [-20.512, 46.901, 0]
    print("Creating SUV...")
    spawn_suv(qlabs=qlabs, actor_number=actor_counter,
        position=[suv_pos[0], suv_pos[1], 0], rotation_deg=90, color=(0.6, 0.1, 0.1))
    vehicle_actors.append((actor_counter, "SUV", suv_pos[0], suv_pos[1]))
    actor_counter += 2

    moto_pos = [-51.587, 57.62, 0]
    print("Creating motorcycle...")
    spawn_motorcycle(qlabs=qlabs, actor_number=actor_counter,
        position=[moto_pos[0], moto_pos[1], 0], rotation_deg=90, color=(0.1, 0.1, 0.1))
    vehicle_actors.append((actor_counter, "Motorcycle", moto_pos[0], moto_pos[1]))
    actor_counter += 2
    
    print(f"Vehicles spawned: 1 truck (5.2m×2.3m), 1 SUV (4.5m×1.9m), 1 motorcycle (2.2m×0.8m)")
    print(f"All vehicles fit within standard parking spot (5.5m×2.7m)")
    
    # ============================================================
    # SPAWN DRONE
    # ============================================================
    
    print("\n10. Spawning drone...")
    hQDrone = QLabsQDrone2(qlabs, True)
    hQDrone.actorNumber = 0
    hQDrone.spawn_id_degrees(
        actorNumber=0,
        location=initialPosition,
        rotation=initialOrientation,
        scale=[1, 1, 1],
        configuration=0
    )
    
    # Start drone real-time model
    QLabsRealTime().start_real_time_model(
        modelName=rtmodels.QDRONE2,
        actorNumber=0
    )
    
    # ============================================================
    # SPAWN CAMERA
    # ============================================================
    
    print("11. Spawning camera...")
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn([0, 0, 30], [0, 0, 0])
    
    print("\n" + "="*60)
    print("CAMPUS PARKING SCENE COMPLETE!")
    print("="*60)
    print(f"Buildings: 4")
    print(f"Sidewalk sections: 4")
    print(f"Trees: {len(tree_positions)}")
    print(f"Parking spots: {len(parking_spots)}")
    print(f"Vehicles: 3 (1 truck, 1 SUV, 1 motorcycle)")
    print(f"Total actors used: ~{actor_counter}")
    print("="*60 + "\n")
    
    return qlabs, parking_spots, hQDrone, vehicle_actors


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    qlabs, spots, drone, vehicle_actors = setup()
    
    # Example: Check parking spot occupancy
    print("\n" + "="*60)
    print("CHECKING PARKING SPOT OCCUPANCY")
    print("="*60)
    
    # Wait a moment for scene to stabilize
    time.sleep(2)
    
    # Fetch stored vehicle positions and check occupancy
    print("Fetching vehicle positions...")
    vehicle_positions = get_vehicle_positions(qlabs, vehicle_actors)
    occupancy = check_parking_spot_occupancy(qlabs, spots, vehicle_positions=vehicle_positions)
    
    # Get list of empty spots
    empty_spots = get_empty_parking_spots(spots, occupancy)
    
    total    = len(spots)
    occupied = sum(occupancy.values())
    empty    = len(empty_spots)

    print(f"\nTotal spots:    {total}")
    print(f"Occupied spots: {occupied}")
    print(f"Empty spots:    {empty}")

    # Per-row breakdown
    from collections import defaultdict
    row_totals   = defaultdict(int)
    row_occupied = defaultdict(int)
    for spot in spots:
        lbl = spot.get("label", "?")
        row = lbl[0]
        row_totals[row] += 1
        if occupancy.get(lbl, False):
            row_occupied[row] += 1

    print("\nRow-by-row summary:")
    print(f"  {'Row':<6} {'Total':<8} {'Occupied':<10} {'Empty'}")
    for row in sorted(row_totals.keys()):
        t = row_totals[row]
        o = row_occupied[row]
        print(f"  {row:<6} {t:<8} {o:<10} {t - o}")
    
    print("\n" + "="*60 + "\n")
