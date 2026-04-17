# ============================================================
# vehicles.py
# Spawns vehicle models (truck, SUV, motorcycle) from
# basic shapes. Each vehicle uses 2 consecutive actor numbers.
# Called by main.py during setup().
# ============================================================

import math

from basic_shape import QLabsBasicShape


def spawn_truck(qlabs, actor_number, position, rotation_deg=0, color=(0.2, 0.3, 0.8)):
    """
    Spawn a truck (5.2m × 2.3m × 2.5m) that fits a 5.5m × 2.7m parking spot.
    Uses actor_number (cab) and actor_number+1 (cargo bed).

    Args:
        position     : [x, y, z] centre
        rotation_deg : 0 / 90 / 180 / 270
        color        : RGB tuple
    """
    rot_rad = math.radians(rotation_deg)

    # Cab
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
        cab.set_material_properties(color=color, roughness=0.3, metallic=False,
                                    waitForConfirmation=True)
        cab.set_enable_dynamics(False, waitForConfirmation=True)

    # Cargo bed (offset behind cab)
    offset = 2.6
    bed = QLabsBasicShape(qlabs)
    bed.actorNumber = actor_number + 1
    bed_pos = [
        position[0] + offset * math.sin(rot_rad),
        position[1] + offset * math.cos(rot_rad),
        position[2] + 0.9
    ]

    ok_bed = bed.spawn_degrees(
        location=bed_pos,
        rotation=[0, 0, rotation_deg],
        scale=[1.7, 3.2, 1.8],
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )
    if ok_bed:
        bed.set_material_properties(color=color, roughness=0.3, metallic=False,
                                    waitForConfirmation=True)
        bed.set_enable_dynamics(False, waitForConfirmation=True)

    return ok_cab and ok_bed


def spawn_suv(qlabs, actor_number, position, rotation_deg=0, color=(0.6, 0.1, 0.1)):
    """
    Spawn an SUV (4.5m × 1.9m × 2.6m) that fits a 5.5m × 2.7m parking spot.
    Uses actor_number (body) and actor_number+1 (roof).
    """
    # Body
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
        body.set_material_properties(color=color, roughness=0.2, metallic=False,
                                     waitForConfirmation=True)
        body.set_enable_dynamics(False, waitForConfirmation=True)

    # Roof / cabin
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
        roof.set_material_properties(color=color, roughness=0.2, metallic=False,
                                     waitForConfirmation=True)
        roof.set_enable_dynamics(False, waitForConfirmation=True)

    return ok_body and ok_roof


def spawn_motorcycle(qlabs, actor_number, position, rotation_deg=0, color=(0.1, 0.1, 0.1)):
    """
    Spawn a motorcycle (2.2m × 0.8m × 1.2m) that fits a 5.5m × 2.7m parking spot.
    Uses actor_number (body) and actor_number+1 (seat).
    """
    # Body / frame
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
        body.set_material_properties(color=color, roughness=0.3, metallic=True,
                                     waitForConfirmation=True)
        body.set_enable_dynamics(False, waitForConfirmation=True)

    # Seat
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
        seat.set_material_properties(color=(0.2, 0.2, 0.2), roughness=0.5, metallic=False,
                                     waitForConfirmation=True)
        seat.set_enable_dynamics(False, waitForConfirmation=True)

    return ok_body and ok_seat
