# ============================================================
# parking_lot.py
# Generates parking spot grid positions and spawns the
# white line markers (spline lines + reference frames).
# Called by main.py during setup().
# ============================================================

import math

from spline_line import QLabsSplineLine
from reference_frame import QLabsReferenceFrame


def generate_parking_grid(start_xyz, rows=3, cols=4, spot_length=5.5, spot_width=2.7,
                           row_spacing=6.0, col_spacing=3.0, orientation_deg=0):
    """
    Compute [x, y, z] centre positions for every spot in a parking grid.

    Args:
        start_xyz      : [x, y, z] origin of the grid
        rows           : number of row bands
        cols           : spots per row
        spot_length    : depth of each spot (metres)
        spot_width     : width of each spot (metres)
        row_spacing    : aisle gap between rows (metres)
        col_spacing    : extra gap between columns (metres, use 0 for flush)
        orientation_deg: grid rotation in degrees

    Returns:
        List of [x, y, z] positions, row-major order
    """
    positions = []
    theta = math.radians(orientation_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    for row in range(rows):
        for col in range(cols):
            local_x = col * (spot_width + col_spacing)
            local_y = row * (spot_length + row_spacing)
            positions.append([
                start_xyz[0] + local_x * cos_t - local_y * sin_t,
                start_xyz[1] + local_x * sin_t + local_y * cos_t,
                start_xyz[2]
            ])

    return positions


def spawn_perpendicular_parking_lines(qlabs, actor_number_left, actor_number_right,
                                      center_xyz, yaw_deg, length_m=5.5, width_m=2.7,
                                      line_w=0.10, color=(0.9, 0.9, 0.9),
                                      configuration=QLabsSplineLine.LINEAR):
    """
    Spawn the left and right boundary lines of a single parking spot.

    Returns:
        List of spawned side dicts (length 0, 1, or 2)
    """
    L = length_m / 2.0
    W = width_m / 2.0
    yaw_rad = math.radians(yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)

    spawned = []

    for side, actor_num, sign in [("left", actor_number_left, -1),
                                   ("right", actor_number_right, 1)]:
        offset_pos = [
            center_xyz[0] + sign * W * cos_yaw,
            center_xyz[1] + sign * W * sin_yaw,
            center_xyz[2]
        ]

        spline = QLabsSplineLine(qlabs, verbose=False)
        ok_spawn = spline.spawn(
            location=offset_pos,
            rotation=[0, 0, yaw_rad],
            scale=[1, 1, 1],
            configuration=configuration,
            waitForConfirmation=True
        )

        if ok_spawn:
            pts = [
                [0, -L, center_xyz[2], line_w],
                [0,  L, center_xyz[2], line_w],
            ]
            ok_pts = spline.set_points(
                list(color), pts,
                alignEndPointTangents=False,
                waitForConfirmation=True
            )
            if ok_pts:
                spawned.append({"id": actor_num, "side": side})

    return spawned


def spawn_parking_row_back_line(qlabs, actor_number, row_positions, yaw_deg,
                                spot_length=5.5, spot_width=2.7, line_w=0.10,
                                color=(0.9, 0.9, 0.9),
                                configuration=QLabsSplineLine.LINEAR):
    """
    Spawn a single horizontal line across the front/back of an entire row.
    """
    if len(row_positions) < 2:
        return False

    yaw_rad = math.radians(yaw_deg)
    front_offset = spot_length / 2.0
    fwd_x = math.sin(yaw_rad)
    fwd_y = -math.cos(yaw_rad)

    first, last = row_positions[0], row_positions[-1]

    def front(spot):
        return [
            spot[0] + fwd_x * front_offset,
            spot[1] + fwd_y * front_offset,
            spot[2]
        ]

    fc_first = front(first)
    fc_last  = front(last)

    line_center = [
        (fc_first[0] + fc_last[0]) / 2,
        (fc_first[1] + fc_last[1]) / 2,
        first[2]
    ]

    line_length = math.hypot(fc_last[0] - fc_first[0],
                              fc_last[1] - fc_first[1]) + spot_width

    line_angle = math.atan2(fc_last[1] - fc_first[1],
                             fc_last[0] - fc_first[0])

    spline = QLabsSplineLine(qlabs, verbose=False)
    ok_spawn = spline.spawn(
        location=line_center,
        rotation=[0, 0, line_angle],
        scale=[1, 1, 1],
        configuration=configuration,
        waitForConfirmation=True
    )

    if ok_spawn:
        half = line_length / 2
        pts = [
            [-half, 0, line_center[2], line_w],
            [ half, 0, line_center[2], line_w],
        ]
        return spline.set_points(
            list(color), pts,
            alignEndPointTangents=False,
            waitForConfirmation=True
        )

    return False


def build_parking_lot(qlabs, actor_counter, sections):
    """
    Build all parking sections: spot lines, reference frames, and row back-lines.

    Args:
        qlabs         : QLabs connection
        actor_counter : starting actor number (int)
        sections      : list of section dicts with keys:
                          start [x,y,z], rows (int), cols (int)

    Returns:
        (parking_spots, actor_counter)
          parking_spots — list of spot dicts
          actor_counter — updated counter after all actors are used
    """
    parking_spots = []

    for section_idx, section in enumerate(sections):
        cols = section["cols"]
        rows = section["rows"]


        spot_positions = generate_parking_grid(
            start_xyz=section["start"],
            rows=rows,
            cols=cols,
            spot_length=5.5,
            spot_width=2.7,
            row_spacing=10,
            col_spacing=0,
            orientation_deg=90
        )

        # Group into rows for back-line generation
        row_groups = [
            spot_positions[r * cols: (r + 1) * cols]
            for r in range(rows)
        ]

        # Spawn spot lines + reference frames
        for i, center_pos in enumerate(spot_positions):
            row_idx = i // cols
            yaw = 90 if row_idx % 2 == 0 else 270   # alternating direction

            spot_lines = spawn_perpendicular_parking_lines(
                qlabs=qlabs,
                actor_number_left=actor_counter,
                actor_number_right=actor_counter + 1,
                center_xyz=center_pos,
                yaw_deg=yaw,
                length_m=5.5,
                width_m=2.7,
                line_w=0.10,
                color=(0.9, 0.9, 0.9),
                configuration=QLabsSplineLine.LINEAR
            )

            if len(spot_lines) == 2:
                ref_frame = QLabsReferenceFrame(qlabs)
                ref_frame.spawn_id(
                    actorNumber=actor_counter,
                    location=center_pos,
                    rotation=[0, 0, math.radians(yaw)],
                    scale=[1, 1, 1],
                    configuration=0,
                    waitForConfirmation=True
                )

                col_idx    = i % cols
                row_letter = chr(ord('A') + row_idx)
                spot_label = f"S{section_idx + 1}-{row_letter}{col_idx}"

                parking_spots.append({
                    "center"      : list(center_pos),
                    "is_occupied" : False,
                    "row"         : row_idx,
                    "col"         : col_idx,
                    "row_label"   : row_letter,
                    "label"       : spot_label,
                    "direction"   : "right" if yaw == 90 else "left",
                    "spot_id"     : len(parking_spots),
                    "frame_actor" : actor_counter,
                    "section"     : section_idx + 1
                })
                actor_counter += 3   # 2 lines + 1 ref frame

        # Row back-lines
        for row_idx, row_positions in enumerate(row_groups):
            row_yaw = 90 if row_idx % 2 == 0 else 270
            spawn_parking_row_back_line(
                qlabs=qlabs,
                actor_number=actor_counter,
                row_positions=row_positions,
                yaw_deg=row_yaw,
                spot_length=5.5,
                spot_width=3.84,
                line_w=0.14,
                color=(0.9, 0.9, 0.9)
            )
            actor_counter += 1



    return parking_spots, actor_counter
