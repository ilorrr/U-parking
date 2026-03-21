# ============================================================
# scene_elements.py
# Spawns static scene elements: buildings, sidewalks, trees, 
# grass.
#
# Called by main.py during setup().
# ============================================================

from basic_shape import QLabsBasicShape


def spawn_building(qlabs, actor_number, position, size, color=(0.8, 0.7, 0.6), rotation_deg=0, is_glass=False):
    """
    Spawn a rectangular building using a scaled cube.
    """
    building = QLabsBasicShape(qlabs)
    building.actorNumber = actor_number

    ok = building.spawn_degrees(
        location=position,
        rotation=[0, 0, rotation_deg],
        scale=size,
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )

    if ok:
        building.set_material_properties(
            color=color,
            roughness=0.2 if is_glass else 0.7,
            metallic=False,
            waitForConfirmation=True
        )
        building.set_enable_dynamics(False, waitForConfirmation=True)
        return True

    return False


def spawn_sidewalk(qlabs, actor_number, position, size,
                   color=(0.7, 0.7, 0.7), rotation_deg=0):
    """
    Spawn a flat pavement section.

    Args:
        size : [width, depth, height]
    """
    sidewalk = QLabsBasicShape(qlabs)
    sidewalk.actorNumber = actor_number

    ok = sidewalk.spawn_degrees(
        location=position,
        rotation=[0, 0, rotation_deg],
        scale=size,
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )

    if ok:
        sidewalk.set_material_properties(
            color=color,
            roughness=0.8,
            metallic=False,
            waitForConfirmation=True
        )
        sidewalk.set_enable_dynamics(False, waitForConfirmation=True)
        return True

    return False

def spawn_grass(qlabs, actor_number, position, size,
                color=(0.0, 0.8, 0.0), rotation_deg=0):
    """
    Spawn a flat grass section.
    """
    grass = QLabsBasicShape(qlabs)
    grass.actorNumber = actor_number

    ok = grass.spawn_degrees(
        location=position,
        rotation=[0, 0, rotation_deg],
        scale=size,
        configuration=QLabsBasicShape.SHAPE_CUBE,
        waitForConfirmation=True
    )

    if ok:
        grass.set_material_properties(
            color=color,
            roughness=0.9,
            metallic=False,
            waitForConfirmation=True
        )
        grass.set_enable_dynamics(False, waitForConfirmation=True)
        return True

    return False


def spawn_tree(qlabs, actor_number, position, size=1.0):
    """
    Spawn a simple tree (cylinder trunk + sphere foliage).
    Uses two consecutive actor numbers: actor_number and actor_number+1.

    Args:
        size : scale multiplier (default 1.0)
    """
    trunk_height = 4.0 * size

    # --- Trunk (brown cylinder) ---
    trunk = QLabsBasicShape(qlabs)
    trunk.actorNumber = actor_number
    trunk_pos = [position[0], position[1], position[2] + trunk_height / 2]

    ok_trunk = trunk.spawn_degrees(
        location=trunk_pos,
        rotation=[0, 0, 0],
        scale=[0.3 * size, 0.3 * size, trunk_height],
        configuration=QLabsBasicShape.SHAPE_CYLINDER,
        waitForConfirmation=True
    )

    if ok_trunk:
        trunk.set_material_properties(
            color=(0.4, 0.3, 0.2),
            roughness=0.9,
            metallic=False,
            waitForConfirmation=True
        )
        trunk.set_enable_dynamics(False, waitForConfirmation=True)

    # --- Foliage (green sphere) ---
    foliage = QLabsBasicShape(qlabs)
    foliage.actorNumber = actor_number + 1
    foliage_pos = [position[0], position[1], position[2] + trunk_height + 1.0 * size]

    ok_foliage = foliage.spawn_degrees(
        location=foliage_pos,
        rotation=[0, 0, 0],
        scale=[2.5 * size, 2.5 * size, 2.5 * size],
        configuration=QLabsBasicShape.SHAPE_SPHERE,
        waitForConfirmation=True
    )

    if ok_foliage:
        foliage.set_material_properties(
            color=(0.2, 0.6, 0.3),
            roughness=0.9,
            metallic=False,
            waitForConfirmation=True
        )
        foliage.set_enable_dynamics(False, waitForConfirmation=True)

    return ok_trunk and ok_foliage
