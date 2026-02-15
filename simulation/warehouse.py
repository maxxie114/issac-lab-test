"""Warehouse environment setup for Isaac Sim."""

import numpy as np

# Zone definitions (x, y, z) — robots navigate between these
PICKUP_ZONES = [
    np.array([-8.0, -6.0, 0.0]),
    np.array([-8.0, 0.0, 0.0]),
    np.array([-8.0, 6.0, 0.0]),
    np.array([-3.0, -6.0, 0.0]),
    np.array([-3.0, 6.0, 0.0]),
]

DROPOFF_ZONES = [
    np.array([12.0, -7.0, 0.0]),
    np.array([12.0, -2.0, 0.0]),
    np.array([12.0, 3.0, 0.0]),
    np.array([12.0, 7.0, 0.0]),
]


def setup_warehouse(world, use_prebuilt=True):
    """Build the warehouse environment.

    Args:
        world: Isaac Sim World instance
        use_prebuilt: If True, try to load the pre-built warehouse USD.
                      If False or if the asset isn't available, build programmatically.
    """
    from omni.isaac.core.objects import FixedCuboid, VisualCuboid
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.stage import add_reference_to_stage
    import omni.usd
    from pxr import UsdLux

    assets_root = get_assets_root_path()
    stage = omni.usd.get_context().get_stage()

    loaded = False
    if use_prebuilt and assets_root:
        warehouse_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        try:
            add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")
            loaded = True
            print("[Warehouse] Loaded pre-built warehouse from Nucleus")
        except Exception as e:
            print(f"[Warehouse] Could not load pre-built warehouse: {e}")
            print("[Warehouse] Building warehouse programmatically...")

    if not loaded:
        _build_warehouse_procedural(world, stage)

    # Add zone markers (visible pickup/dropoff indicators)
    for i, pos in enumerate(PICKUP_ZONES):
        world.scene.add(
            VisualCuboid(
                prim_path=f"/World/Zones/Pickup_{i}",
                name=f"pickup_zone_{i}",
                position=pos + np.array([0, 0, 0.01]),
                scale=np.array([0.8, 0.8, 0.02]),
                color=np.array([0.2, 0.6, 1.0]),  # Blue
            )
        )

    for i, pos in enumerate(DROPOFF_ZONES):
        world.scene.add(
            VisualCuboid(
                prim_path=f"/World/Zones/Dropoff_{i}",
                name=f"dropoff_zone_{i}",
                position=pos + np.array([0, 0, 0.01]),
                scale=np.array([0.8, 0.8, 0.02]),
                color=np.array([0.2, 1.0, 0.3]),  # Green
            )
        )

    print(f"[Warehouse] {len(PICKUP_ZONES)} pickup zones, {len(DROPOFF_ZONES)} dropoff zones")
    return {"pickup_zones": PICKUP_ZONES, "dropoff_zones": DROPOFF_ZONES}


def _build_warehouse_procedural(world, stage):
    """Build warehouse from scratch with cuboids."""
    from omni.isaac.core.objects import FixedCuboid
    from pxr import UsdLux

    # Ground plane
    world.scene.add_default_ground_plane()

    # Floor
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Environment/Floor",
            name="warehouse_floor",
            position=np.array([0.0, 0.0, -0.01]),
            scale=np.array([30.0, 20.0, 0.02]),
            color=np.array([0.35, 0.35, 0.35]),
        )
    )

    # Walls
    wall_h, wall_t = 3.0, 0.2
    walls = [
        ("north", [0, 10, wall_h / 2], [30, wall_t, wall_h]),
        ("south", [0, -10, wall_h / 2], [30, wall_t, wall_h]),
        ("east", [15, 0, wall_h / 2], [wall_t, 20, wall_h]),
        ("west", [-15, 0, wall_h / 2], [wall_t, 20, wall_h]),
    ]
    for name, pos, scale in walls:
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Environment/Walls/{name}",
                name=f"wall_{name}",
                position=np.array(pos),
                scale=np.array(scale),
                color=np.array([0.8, 0.8, 0.75]),
            )
        )

    # Shelf racks — 3 rows x 4 columns with aisles
    for row in range(3):
        for col in range(4):
            x = -6.0 + col * 5.0
            y = -5.0 + row * 5.0
            world.scene.add(
                FixedCuboid(
                    prim_path=f"/World/Environment/Shelves/shelf_r{row}_c{col}",
                    name=f"shelf_r{row}_c{col}",
                    position=np.array([x, y, 1.0]),
                    scale=np.array([2.0, 0.6, 2.0]),
                    color=np.array([0.55, 0.35, 0.15]),
                )
            )

    # Lighting
    light = UsdLux.DistantLight.Define(stage, "/World/Environment/Light")
    light.CreateIntensityAttr(3000)

    print("[Warehouse] Built procedural warehouse")
