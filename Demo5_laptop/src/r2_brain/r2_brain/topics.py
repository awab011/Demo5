"""
Topic + blackboard key constants for r2_brain.

Topics here are the contract with AggregatorNode (laptop ↔ H7 bridge) and the
perception nodes. Anything new the BT publishes must be forwarded by
AggregatorNode onto a CAN ID in the 0x110-0x11F laptop→H7 command range.
"""

# ── Subscribed topics ────────────────────────────────────────────────────────
TOPIC_POSE              = '/r2/pose'                    # geometry_msgs/PoseStamped
TOPIC_FOREST_GRID       = '/r2/forest/confirmed_grid'   # std_msgs/String (JSON)
TOPIC_FOREST_PLAN       = '/r2/forest/pickup_plan'      # std_msgs/String (JSON)
TOPIC_ARENA_STATUS      = '/r2/arena/status'            # r2_msgs/ArenaStatus — rack state from fusion
TOPIC_R1_STATUS         = '/r1/status'                  # std_msgs/String (JSON) — R1 pose + zone
TOPIC_H7_STATE          = '/r2/h7_state'                # std_msgs/String (JSON) — TODO: H7 telemetry frame (HANDOFF §6.2)
TOPIC_ARENA_MODE        = '/r2/arena/mode'              # std_msgs/String — "attack" | "defense"
TOPIC_INVENTORY         = '/r2/inventory'               # std_msgs/String (JSON) — KFS we currently hold
TOPIC_ESTOP             = '/r2/estop'                   # std_msgs/Bool
TOPIC_MANUAL_GRID       = '/r2/forest/manual_grid'      # std_msgs/String (JSON) — operator grid from app
TOPIC_MISSION_CMD       = '/r2/mission/cmd'             # std_msgs/String — "start" | "stop" | "retry" (operator)

# ── Published topics (consumed by AggregatorNode → CAN) ──────────────────────
TOPIC_CMD_HIGH_LEVEL    = '/r2/cmd/high_level'          # std_msgs/String (JSON) — BT → AggregatorNode
TOPIC_CMD_PATH          = '/r2/cmd/path'                # nav_msgs/Path or custom — BT → AggregatorNode
TOPIC_R2_PATH           = '/r2_path'                    # r2_navigation/R2Path — forest planner → AggregatorNode (0x100)

# ── Published topics (operator console / diagnostics) ────────────────────────
TOPIC_BRAIN_SNAPSHOT    = '/r2/brain/snapshot'          # std_msgs/String (JSON) — dashboard feed (Step 5: 2 Hz)
TOPIC_INTEGRATION_STATUS = '/r2/debug/integration_status'  # std_msgs/String (JSON) — integration health (1 Hz)

# ── Blackboard keys ──────────────────────────────────────────────────────────
BB_POSE                 = 'pose'
BB_FOREST_GRID          = 'forest_grid'
BB_FOREST_PLAN          = 'forest_plan'
BB_ARENA_RACK           = 'arena_rack'           # dict: slot_num → {kfs_type, confidence, ...}
BB_R1_STATUS            = 'r1_status'            # dict: {x, y, zone, holding}
BB_H7_STATE             = 'h7_state'             # dict: {hsm_mode, sub_state, current_block, faults, hb}
BB_ARENA_MODE           = 'arena_mode'           # str: 'attack' | 'defense'
BB_INVENTORY            = 'inventory'            # list[str]: KFS types we hold
BB_ESTOP                = 'estop'                # bool
BB_LAST_CMD_TS          = 'last_cmd_ts'          # int (ns)
BB_MISSION_ACTIVE       = 'mission_active'       # bool: operator gate for MissionRunning (Selector child)
BB_MANUAL_GRID          = 'manual_grid'          # dict: raw operator grid intent from /r2/forest/manual_grid

# ── Tunables ─────────────────────────────────────────────────────────────────
H7_LINK_TIMEOUT_MS      = 200    # estop if no h7 telemetry for this long
R1_CLEARANCE_MIN_M      = 0.40   # stop motion if R1 closer than this
TICK_HZ                 = 20     # BT tick rate
MISSION_CMD_TIMEOUT_S   = 30.0   # warn (do NOT block) if no /r2/mission/cmd within this of node start
SNAPSHOT_HZ             = 2      # /r2/brain/snapshot publish rate (Step 5)
INTEGRATION_STATUS_HZ   = 1      # /r2/debug/integration_status publish rate


# ── Blackboard convenience ───────────────────────────────────────────────────
def bb_get(client, key, default=None):
    """py_trees Client.get() raises on missing keys — wrap with a default."""
    try:
        return client.get(key)
    except (KeyError, AttributeError):
        return default
