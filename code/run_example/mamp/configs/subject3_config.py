"""Configuration for the new subject-3 pipeline.

Internal arrays always use competition coordinates ``[..., (x, y, z)]`` in
metres and seconds.  Only visualization may reorder them to ``(x, z, y)``.
"""

# Values explicitly stated by the competition problem.
NUM_UAV = 24
UAV_SAFE_DISTANCE = 3.0
OBS_SAFE_DISTANCE = 1.5
V_MAX = 50.0
# The statement uses multiples of g but does not state its numeric value.
# Conventional numerical assumption; change here if the organizer specifies it.
G = 9.81
A_XZ_MAX = 2.0 * G
A_Y_UP_MAX = 2.0 * G
A_Y_DOWN_MAX = 1.0 * G
Y_MAX = 100.0
GOAL_SWITCH_SLANT_RANGE = 3500.0

# Not specified by the official problem statement.  These values are Phase-1
# simulation/evaluation defaults and must remain configurable.
SIMULATION_DT_ASSUMED = 0.05
EVALUATION_DT_ASSUMED = 0.05
# The changed terminal slots are only 5 m apart.  A 3 m arrival ball allows a
# UAV to latch on the side nearest its neighbour and can leave less than the
# required 3 m separation.  One metre keeps capture unambiguous and is also a
# stricter terminal-accuracy criterion.
# Tried 0.9 to buy extra terminal clearance; reverted (see git history /
# PR discussion) -- it broke the previously-working 614-cycle baseline
# (new "no coordinated candidate" failure at cycle 560, 2/24 arrived)
# without fixing the actual observed failure, which happens mid-flight
# (cycle ~112, mission_time ~22s) long before any UAV nears its terminal
# capture radius. This constant is not the lever for that issue.
GOAL_TOLERANCE_ASSUMED = 1.0
RANDOM_SEED_ASSUMED = 0
CURVATURE_SPEED_EPS = 1.0e-8
# The semifinal statement specifies only y <= 100 m.  This engineering
# assumption prevents paths through underground space.
Y_MIN_ASSUMED = 0.0
# Compatibility alias for Phase-1 callers; it is not an official lower bound.
Y_MIN = Y_MIN_ASSUMED

# Phase-2 sparse static Global Guide parameters (engineering choices).
GLOBAL_DX = 20.0
GLOBAL_DY = 5.0
GLOBAL_DZ = 10.0
GLOBAL_BOUNDARY_MARGIN = 100.0
GLOBAL_W_ASTAR = 1.2
GLOBAL_EDGE_SAMPLE_SPACING = 2.5
GLOBAL_ENDPOINT_SEARCH_CELLS = 6
GLOBAL_LOOKAHEAD_DISTANCE = 60.0
# Engineering centerline reserve only.  Physical flight remains bounded by
# Y_MAX=100 m in primitive validation and evaluation.
GLOBAL_GUIDE_VERTICAL_RESERVE = 10.0
ENABLE_TURN_AWARE_PREVIEW = True
TURN_ANGLE_EPS = 0.017453292519943295  # 1 degree, radians
GUIDE_TURN_ACCEL_REF_RATIO = 0.3
TURN_PREVIEW_MIN = 20.0
TURN_PREVIEW_MAX = 300.0

# L1 altitude banding.  The A* cost is pure Euclidean length with no vertical
# weighting, so planning 24 near-identical start/goal pairs independently
# yields 24 near-identical routes: the swarm cruises as one coplanar bundle
# 3.478 m apart against a 3 m hard minimum, and every conflict is pushed down
# to the 0.2 s coordination layer.  Assigning interleaved cruise altitudes
# (uav_index % len) separates neighbouring UAVs -- interleaved rather than
# blocked because it is the index-adjacent UAVs that start 3.478 m apart.
# Spacing must exceed 2 * max(PRIMITIVE_VERTICAL_OFFSETS) = 20 m, otherwise L2
# re-mixes the bands on its own.
#
# DISABLED BY DEFAULT (empty tuple).  Banding demonstrably fixes the problem it
# targets -- the CSV scenario's cruise-phase "no coordinated candidate" moved
# from cycle 112 (22.4 s, 0/24 arrived) out to cycle 562 (112.4 s, 1/24), a 5x
# improvement that confirms the diagnosis -- but every variant tried also broke
# the terminal phase, turning the 614-cycle 24/24 baseline into a failure:
#   (30, 60, 90), descend to goal from x=4000   -> baseline 531, CSV 562
#   (30, 60, 90), rejoin planned route by x=3400 -> baseline 365
#   (30, 50, 70), descend to goal from x=4000   -> baseline 382
# The trade is a cruise-phase failure for a terminal-phase one, so it is left
# off.  Set this to a spaced tuple (>= 20 m apart) to re-enable and experiment.
GUIDE_ALTITUDE_BANDS = ()
# Altitude profile when banding is on: climb to the band by CLIMB_END, hold it
# to CRUISE_END, rejoin the originally planned altitude by DESCENT_END, and
# follow the planned route unchanged past that.  Unwinding the band earlier
# (CRUISE_END 2600 / DESCENT_END 3400, i.e. before the 3500 m goal switch) was
# tried and was worse, not better: baseline 365 vs 531 cycles.  The descend-and-
# rejoin manoeuvre is itself what the single-UAV layer chokes on, so doing it
# sooner only moves the failure earlier.
GUIDE_BAND_CLIMB_END_X = 600.0
GUIDE_BAND_CRUISE_END_X = 4000.0
GUIDE_BAND_DESCENT_END_X = 4900.0

# Phase-3 vectorized quintic primitive parameters.  These are engineering
# defaults, not values stated by the competition problem.
PRIMITIVE_HORIZON = 3.0
PRIMITIVE_DT = 0.05
PRIMITIVE_LATERAL_OFFSETS = (-20.0, -10.0, 0.0, 10.0, 20.0)
PRIMITIVE_VERTICAL_OFFSETS = (-10.0, 0.0, 10.0)
PRIMITIVE_DELTA_SPEED_CANDIDATES = (-10.0, 0.0, 10.0, 20.0)
PRIMITIVE_V_END_MIN = 0.0
PRIMITIVE_HIGH_SPEED_TARGET_RATIO = 0.95
PRIMITIVE_DIRECTION_EPS = 1.0e-10
PRIMITIVE_CONSTRAINT_EPS = 1.0e-8

# Phase-4 fine sampled-horizon validation and static ranking parameters.
# The optional guard is an engineering allowance for motion between samples;
# None selects 0.5 * V_MAX * VALIDATION_DT.  OBS_SAFE_DISTANCE remains the
# official evaluator threshold and is deliberately unchanged.
VALIDATION_DT = 0.01
STATIC_COLLISION_GUARD = None
PRIMITIVE_PROGRESS_SCALE = V_MAX * PRIMITIVE_HORIZON
PRIMITIVE_LATERAL_SCALE = max(abs(value) for value in PRIMITIVE_LATERAL_OFFSETS)
PRIMITIVE_VERTICAL_SCALE = max(abs(value) for value in PRIMITIVE_VERTICAL_OFFSETS)
PRIMITIVE_CURVATURE_REFERENCE = 0.01
PRIMITIVE_PREFERRED_CLEARANCE = 5.0
PRIMITIVE_CLEARANCE_SCALE = 3.0
PRIMITIVE_COST_WEIGHT_PROGRESS = 1.0
PRIMITIVE_COST_WEIGHT_OFFSET = 0.25
PRIMITIVE_COST_WEIGHT_CURVATURE = 0.10
PRIMITIVE_COST_WEIGHT_CLEARANCE = 0.20
# This is only a normalized acceleration-use proxy, not official energy.
PRIMITIVE_COST_WEIGHT_EFFORT_PROXY = 0.0

# Phase-5 dynamic-obstacle soft preference.  The hard guard is calculated
# from the sampled analytic obstacle speeds and box rotation speeds.
DYNAMIC_PREFERRED_CLEARANCE = 5.0
DYNAMIC_CLEARANCE_SCALE = 3.0
DYNAMIC_CLEARANCE_COST_WEIGHT = 0.10

# Phase-5.5 receding-horizon execution defaults (engineering parameters).
EXECUTION_HORIZON = 0.20
EXECUTION_RECORD_DT = 0.01
ROLLING_TEST_END_X = 3000.0
ROLLING_TEST_MAX_TIME = 100.0

# Phase-5.6 terminal handling (engineering policy; not official constraints).
TERMINAL_TRIGGER_DISTANCE = 250.0
TERMINAL_DECEL_REF = 0.4 * A_XZ_MAX
TERMINAL_CAPTURE_SPEED = 10.0
TERMINAL_SPEED_DELTA = 5.0
TERMINAL_GOAL_DIRECTION_DISTANCE = GLOBAL_LOOKAHEAD_DISTANCE
TERMINAL_COST_WEIGHT_GOAL = 2.0
TERMINAL_COST_WEIGHT_VREF = 0.5
# Reward the 0.2 s prefix that is actually executed; endpoint-only attraction
# can be postponed forever by a fixed-horizon receding controller.
TERMINAL_EXECUTED_PREFIX_GOAL_WEIGHT = 5.0
ARRIVAL_BISECTION_ITERATIONS = 40
ENABLE_GOAL_SWITCH = False

# Phase-5.7 goal-management engineering interpretation.  The competition
# wording around the 3500 m trigger and goal noise is not yet definitive.
GOAL_SWITCH_TRIGGER_MODE = 'individual_slant_range'
GOAL_SWITCH_REFERENCE_ORIGIN = (0.0, 0.0, 0.0)
DYNAMIC_GOAL_ENABLED = False
GOAL_SINUSOID_FREQUENCY = 1.0
GOAL_SINUSOID_AMPLITUDE = (0.0, 2.0, 2.0)  # DEMO / ASSUMPTION, metres
GOAL_SINUSOID_PHASE = (0.0, 0.0, 1.5707963267948966)  # DEMO
GOAL_RANDOM_AMPLITUDE = (0.0, 1.0, 1.0)  # DEMO uniform half-width, metres
GOAL_RANDOM_UPDATE_FREQUENCY = 0.5  # DEMO / ASSUMPTION, Hz
GOAL_RANDOM_SEED = 0
GOAL_FILTER_ALPHA = 0.5
GOAL_DEADBAND = 1.0
ARRIVAL_GOAL_MODE = 'observed'
