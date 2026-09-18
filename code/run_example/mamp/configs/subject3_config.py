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
# Bands alone are NOT sufficient -- they must be paired with the corridors
# below.  Measured, competition mode, seed 0, as "hardcoded baseline / CSV":
#   no banding                 614 ok 24/24  /  112 FAIL 0/24
#   (30,60,90) bands only      531 FAIL      /  562 FAIL 1/24
#   (30,50,70) bands only      382 FAIL      /  -
#   (30,60,90) + corridors     583 ok 24/24  /  583 ok 24/24
# Bands alone only move the failure from the cruise phase to the terminal
# phase; adding corridors clears both, and the two configurations then land on
# the same result, i.e. the millimetre-level CSV rounding that used to decide
# success no longer changes the outcome.  Empty tuple disables banding.
#
# 4 bands instead of 3: measured real-flight pairwise clearances (not just the
# static-geometry corridor probe below) show the one genuine mid-flight
# congestion spike -- aside from the launch grid and the terminal parking
# formation, both dense by design -- sits at x in [3200, 3600), i.e. right at
# GOAL_SWITCH_SLANT_RANGE=3500 and inside the geometric "waist" the corridor
# comment below describes.  That is 8 same-altitude UAVs converging on the
# switch at once under 3 bands; splitting to 4 halves that to 6.  15/40/65/90
# was chosen over an even split (22.5 apart) to stay clear of both the y=0
# floor and the 90 m ceiling with margin, while keeping every gap above the
# required 25 m; the waist's lateral corridor (see below) was independently
# probed at y=15/40/65/90 and is identical to the old y=30/60/90 probe, so
# nothing here depends on which of the two obstacles-are-taller-than-either
# case actually holds. 6 equal bands (90 m / 5 gaps = 18 m) would violate the
# spacing rule above and was rejected on that basis alone.
GUIDE_ALTITUDE_BANDS = (15.0, 40.0, 65.0, 90.0)
# Lateral corridors, layered on top of the altitude bands. SUPERSEDED by
# lane_assignment.py (see GUIDE_LANE_* below) but kept, unremoved, as the
# fallback path shaped_initial_route() takes when GUIDE_LANE_ASSIGNMENT_ENABLED
# is False -- that reproduces the exact prior known-good baseline (583 cycles,
# score 76.4) rather than a re-derived approximation of it.
#
# Original scheme, still active in the fallback: each UAV's cruise section is
# pushed sideways to goal_z * scale, so corridor order matches goal order and
# the routes never have to cross when they converge at the end. Unlike raising
# a route, shifting it sideways can move it into an obstacle, so each UAV
# tries these scales in order and keeps the first whose every segment clears
# the real obstacle geometry; an empty tuple disables corridors.
# Most UAVs degrade all the way to no corridor: measured at mid-cruise, all 24
# planned routes thread the same lateral gap at z between -8 and -13 m
# regardless of their -40..+40 starts and -100..+130 goals, so there is simply
# no lateral room down low.  Only the 90 m band, where just 4 obstacles still
# reach, has space to spread.  That handful of relocated routes is nonetheless
# what turns the bands-only terminal failure into a completed mission.
#
# Why this was superseded: the real congestion isn't a place, it's the goal
# switch itself -- 24 UAVs abandoning tightly-packed first-leg goals for
# changed goals up to 1100 m apart, all at once. goal_z*scale only spreads
# UAVs relative to their FIRST-LEG goal (+-130 m), which is irrelevant to
# where they're headed after the switch, so it can't touch that. Measured,
# switch-region window x in [3200,3600), same-layer near-misses: <6m 285,
# <4m 61 with this scheme -- unimproved, because the fan-out itself is
# untouched. GUIDE_LANE_* replaces this with an offset computed from each
# UAV's actual post-switch destination.
GUIDE_CORRIDOR_SCALES = (2.5, 2.0, 1.5)

# Lane assignment: supersedes GUIDE_CORRIDOR_SCALES above. Each UAV's
# first-leg cruise offset is computed by lane_assignment.assign_lane_offsets()
# from where it is actually headed AFTER the goal switch (not its first-leg
# goal), so the fan-out at the switch point happens gradually across the
# whole first leg instead of all at once. See lane_assignment.py for the
# measurement trail: two purely-geometric hotspot predictors were tried and
# rejected first (first-leg route density is nearly uniform end to end --
# the congestion is an EVENT, not a PLACE, so nothing in first-leg geometry
# alone predicts it), before landing on this "just fly toward the real
# destination early" approach.
GUIDE_LANE_ASSIGNMENT_ENABLED = True
# Discount ladder for the lane offset computed above -- mirrors the old
# GUIDE_CORRIDOR_SCALES ladder's purpose: a lane centre checked reachable on
# a coarse probe grid can still clip an obstacle on the real, densely-sampled
# route, so each UAV tries the full offset, then discounted fractions of it
# (pulling back toward its own natural, unshaped cruise line), before
# altitude-only and the plain route (both handled by shaped_initial_route()
# itself, not this ladder).
GUIDE_LANE_DISCOUNT_LADDER = (1.0, 0.7, 0.4)
# group_by_effective_goal() splits UAVs into destination groups wherever a
# gap between (sorted) effective goals exceeds max(MEDIAN_GAP * GAP_FACTOR,
# MIN_GAP). This competition's changed-goal data has 3 tight clusters (~5-10 m
# within-group spacing) separated by 440-570 m gaps, so almost any reasonable
# factor/floor separates them correctly; MIN_GAP=50 m is the binding
# constraint here (median gap is only 5 m) and exists so a scenario with no
# real clustering (e.g. plain fixed-mode goals spaced ~10 m apart, uniformly)
# doesn't get spuriously split on floating-point noise.
GUIDE_LANE_GROUP_GAP_FACTOR = 3.0
GUIDE_LANE_GROUP_MIN_GAP = 50.0
# x-step for probing whether a candidate lane is reachable along the cruise
# stretch before committing to it (assign_lane_offsets() checks every
# multiple of this from the climb breakpoint up to min(descent breakpoint,
# GOAL_SWITCH_SLANT_RANGE) -- geometry past the switch point doesn't matter,
# a UAV that switches replans there anyway). 300 m matches the resolution
# used throughout this feature's own measurement trail; halving it did not
# change which candidates passed or failed on this obstacle field.
GUIDE_LANE_PROBE_X_STEP = 300.0
# If a group's own destination is not reachable along the probe grid (an
# obstacle sits in the way at some altitude the group uses), its candidate
# is scaled toward the centreline (z=0) by this factor and retried, up to
# this many times, before giving up and leaving that group at offset 0 (not
# a hard failure -- shaped_initial_route()'s band-only/plain-route fallbacks
# still apply). Inactive on the current obstacle field: every group's own
# destination (+-560 m and the near-centreline group) was measured reachable
# at every candidate altitude on the first try, so this exists purely for
# generalization to configurations where that isn't true.
GUIDE_LANE_SHRINK_FACTOR = 0.85
GUIDE_LANE_SHRINK_MAX_TRIES = 8
# Altitude/corridor profile, expressed as fractions of the CURRENT leg's own
# (start_x, goal_x) span rather than absolute metres, so the same
# band_waypoints() serves both the first leg (0->5000 m) and the much shorter,
# variably-positioned post-switch leg.  Climb to the band by CLIMB_FRACTION,
# hold it to CRUISE_END_FRACTION, rejoin the originally planned altitude by
# DESCENT_END_FRACTION, and follow the planned route unchanged past that.
# These values reproduce the first leg's previously-tuned absolute breakpoints
# exactly (0.12*5000=600, 0.80*5000=4000, 0.98*5000=4900 m). Unwinding the band
# earlier (fractions 0.52/0.68, i.e. before the 3500 m goal switch) was tried
# and was worse, not better: baseline 365 vs 531 cycles. The descend-and-rejoin
# manoeuvre is itself what the single-UAV layer chokes on, so doing it sooner
# only moves the failure earlier.
GUIDE_BAND_CLIMB_FRACTION = 0.12
GUIDE_BAND_CRUISE_END_FRACTION = 0.80
GUIDE_BAND_DESCENT_END_FRACTION = 0.98

# Post-switch replan profile.  A UAV crossing GOAL_SWITCH_SLANT_RANGE is
# already at its cruise band altitude, so there is no climb to do -- the climb
# ramp is a harmless no-op because band_altitude passed in equals the UAV's
# current altitude (see shaped_switch_route). CLIMB_FRACTION is a small
# positive epsilon only to satisfy band_waypoints()'s strict ordering check.
#
# Cruise/descent fractions are NOT a copy of the first leg's 0.80/0.98: this
# leg is only ~1500 m (vs. 5000 m), and the worst-case altitude change is much
# larger relative to that -- a UAV at the 90 m band switching to the rear
# group's y=10 goal must lose 80 m. A first attempt used the first leg's "last
# 10%" window, which on this leg leaves only ~135 m of horizontal distance for
# that 80 m drop (~27 degrees) -- steeper than the down-acceleration budget
# (A_Y_DOWN_MAX = 1g, tighter than the 2g climb budget) can fly, and it
# reproduced exactly the "base single-UAV planning failed" hard failure this
# shaping is supposed to avoid. 0.50/0.95 leaves ~45% of the leg (~675 m) for
# the same 80 m drop (~6.8 degrees, comparable in kind to the first leg's own
# descent slope) while still holding the pre-switch altitude through the
# immediate post-switch window, which is where the pileup this shaping targets
# actually happens (diagnosed within ~50 m of the switch trigger).
GUIDE_SWITCH_BAND_CLIMB_FRACTION = 0.01
GUIDE_SWITCH_BAND_CRUISE_END_FRACTION = 0.50
GUIDE_SWITCH_BAND_DESCENT_END_FRACTION = 0.95

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
