# OSM route mode (Robotour 2026, Stromovka)

Operational notes for the config-switchable route-planning mode added to
the `followme` app. Design rationale lives in the module docstrings
(`osm_router.py` first, then item 28 in `tulak_obstacle.py`); this file is
the "how do I actually run it" half.

## What it does

Everything is driven by QR codes, so changing what Matty is doing never
means editing a config in the field.

| You show / do | Matty does |
|---|---|
| release the emergency stop | **WAITING** - stopped, holding |
| QR **`start`** or **`go`** | **FREE** - road mask + obstacle avoidance only. No route, no GPS bearing, no corridor bias |
| QR **`50.106589, 14.417428`** | plans a route over the OSM path network and **FOLLOWS** it |
| QR **`stop`** (or `abort` / `cancel`) | forgets the target, back to **WAITING** |
| press + release the e-stop | back to **WAITING** from anywhere |

All matched case-insensitively on the whole decoded string. A code that is
neither a command nor a coordinate pair is logged and ignored, so a stray
sticker cannot change the mode.

Every transition is idempotent against the camera re-reading a held-up
code ~10x/second. The guard is on *"does this change the state"*, not
*"is this the same text as last time"* - which is why `start` -> `stop` ->
`start` works while `start` -> `start` does nothing. Showing the same
coordinates again **after a `stop` does re-plan** (the stop cleared the
target, so it is a new one); showing them while already following does
not. Multi-checkpoint runs need nothing extra: arrive, get shown the next
code, continue.

`start`/`go` is the debug mode: it exercises the NN road following, the
depth avoidance and the whole tuned state machine, with only the route
guidance switched off.

### The competition mode, in one sentence

Instead of a bearing pointed at the destination, Matty follows an aim
point a few metres ahead **on a mapped path**. That distinction is the
whole point: a bearing to a point 200 m away goes straight through
whatever lies between - which in a park is a lawn.

Road following and obstacle avoidance are unchanged and still have
priority. The router never sees a depth reading and cannot relax a
threshold, start a maneuver, or suppress a stop.

### QR coordinate formats accepted

`50.1290345, 14.3770581` · `50.1299558N, 14.3793860E` ·
`N50.12, E14.38` · `14.38E, 50.12N` (letters resolve the order) ·
`50°5'14.8"N 14°25'14.4"E` · a pasted Google Maps URL.

> The `N`/`E`-suffixed form was **silently rejected before 2026-09-04** —
> it decoded fine and was then discarded as "not coordinates", so three
> targets shown at CZU were ignored with no visible error.

### Maps: one file or several

`map_file` takes a path **or a list**. With a list, every map is loaded at
boot and the one to use is chosen from the first GPS fix — so the same
config works at CZU and at Stromovka with nothing to edit:

```json
"map_file": ["maps/stromovka.json", "maps/czu_suchdol.json"]
```

Selection prefers a map whose bbox contains the fix, then the nearest
network; it is logged (`ROUTE: using map czu_suchdol.json ...`).

### Config gates

Both default to the full competition behaviour:

| Key | Effect when `false` |
|---|---|
| `enable_gps_routing` | coordinate codes ignored; `start`/`stop` still work |
| `wait_for_start_qr` | no start gate - boots straight into FREE, the pre-protocol behaviour |

## What went wrong at CZU (2026-09-04) and what changed

The start/stop protocol worked perfectly. Two separate bugs made the
*coordinate* path look unstable, and neither was the QR reading:

**1. The wrong map was loaded, and nothing said so.** Matty ran at CZU
Suchdol with `maps/stromovka.json` — bbox 50.0975–50.1135, 14.407–14.437,
about 2.5 km away. Both the robot's own position (2474 m) and the target
(2511 m) snapped to the *same* corner of the Stromovka graph, so the route
came out **0.0 m long** and the router immediately published
`arrived` + `hold: stop`. From the outside: *"it read the coordinates and
then refused to move."* A warning did exist (`target_snap_warn_m`) but it
printed to stdout, which the log does not capture.

**2. `50.1299558N, 14.3793860E` did not parse.** The decimal-with-
hemisphere format was not handled, so three targets were shown and
silently ignored.

Fixed, all covered by `test_qr_protocol.py`:

- planning is **refused** when either end is more than `max_snap_dist_m`
  (50 m) from any mapped way, with the real cause named — including
  *"OUTSIDE the loaded map area … wrong map_file for this location?"*
- an independent cross-check rejects any route shorter than the arrival
  radius while the target is further away in a straight line
- the refusal reason is published in `route_hint`, so it lands **in the
  log**, not just on stdout
- the map area is printed at boot
- `map_file` accepts a list and picks the right map from the first fix
- the hemisphere formats parse, in every prefix/suffix/swapped order

Also confirmed: **`start` erasing the target is by design** — it enters
FREE mode, which drops the route. You read that correctly.

> **Note on your `terminate_on_stop: True`.** With that setting the
> emergency-stop *release* cannot reset the mode, because the press raises
> `EmergencyStopException` and ends the run first. That is a fine
> workflow — you restart and land in WAITING anyway — but if you want the
> press/release cycle to act as a mode reset without restarting, set it to
> `false`. The app then holds itself stopped while the button is engaged.

## Three more fixes from the 2026-09-04 CZU logs

**Matty reversed while WAITING.** Holding a QR code close to the camera IS
an obstacle, and the avoidance state machine was still running underneath
the hold — which was applied as `min(speed, 0)` on the finished command,
and that does nothing to a *negative* speed. Now an absolute hold
(`stop`, or `creep` with `route_hold_creep_speed = 0`) returns **before**
the state machine runs, so nothing can command a maneuver. The depth
pipeline is untouched and every sensing handler keeps running and logging
— the reaction is suppressed, not the sensing, so the streaks are current
the moment the hold lifts.

**The far-away mask kept switching off.** Its blackout gate compared the
upper region's invalid fraction against 0.85 — but outdoors that region is
mostly *sky*, which legitimately returns nothing, so the fraction sat at
0.85–0.95 and straddled the threshold. Measured over 15 585 frames from
today: the fill was withheld in **20.6 %** of them, and in **96.6 % of
those the lower frame was 96 % valid** — the camera was working every
time. The gate now needs *both* the upper region mostly invalid **and**
the lower frame dead (`depth_far_mask_min_lower_valid_frac`, the same
ground-band discriminator the blind hold already uses):

| | withheld |
|---|---|
| old gate | 3213 / 15585 = **20.6 %** |
| new gate | 110 / 15585 = **0.7 %** |

Run 174729 goes from 77.3 % → 0.5 %. The genuine blackouts (the indoor
runs, lower frame 0.00 valid) are **unchanged** — still caught.

**The compass is stable but mis-calibrated.** Two independent fits, six
days apart, different sites, different reference (course-over-ground vs
fix-differencing):

| | constant | hard-iron | phase |
|---|---|---|---|
| 2026-08-29 Stromovka | +6.2° | 9.7° | 12° |
| 2026-09-04 CZU | +7.5° | 10.9° | 6° |

Drift *within* a run was only ±3° over 150–300 s, so the sensor is steady —
it is biased in a **direction-dependent** way, which is hard iron
(something ferrous on the robot, turning with it). No single offset can
remove it. Config now carries `compass_offset_deg: 7.0` plus
`compass_hardiron_deg: 10.3` / `compass_hardiron_phase_deg: 9`, which over
623 samples from both sites takes the heading error from a median of
**8.0° to 3.1°** (p90 14.1° → 8.8°).

Re-fit with **`fit_compass.py`** after anything is added to, removed from
or moved on the robot — it prints the config block to paste:

```bash
python fit_compass.py "../../logs/**/2026_09_04_v3_czu_test0/*.log"
```

## How big a map can Matty carry?

The whole of Prague is **65 MB on disk** — disk was never the constraint.
RAM and boot time are:

| map | file | boot | RAM | snap |
|---|---|---|---|---|
| Stromovka | 0.8 MB | 0.1 s | 12 MB | 0.29 ms |
| CZU Suchdol | 0.3 MB | 0.0 s | 5 MB | 0.06 ms |
| both, as a `map_file` list | 1.1 MB | 0.1 s | **10 MB** | — |
| **all of Prague** | 64 MB | **8.8 s** | **1003 MB** | 0.08 ms |

`snap()` used to be 39.6 ms on the Prague map, called on every GPS fix —
now indexed by a 100 m grid, so it is **0.08 ms** and independent of map
size (verified identical to brute force on 1200 random probes across all
three maps). A* is 37–59 ms for routes up to 6 km.

**Recommendation: ship a list of per-venue maps, not the city.** It gives
the same "no config editing" benefit for **1 % of the RAM**. Prague-wide
now works if you want it, but 1 GB and 8.8 s of boot buys nothing you
cannot get from two 400 kB files.

## What happens if GPS drops out

**First, which odometry.** This is the ESP32's **wheel encoders** via
`matty.py`, not the OAK's visual odometry — `is_visual_odom` is not
enabled, `oak.pose3d` is not linked to anything, and nothing in this app
consumes it. The common warning about OAK VIO being unreliable does not
apply here. Furthermore the dead reckoning uses only the **forward
component projected on the robot's own heading**, i.e. distance travelled —
so heading drift, the failure mode that actually kills odometry, never
enters.

Measured wheel-odometry distance error against GPS, forward component only:

| window | n | median odometry | error median | p90 |
|---|---|---|---|---|
| 5 s | 770 | 2.5 m | 6.9 % | 21.1 % |
| 10 s | 429 | 4.9 m | 6.0 % | 20.1 % |
| 20 s | 225 | 9.9 m | 5.4 % | 19.3 % |
| 40 s | 115 | 19.6 m | 6.1 % | 18.9 % |

**Flat from 5 s to 40 s** — that is a scale error (wheel radius, slip), not
a drifting integrator, and GPS measures the chord while odometry measures
the path so this is an upper bound. At 0.5 m/s a 15 s window is 7.5 m, and
6 % of that is 0.45 m of along-route uncertainty — small against the 8 m
corridor threshold.

Dropouts themselves: **25 longer than 2.5 s, median 3.5 s, none longer than
4.0 s**.

| fix age | behaviour |
|---|---|
| 0 – 15 s | follow the route on odometry, nothing changes. Covers every dropout ever recorded |
| 15 – 45 s | **degraded**: authority fades toward the road mask, speed capped to 0.3 m/s, **cross-track withheld** |
| > 45 s | **`gps_lost`**: route guidance off, road following + obstacle avoidance only — the same thing `start`/`go` does |
| fix returns | re-plans from wherever the robot actually is, and resumes |

Two deliberate choices:

**Cross-track is withheld, not frozen.** It cannot be measured without a
fix, and a frozen value is a constant steering offset with no feedback —
that's how you spiral off a path rather than hold it. The follower's
corridor bias goes to zero on its own.

**It does not dead-reckon onward to the target.** The last known point is
where the robot already *is*, so it isn't a target; and steering at a
distant one on wheel odometry that this project documents as drifting
through blind turns would be *confidently wrong* rather than merely
uninformed. Reflexive road-following is both safer and the best-tested mode
there is. **The target is kept**, so nothing is lost when the fix returns.

## Articulated steering — the flipping risk

The reported incident: a pole in the middle of the path, correctly
detected, passed on the left. The camera lost it, the right zone read
clear, Matty turned hard right — and the **rear-right** wheel caught the
pole and started to climb it, loading the single central articulation joint
in exactly the way it must never be loaded.

### The geometry (measured 2026-09-05)

| | |
|---|---|
| boxes | 17.5 × 22.5 cm, front and rear |
| gap between them | 14.5 cm, **joint at its centre** |
| bumpers | 3 cm fore and aft → total length 55.5 cm |
| wheels | +6.5 cm each side → **total width 35.5 cm** |
| axle separation | 31 cm |

From those: joint → rear bumper **L = 0.2775 m**, half width **W = 0.1775 m**,
so joint → rear outer corner **r = 0.329 m** at **φ₀ = 32.6°** off the long
axis.

Articulating by γ rotates the rear body about that joint, so the corner's
lateral extent from the centreline is

> **extent(γ) = r · sin(φ₀ + γ)**

which is exactly `W` at γ=0 and peaks at `r` when φ₀+γ = 90°, i.e. **57° of
articulation**. Beyond the body edge that is **0.152 m** at the peak and
**0.144 m** at the platform's own 45° limit — not a rounding error, and
precisely why a pole the front passes cleanly can still be caught by the
rear wheel. Inverting gives the steering ceiling:

> **γ ≤ asin((clearance − margin) / r) − φ₀**

### Why it was invisible

Measured on hard-steer forward cycles:

| state | obstacle beside <0.8 m | *visible to the camera* |
|---|---|---|
| TURNING | 63.4 % | 30.4 % |
| REALIGNING | **62.5 %** | **7.8 %** |

Nothing on this robot looks sideways or back, so a short **per-flank
memory** (6 s / 2 m, expiring on both) supplies what the live zones cannot.

### One correction worth knowing

The side zones report **range** to something in a forward-*diagonal* sector
(roughly 11–29° off axis), **not** lateral clearance. A 1.0 m reading is
about 0.34 m laterally and 0.94 m ahead. Using the range as clearance —
the obvious mistake — overstates the room by about 3×. `side_zone_bearing_deg`
(20°, the sector centre) does the conversion; the exact value barely
matters, 11° vs 20° changes the binding rate only 3.6 % → 3.1 %.

Applied to **both** flanks: which corner swings which way depends on how
the articulation change is shared between front and rear wheels, which
depends on grip and is not knowable here. Symmetric is the honest reading
and costs little, since it only binds within about half a metre.

Effect on the recorded runs: **3.1 % of cycles changed, median reduction
20°** — which is exactly the difference between a 45° swing into 0.3 m of
clearance and one that fits. It only ever *reduces* magnitude and never
flips sign (asserted over 4000 random states).

> `matty.py` has `FRONT_REAR_AXIS_DISTANCE = 0.32` against your measured
> **0.31** — a 3.2 % error in the turning-radius model. It does **not**
> affect the dead reckoning below (that uses the forward projection, i.e.
> `speed × dt`, not the radius), and heading normally comes from the IMU,
> so the impact is small — but it is wrong and worth correcting.

## Waypoints on bends and at crossroads

**Sharp bends were never treated as turns.** `junction` counts graph
*degree*, so a switchback or a corner around a building looked like
straight going and got the full 8 m lookahead, full speed and the 20°
cruising ceiling. Sampled along real routes, away from any topological
junction, the aim point sat within 6–8° of the road direction at p90 — but
reached **164°** at worst. That is the route asking Matty to drive off the
path, which is exactly what destabilises the road mask.

Two changes: `Route` now flags **corners** (bend ≥ `corner_angle_deg`, 35°)
and treats them exactly like junctions — short lookahead clamp, raised
steering ceiling, reduced speed; and `_limit_aim_offset` shortens the
lookahead until the line to the aim point stops diverging from the road's
own direction (shortening, not clamping the bearing, so the aim point stays
*on* the path and the search converges by construction).

| corridor-mode aim offset | p99 | max | >45° |
|---|---|---|---|
| Stromovka before | 15° | 48° | 0.1 % |
| Stromovka after | 15° | **24°** | **0.0 %** |
| Suchdol before | 46° | 82° | 1.0 % |
| Suchdol after | **20°** | **37°** | **0.0 %** |

Junction-mode coverage rose only 20.0 → 20.4 % (Stromovka), so corners are
caught without flooding the route with junction behaviour.

## Should the road mask get a focus window?

Tested, and the answer is **weight, don't crop**. Over 44 623 frames:

| centroid method | frame-to-frame jitter | goes blind |
|---|---|---|
| plain (current) | 2.34 px | never |
| **row-weighted (near rows more)** | **2.15 px** | never |
| fixed mid-frame band | 2.61 px — *worse* | **2.5 % of frames** |

A hard window costs signal and can leave Matty with no steering input at
all, which is exactly the "don't limit Matty's decisions" concern. A
weighting cannot. The ramp earns its place because the drivable fraction
rises from **0.03** just under the sky cut to **0.81** at the bottom — the
far rows carry little signal and most of the noise (pitch error moves the
horizon there, and it is where the out-of-distribution lawn-as-road
hallucination lives). `mask_row_weighting` is on by default; it shifts the
centroid by only 0.95 px at the median, so it does not re-decide ordinary
steering.

A pitch-adaptive sky cut was considered and dropped: at 0.03 drivable
fraction just below the current cut there is almost nothing there to gain.

**Persistence gating was considered and rejected as redundant.** 65 % of
large deflections (≥12°) are single isolated frames — but
`max_steering_rate_deg_s` (30°/s = 3° per cycle) already caps what a
one-frame spike can do, which is why DRIVE shows only 1.1 % of hard-steer
cycles turning into a close flank against TURNING's 30 %.

## Compass: how much more data would help?

Almost none, and the reason is worth knowing. Bootstrapped over the 623
straight-driving samples now available:

| samples | offset (95 % band) | hard iron (95 % band) |
|---|---|---|
| 75 | ±1.5° | ±2.5° |
| 300 | ±0.8° | ±1.1° |
| **623 (now)** | **±0.6°** | **±0.8°** |

The parameters are already pinned to well under a degree. Meanwhile the
residual after the fit is 3.1° median — and the consecutive-sample scatter
of the error itself is **1.7°**, so most of what is left is *GPS course
noise, not compass error*. A soft-iron (2nd harmonic) term was fitted and
came out at **0.5° amplitude**, improving the median by 0.03° — there is no
soft-iron problem to solve.

What actually helps, none of it needing a test area:

1. **Heading coverage.** This is the binding constraint, not sample count.
   Current: 130 samples facing 0–45° but only **8** facing 135–180° and 36
   facing 315–360°. *Driving the same loop in the opposite direction
   doubles coverage for free.*
2. **Longer straight legs above 0.25 m/s** — course over ground is unusable
   below that and while turning, so straight fast driving is what produces
   samples at all.
3. **The same route on different days / at different sites** — a term that
   repeats is the robot's own hard iron; one that does not is a local
   magnetic anomaly.
4. **Re-fit after any hardware change.** The hard-iron term went 9.7° →
   10.9° between August and September; the constant barely moved.

## Two things the logs actually say about the road mask

Both reproducible with `replay_osm_router.py`; they drive every design
decision below.

**1. The mask supplies no restoring signal toward the road.** While 1-8 m
off the planned corridor it steered *back* toward it in only **40-48 % of
frames** - worse than a coin flip - mean contribution **-0.7 to -1.0 deg**,
i.e. very slightly *away*:

| \|cross-track\| | restoring % | mean signed deg |
|---|---|---|
| 0-1 m | 48.2 | -0.11 |
| 1-2 m | 41.9 | -1.03 |
| 2-4 m | 39.9 | -0.73 |
| 4-8 m | 48.5 | -0.71 |

This is not a defect. It is a **lane-keeping** sensor with no notion of
*which* lane. Once Matty is on the grass and the grass reads drivable,
nothing in the mask ever says "the road is over there". That is the exact
mechanism behind what you saw in the field, and no amount of tuning the
mask fixes it - it is answering a different question.

**2. The mask degrades predictably with camera-vs-road misalignment**,
which is measurable because the map knows where the road runs:

| camera off road axis | multi-blob % | centroid pulled >10 % off largest blob |
|---|---|---|
| 0-15 deg | 26.7 | 7.3 |
| 15-30 deg | 26.5 | 9.4 |
| 30-45 deg | 36.7 | 18.2 |
| **45-60 deg** | **51.1** | **30.3** |

`mask_center()` averages *all* drivable pixels, so a frame containing a
road blob and a lawn blob yields a centroid pointing at **neither** -
straight at the boundary between them. And 45-60 deg off-axis is precisely
the state an avoidance turn leaves the robot in.

> **Tested and rejected:** using the largest connected component, or the
> component nearest the bottom-centre of the frame, instead of the global
> centroid. Neither beat the plain centroid against a map-derived
> reference, *including on fragmented frames*. The centroid is not the
> problem; the absence of any road-anchored reference is. Recorded so it
> is not re-attempted.

## Division of labour

Split by **frequency band and question**, not by trust level:

| Band | Owner | Question it answers |
|---|---|---|
| immediate | depth / avoidance | is the way ahead passable *right now* (overrides all) |
| fast (10 Hz) | RedRoad mask | lateral centring within the surface ahead |
| slow (<0.1 Hz) | OSM + GPS + compass | which branch, and how far off the corridor |

The key asymmetry: **GPS error is noisy but bounded and roughly zero-mean
(1-3 m); mask error is unbounded and self-reinforcing** - a 60-second,
35-metre excursion. For the *slow* terms, bounded-but-noisy beats
smooth-but-divergent, and heavy low-passing makes the former usable. The
map does not have to be accurate; it has to be non-divergent.

## Three control situations

`_guidance()` names the situation rather than interpolating one number:

**CORRIDOR** (default, ~82 % of cycles) - the route is a slow **additive
bias**, never a blend. A blend at 45 % authority lets the mask *cancel*
the only restoring signal there is; an addition cannot be outvoted, it
shifts the equilibrium the mask settles into while leaving the mask in
charge of fast corrections. Gains are gentle on purpose - ~3 deg/m of
cross-track, capped at 15 deg, heavily smoothed - because this is the
*low-frequency* term and GPS noise must never become a sharp correction.
Measured on run 130657: median bias 3.9 deg, p90 7.6 deg.

Alongside it, `mask_trust` fades `last_dir` from 1.0 to 0.25 between 25 deg
and 60 deg off the road axis, per measurement 2. It engaged in 18 % of
cycles on that run.

**JUNCTION** (~18 % of cycles) - the route takes over (authority 0.85)
*and* gets a much larger steering ceiling. This is the fix for the
T-junction concern:

| steering | turning radius (`0.16/tan(theta/2)`) | arc for a 90 deg turn |
|---|---|---|
| 20 deg (old ceiling) | 0.91 m | 1.43 m - swings wide across the corner |
| **40 deg** (junction) | **0.44 m** | **0.69 m** - fits inside a path |
| 45 deg (platform max) | 0.39 m | 0.61 m |

The steering *rate* limit is relaxed to 90 deg/s alongside it - at the
cruising 30 deg/s, winding on 40 deg takes 1.3 s, about 0.65 m, most of
the junction - and speed is capped to 0.3 m/s so the turn has room. The
rate is only ever taken as a **relaxation**, never a tightening.

**RECOVERY** - confirmed off the corridor. Route dominant, short lookahead
so the return angle is steep, speed capped.

### Safety net kept at junctions

If the mask sees essentially *no* drivable surface the way the route wants
to turn (`route_min_road_frac`, 0.02 against a healthy 0.20-0.31), the
route's authority is cut to `route_veto_authority` (0.3). Deliberately a
veto at a very low threshold, not the old proportional road-fraction gate
- that gate would have blocked legitimate turns onto a branch sitting at
the edge of the frame, which is the junction case itself.

## First knobs to turn

| Symptom | Knob |
|---|---|
| drifts off path centre on straights | `route_cross_track_gain_deg_per_m` up |
| weaves / fights the mask on straights | `route_bias_max_deg` down, or `route_bias_alpha` down |
| swings wide at forks | `junction_steer_limit_deg` up, `junction_speed_limit` down |
| takes the wrong branch | `junction_authority` up, `junction_zone_m` up |
| follows a hallucinated lawn after a maneuver | `mask_trust_full_deg` down (fade sooner) |

## Files

| File | Purpose |
|---|---|
| `osgar-apps/followme/osm_router.py` | the `OSMRouter` node: graph, A\*, tracking, corridor monitor |
| `osgar-apps/followme/osm_fetch.py` | downloads the offline map from Overpass |
| `osgar-apps/followme/maps/stromovka.json` | pre-downloaded Stromovka extract (2 233 ways) |
| `osgar-apps/followme/config/matty-tulak-osm.json` | the run config |
| `osgar-apps/followme/replay_osm_router.py` | offline validation against recorded logs |
| `osgar-apps/followme/test_qr_protocol.py` | the QR run-mode state machine, no log/robot/network needed |
| `osgar-apps/followme/view_obstacle.py` | now shows a route HUD line for OSM-mode logs |

## Before the competition

The robot has **no internet at Stromovka**, so the map must be on disk.
It already is, but re-fetch if the area changes:

```bash
cd osgar-apps/followme
python osm_fetch.py --bbox 50.0975 14.4070 50.1135 14.4370 -o maps/stromovka.json
```

Pick the bbox generously. A graph truncated at the bbox edge does not fail
loudly — it quietly stops offering the ways that continue past the cut, so
the router either detours or calls the target unreachable.

Check what a saved map contains without downloading anything:

```bash
python osm_fetch.py --info maps/stromovka.json
```

Sanity-check a route you expect to need, before the day:

```bash
python replay_osm_router.py --map maps/stromovka.json --from 50.10583 14.42734 --target 50.10659 14.41743
```

## Running

```bash
cd osgar-apps/followme
python -m osgar.record config/matty-tulak-osm.json
```

`osm_router:OSMRouter` and `tulak_obstacle:TulakObstacle` are both resolved
relative to the working directory, so launch from `osgar-apps/followme` as
before. The map path (`maps/stromovka.json`) is resolved relative to the
module, so it works from anywhere.

Startup sequence:

1. Camera boots; the app holds still (`have_obstacle_data`), as always.
2. Router is in WAITING and publishes `hold: creep` → forward speed capped
   at `route_hold_creep_speed` (**0.0 by default = stand still**).
3. Release the emergency stop.
4. Show a QR — `start`/`go` for plain road following, or coordinates for a
   route. The router plans on the first GPS fix and logs the route length,
   point count and junction count.
5. Hold releases, Matty drives.
6. On arrival it publishes `hold: stop`. Show the next checkpoint's QR to
   continue, or `stop` to park it.

**`terminate_on_stop` is `false` in this config**, unlike
`matty-tulak-obstacle.json`. That is what lets the press/release cycle act
as a mode reset instead of killing the process. The app holds itself
stopped for as long as the button is engaged (ahead of every other hold,
including the blind hold) rather than trusting the ESP32 firmware to
refuse motion — which the code cannot verify. Set it back to `true` if you
would rather the run end on the button.

## What to watch in the log

Every `ROUTE:` line comes from the router. The steady-state one:

```
ROUTE: following   progress=63/191m cross=+1.2m next_junction=8m authority=0.72 aim=50.105123,14.426004
```

- **cross** — signed distance off the planned corridor, `+` = robot right
  of the route. Should sit under ~3 m.
- **authority** — climbing above 0.45 means a junction is coming.
- The app's own GPS status line ends with `(osm-route:following authority=… remaining=… cross=…)`.

Router states you will see in `ROUTE:` lines: `waiting`, `free`,
`no_fix`, `planning`, `following`, `recovering`, `lost`, `arrived`,
`failed`.

Escalation, in order:

| Log line | Meaning |
|---|---|
| `off the planned corridor (…) - steering back` | > 8 m off for > 6 s → short lookahead, authority 0.9 |
| `re-planning (drifted onto another mapped path)` | off the plan but on a *different* real path — avoidance moved us, nothing wrong |
| `re-planning (recovery took too long)` | 30 s of recovery without getting back |
| `LOST - Nm from the nearest mapped way … holding` | > 25 m from *any* path for 10 s → full stop, needs a human |
| `NO ROUTE to … target unreachable` | check the map bbox and the highway filter |
| `WARNING - the QR target is Nm from the nearest mapped way` | the route ends at the mapped point, not the coordinate |

## Which ways Matty may use

Whitelist in config, so it can be changed without re-downloading. On the
Stromovka map, 351 of 2 233 ways are excluded, including **all 66
staircases** — there are 10 of them inside the area driven on 2026-08-29,
connecting otherwise sensible paths, so a router that ignores this *will*
route over one. Also excluded: trunk/secondary roads (live traffic),
`access=private`, `foot=no`, `sac_scale`, construction/proposed/platform.

Everything else is allowed but costed — `cost = length × highway penalty ×
surface penalty` — so a dirt path stays usable but has to be meaningfully
shorter to beat asphalt.

**A path that exists on the ground but not in OSM is never routed onto**,
because the graph has no edge there. That is the "don't take a legit-looking
unmapped road" rule, and it needs no separate check.

## Validation done so far (no hardware)

Run-mode state machine — `python test_qr_protocol.py`, 22 assertions
covering every transition above plus both config gates. Needs no log, no
robot and no network.


Replayed against all 2026-08-29 Stromovka logs with the real OSM data:

```bash
python replay_osm_router.py "../../logs/2026_08_matty-tulak-obstacle/2026_08_29_V2_Stromovka0/*.log" --map maps/stromovka.json
```

- **11 of 14 runs**: cross-track median 0.2–1.9 m, **zero** recovery or lost
  time. No false alarms.
- **125548** (drove onto the lawn, median 14 m off-path): 83 s recovering,
  then `LOST`. Correctly caught.
- **130005** and **134647** (the two shorter real excursions I found at
  t=257–314 s and t=158–244 s): 55 s and 31 s recovering. Correctly caught.

So the corridor monitor fires on exactly the three runs with real
excursions and on none of the eleven without. Thresholds come from that
data: on-path fixes sit at a 1.25 m median, real excursions run 6–35 m
sustained for 20–60 s — a wide empty band between "noisy" and "actually off".

End-to-end, router → real `TulakObstacle` → `desired_steering`:

```bash
python replay_osm_router.py <one.log> --map maps/stromovka.json --with-app
```

On run 124618 (191 m route, 3 junctions): route mode engaged 100 % of
cycles, bearing authority min 0.45 / median 0.45 / max 0.85 exactly as
designed, speed median 0.50 m/s, arrived at t=425 s.

Control terms all engage and stay bounded (run 130657, 221 m route):
guidance modes corridor 82 % / junction 18 %; camera-vs-road axis median
9 deg, p90 50 deg; `mask_trust` below 1.0 in 18 % of cycles, minimum 0.25;
corridor bias median 3.9 deg, p90 7.6 deg, max 15.0 (the cap); route steer
limit 20-40 deg, above 25 deg in 22 % of cycles; speed cap active 18 %.

Performance: A\* is 0.5-1.4 ms for park-scale routes, snapping 0.17 ms -
negligible against the 10 Hz control loop.

Visual check of one run:

```bash
python replay_osm_router.py <one.log> --map maps/stromovka.json --html route.html
```

Blue = planned route, dots = recorded track coloured green/orange/red by
cross-track. A red arc bulging away from the blue line is the robot on
the grass.

## Known limitations

- **`oneway` is ignored.** Every allowed way is bidirectional. Fine in a
  park at 0.5 m/s; a real omission on service roads shared with cars.
- **Barrier nodes (gates, bollards) are not modelled.** A bollard is fine
  for Matty, a closed gate is not, and OSM rarely distinguishes them.
- **A route that starts with a U-turn is planned, not avoided** beyond
  `uturn_penalty_m` biasing the first step (measured: it accepts ~5 m of
  extra route to start in the direction faced). Matty's minimum turning
  radius is ~0.39 m so a U-turn on a 3 m path is geometrically possible,
  but it was never tested — **place the robot facing roughly along the
  intended direction at the start.**
- **No elevation.** Stromovka is flat enough; a steep park would want an
  incline penalty.
- **A hard junction turn is commanded from a GPS/compass bearing.** At a
  fork the route can ask for up to 40° of steering. With compass error
  around 6° and GPS around 2 m the aim bearing can be ~20° off, and
  turning hard the wrong way at a fork is the worst case in this design.
  Mitigated five ways — the ceiling ramps in over `junction_zone_m` rather
  than switching on, the mask keeps 15 % of the blend,
  `route_min_road_frac` vetoes a turn into nothing, speed is capped at
  0.3 m/s, and obstacle avoidance overrides all of it — but watch this
  first in the field.
- **`mask_trust` assumes the map is right about the road axis.** If the
  planned route is wrong, fading the mask out removes the sensor that
  could have noticed. Bounded by `mask_trust_min` (0.25), never zero.
- **`terminate_on_stop` is `false` in the OSM config.** The process
  survives the emergency-stop press so that the release can act as a mode
  reset. The app holds itself stopped meanwhile, ahead of every other
  hold, but this is a deliberate change to what the physical button does
  to the software — set it back to `true` if you would rather the run end.
- **Nothing has run on hardware.** Everything above is log replay.

## Bench check before driving

1. Confirm the map loads: the console prints
   `OSM map …: 7166 routable nodes, 8292 segments (351 ways excluded)`.
2. Confirm Matty stays still with no QR shown, and that the log says
   `ROUTE: waiting`.
3. Run `python test_qr_protocol.py` — 22 assertions over the whole run-mode
   state machine, no robot needed.
4. Show a `start` QR; confirm `ROUTE: QR "start" - road following only` and
   that Matty drives on the road with no GPS involvement.
5. Show `stop`; confirm it parks and logs `waiting for a QR`.
6. Press and release the emergency stop; confirm
   `ROUTE: emergency stop released - waiting for a QR`.
7. Show a QR for a target ~50 m away; confirm `ROUTE: planned new-target`
   with a plausible length and junction count, and that the hold releases.
8. Confirm `compass_offset_deg` matches between the `app` and `osm_router`
   config blocks — the router uses it only for the U-turn bias, so a
   mismatch costs a suboptimal first step, never a wrong route, but keep
   them in sync anyway.

## Reverting

Run `config/matty-tulak-obstacle.json` instead. With no `osm_router`
module wired, `route_mode` in `tulak_obstacle.py` stays `False` and every
branch that reads it is skipped — the driver behaves exactly as it did
before this mode existed. Verified by replaying run 124618 through the old
config after every change above: `route_mode False`, `mask_trust 1.0`,
`bias 0.0`, status line `(no target)`, and a bit-identical command profile
(speed median 0.50 / max 0.50; steering median -0.2 / min -20.0 / max
45.0).
