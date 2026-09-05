"""
  Obstacle Detection 3D - Zones

  Alternative to osgar.obstdet3d:ObstacleDetector3D that looks at three
  column bands (left / centre / right) of the depth frame instead of
  just the centre, so a caller can tell not only "is something close"
  but also "which side currently has more room".

  The centre band uses the SAME row/column window as the original
  ObstacleDetector3D by default, so stop_dist behaviour calibrated
  against that module carries over unchanged. Left/right bands are new
  - their column ranges are a starting point only, NOT calibrated.
  Check them against your own depth viewer (Oak-D Pro, 400x640) and
  adjust to match the camera's real FOV and the width of the robot.

  Two robustness changes over the original, aimed at the kind of
  "flying pixel" edge noise visible right at the ground/background
  boundary in stereo depth (a thin band of spuriously near readings
  where the true depth jumps from close ground to a far background):

    - a window is only trusted if at least min_valid_frac of its
      pixels have a valid (nonzero) reading; otherwise it is reported
      as unknown rather than as whatever the few stray pixels say
    - distance is a low percentile of the valid pixels rather than the
      bare minimum, so a handful of near-zero stray pixels can't set
      the whole reading by themselves

  The centre zone still falls back to 0.0 ("assume the worst") when
  untrusted, to preserve the original fail-safe behaviour used for
  hard stops. Left/right are only ever used to pick a turn direction,
  so an untrusted window there is reported as None (unknown) instead.

  --- Free-space profile (depth_profile) ---

  A fourth output, additional to and independent of obstacle_zones:
  free_space_bins distances (default 9) across free_space_cols, same
  row window and per-bin logic as the L/C/R zones, just finer-grained -
  a coarse 1D depth scan rather than a 3-way left/centre/right split.
  Purely additive - obstacle_zones is unchanged, still the only thing
  hard-stop/avoidance logic should key off.

  Deliberately NOT thresholded here into "open"/"blocked" the way the
  hard-avoidance zones are - that would require knowing what counts as
  "open enough", which is a moving target (see tulak_obstacle.py's
  adaptive turning_dist) that this module has no visibility into. This
  just reports the raw per-bin distances (None where a bin has too
  little valid data, same convention as left/right) and lets the
  consumer decide what "open" means for its own current situation - see
  tulak_obstacle.py's _free_space_steering for the intended use (a
  continuous steering nudge toward whichever bins have the most
  clearance, meant to run well before the discrete avoidance state
  machine's threshold, not replace it).

  --- Trusting a window, and what happens when you can't ---

  Two gates decide whether a window's reading is believable at all:
  min_valid_frac (enough non-zero pixels) and min_real_frac (enough of
  them GENUINELY MEASURED rather than far-mask fill - see __init__).
  A window failing either reports its fail value: None for the sides
  ("unknown", read as far), and center_fail_dist_m for the centre.

  center_fail_dist_m is a deliberate risk dial, not a tuning knob. At
  0.0 a blank centre reads as an obstacle at zero distance and stops the
  robot - maximum safety. Raised above the highest turning_dist the
  consumer can reach, missing data stops nothing, which is what a
  competition run may want and is exactly the protection that lets a
  close obstacle which has stopped returning stereo be driven into.
  Consumers must not test the published value against 0.0 to detect
  missing data once this is configurable.

  --- Synthetic far-fill cross-check ---

  oak_camera_v3.py remaps invalid pixels in the upper frame to a large
  constant (depth_far_mask_value_mm) so sky / out-of-range background
  reads as "far" rather than tripping the fail-safe below and freezing
  the robot. Necessary, and unchanged - but its blackout gate is
  evaluated over the whole masked region while the fill is applied
  per-pixel, so a locally-blind zone window (sunlit or low-texture wall
  at close range) gets filled too, and then reads as a confident "15m
  clear". _sanitize_zones() drops such a reading ONLY when a different
  zone actually measured something close, substituting that measured
  distance. When every zone reads far - the real open-sky case - it does
  nothing at all. See far_fill_suspect_m in __init__.

  --- Pitch-compensated row window ---

  `rows` (and `ground_rows`, see below) are now *base* windows that
  get shifted up/down each frame based on current pitch - read from the
  OAK-D Pro's own onboard IMU ('orientation_list', a fused rotation
  quaternion; see on_orientation_list) rather than the chassis ESP32 IMU
  this used to read from the platform's 'rotation' topic - using the mono
  camera's vertical FOV to convert pitch angle to a pixel shift.
  left_cols/right_cols/center_cols/ground_cols are NOT touched - pitch
  moves things vertically, not sideways.

  Four things keep this from reacting to every bump - or worse, turn -
  on a suspension-less chassis:
    - samples the IMU itself reports as degraded (estimated orientation
      error above max_pitch_error_rad - a per-sample field the chassis
      IMU never exposed) are dropped before they can affect pitch at
      all, not just smoothed over afterward - see on_orientation_list
    - samples with a physically-implausible roll (above
      max_plausible_roll_deg) are ALSO dropped, independent of the above
      - field testing found a sharp turn can corrupt the fused
      orientation (roll swinging to -64deg on a chassis that cannot
      physically roll that far) while max_pitch_error_deg's own
      confidence field stays at 0.0 the whole time, not catching it.
      Classic accel+gyro-fusion failure mode: lateral/centripetal
      acceleration during a turn gets misread as a change in gravity
      direction. Since pitch comes from the same quaternion as roll,
      it's corrupted too whenever this happens - see on_orientation_list.
    - the pitch used for the shift is further smoothed (pitch_smoothing_
      alpha, an exponential moving average) so a single-frame jolt
      barely moves it, while a sustained ramp climb does. Time constant
      is roughly (depth frame interval) / pitch_smoothing_alpha - e.g.
      at 10fps and alpha=0.15, ~0.7s. Can be tuned faster than a naive
      IMU-smoothing design would allow, because the accuracy gate above
      already screens out the samples most likely to be vibration/shock
      artifacts before they ever reach this EMA - this layer now mainly
      has to reject residual single-frame noise among ALREADY-trusted
      samples, not do all the noise rejection alone. If the window still
      feels laggy after raising this, check whether max_pitch_error_deg
      is rejecting too large a fraction of samples during the exact
      moments responsiveness matters (e.g. while actually climbing a
      ramp, which is also when vibration - and rejections - increase) -
      watch pitch_error_deg vs max_pitch_error_deg in the verbose log/HUD
      during a real transition before assuming the EMA alone is at fault.
    - the window only actually updates (and only logs) once the
      smoothed pitch implies a shift of at least min_shift_change_px
      pixels since the last update - small noise changes nothing

  When the window does move, it prints a line (unconditionally, not
  only under verbose) so you can see it happening from the OS console.

  This needs several things verified for your actual hardware/mount,
  same spirit as the "not calibrated" note on left/right columns above:
    - vertical_fov_deg: defaults to 55, the OV9282 mono VFOV Luxonis
      lists for the STANDARD (non-"W"/wide) OAK-D Pro.
    - the pitch AXIS itself (see _quat_to_rpy): CONFIRMED correct via
      hand-tilt test (rotating the camera changes roll_deg, tilting it
      changes pitch_deg monotonically, yaw_deg tracks heading) - each
      Euler angle tracks the physical motion its name implies.
    - pitch_offset_deg: defaults to 0, but should NOT be 0 in practice.
      The IMU die sits at some fixed rotation inside the OAK-D Pro
      housing that does not line up with "camera level" - field testing
      found ~80 deg of raw pitch at genuine standstill, not ~0. This is
      what was actually causing the shift to sit pinned at
      +-max_pitch_shift_deg's clamp even on a level chassis - NOT an
      axis or sign problem, a missing zero-calibration. To (re)capture
      it: rest the robot on a GENUINELY level surface (check with an
      actual level/phone app, not by eye), read pitch_raw_deg off the
      verbose log or view_obstacle.py's "IMU raw" HUD line, and set that
      reading as pitch_offset_deg. It is subtracted from every raw pitch
      before anything else touches it (see on_orientation_list). This is
      a plain scalar offset, not a full rotation correction - since
      BNO08x roll/pitch are gravity-referenced (see module docstring's
      IMU section), it should hold regardless of which way the robot is
      facing (yaw), but does NOT account for cross-talk from simultaneous
      large roll (e.g. driving across a side-slope). Watch pitch_deg
      while rolling the chassis on purpose if that turns out to matter -
      if pitch swings noticeably with pure roll and no real pitch change,
      the fix is a full reference-quaternion delta instead of a scalar
      offset, not a bigger/smaller number here.
    - pitch_shift_sign: defaults to +1, currently -1. Derived (not yet
      field-verified with the shift re-enabled) from the hand-tilt
      result: pitch_deg DECREASES as the nose lifts, and for the window
      to move the correct direction (down/toward the ramp) as pitch
      physically increases, pitch_shift_sign must be negative when pitch
      itself moves opposite to physical nose-up rotation - which is the
      case here. Re-verify once max_pitch_shift_deg is back above 0: if
      the window still moves toward sky instead of toward the ramp when
      you tilt the robot nose up, flip this sign.
    - max_pitch_error_deg: defaults to 15. This is the BNO08x's own
      estimated orientation error (rotationVectorAccuracy, in radians -
      LOWER is better despite the name; see on_orientation_list for why
      that's worth double-checking rather than assuming), NOT the
      separate 0-3 UNRELIABLE..HIGH categorical accuracy enum. 15 deg is
      an unverified starting point - log the raw error_rad values on
      real rough ground (degrees = easier to eyeball) and tighten or
      loosen from there. Watch how often "keeping last trusted pitch"
      logs before tightening further, since a threshold too strict just
      stops the window moving at all rather than making it move
      correctly.

  If pitch ever pushes a window past the top/bottom of the frame, it
  is clamped to stay in-bounds rather than shrinking or wrapping.

  --- Fixed camera-to-chassis mount tilt (camera_tilt_deg) ---

  The pitch-compensation above only tracks CHANGES in chassis pitch
  reported by the IMU at runtime (ramps, bumps) - it starts from
  `rows`/`ground_rows` as given, implicitly assuming the camera looks
  perfectly horizontal when the chassis itself reads 0 pitch. If the
  camera is physically mounted at a fixed upward (or downward) angle
  relative to the chassis, that assumption is wrong even on flat
  ground with the chassis level, and `rows`/`ground_rows` need to
  already be positioned for that permanent tilt.

  Concretely: for a level camera, the horizon sits at exactly the
  vertical center of the frame (img_height/2), regardless of mount
  height. Tilting the camera up by camera_tilt_deg moves the horizon
  DOWN (toward larger row numbers) by camera_tilt_deg worth of pixels
  - so a `rows` window whose top edge was chosen assuming a level
  camera can end up straddling the horizon once the real fixed tilt is
  accounted for, and a window that's up in the sky/far background most
  of the time reads mostly-invalid -> falls back to the centre zone's
  0.0 fail-safe ("assume the worst") -> the robot reads a permanent
  false "obstacle" immediately ahead and won't drive. If you see this
  (obstacle_zones center pinned near 0 with nothing actually in front
  of the robot), check camera_tilt_deg and the `rows` top edge first.

  camera_tilt_deg (default 0, degrees, positive = tilted up) is applied
  as an ADDITIONAL shift on top of the dynamic IMU-pitch shift above,
  every frame, unconditionally (it's a fixed mechanical fact, not
  something that needs the smoothing/threshold/clamp logic that exists
  to filter noisy or bump-transient IMU readings). Unlike
  pitch_shift_sign, its sign does NOT need field calibration/flipping:
  "tilt up moves the window down" falls directly out of standard image
  projection geometry (row 0 = top, increasing downward - true for
  every image library involved here), not out of the IMU's own,
  otherwise-unverified sign convention. Still worth a sanity check
  against a real depth viewer once you can - the shift MAGNITUDE also
  depends on vertical_fov_deg being right for your camera variant.

  Because `rows`/`ground_rows` given in config are now expected to
  represent the level-camera baseline (with camera_tilt_deg making up
  the difference at runtime), if your existing values were already
  hand-tuned by eye against the real (already-tilted) camera, applying
  a nonzero camera_tilt_deg on top will shift them further than before
  - re-verify against a depth viewer rather than assuming old numbers
  still apply unchanged.

  --- Startup: robot powers on already tilted ---

  self.smoothed_pitch (the EMA of dynamic IMU pitch, see above) starts
  at 0 and only snaps to the very first accuracy-passing 'orientation_list'
  reading it gets, after which it settles normally via the slow EMA (pitch_smoothing_
  alpha is deliberately slow - around a 1-2s time constant at typical
  depth fps - specifically so a single bump/jolt can't yank the window
  around). Without that snap, if the chassis happens to already be
  pitched at power-on (resting on a ramp, one side on a curb, etc.),
  the window would start from the WRONG (level-assumed) position and
  only ease into the correct one over the next several seconds - i.e.
  exactly the same false-"obstacle"-immediately-ahead failure mode
  described above for camera_tilt_deg, just transient instead of
  permanent, and easy to misdiagnose as a camera_tilt_deg problem since
  it looks the same from the printed line. The snap only fixes the
  startup transient; camera_tilt_deg above is still required for the
  permanent fixed-mount component even after it settles.

  --- Ground / drop-off (staircase, ledge) check ---

  Same depth frame, opposite polarity: a dedicated ground_rows/
  ground_cols band, positioned to look at the floor immediately ahead
  of the robot, is expected to read a valid, fairly close distance on
  safe ground. If it comes back mostly invalid (no return - looking
  into open space past an edge) OR reads farther than max_ground_dist
  (the floor receding away, e.g. down a staircase), that is published
  as ground_hazard = True. This is the opposite of the obstacle zones:
  there, "too close" is bad; here, "too far / no floor where one is
  expected" is bad.

  A wall or object pressed close enough to the camera can ALSO blank
  out the whole frame (including the ground band) at extreme close
  range - that looks identical to "the floor is missing" but is a
  very-close-obstacle event the regular center/obstacle logic already
  handles, and can recover from by backing up (a ground-hazard freeze
  cannot, since nothing changes while frozen). So a bad ground reading
  is only reported as a hazard when the center zone does NOT already
  show something within near_object_suppress_dist - otherwise it's
  suppressed and left to the obstacle-avoidance state machine instead.

  IMPORTANT calibration trap: your camera's own stereo threshold
  filter already zeroes out anything closer than its configured
  minimum range (e.g. 200mm) as invalid. If ground_rows/ground_cols
  happens to sit low enough in the frame that the true floor right
  under the robot normally falls inside that permanent near-field
  dead zone, this check will read "invalid" on completely ordinary
  flat ground - indistinguishable from a real drop-off. ground_rows
  MUST be positioned, using your own depth viewer, above that dead
  zone, where level ground gives a valid reading in normal operation.
  There is no way to pick a universally-correct default for this
  without knowing your mount height/tilt - the numbers below are a
  starting point to look at, not a calibrated value.

  Recommended rollout: run with the consuming node's hard-stop
  reaction disabled first, log ground_valid_frac/ground_dist across
  normal flat ground, bumps, and ramps, pick real thresholds from
  that, THEN enable the stop behaviour.

  This module only detects and reports ground_hazard (a raw per-frame
  boolean, no debouncing here - the same close_confirm-style streak
  pattern already used for obstacles lives in the consuming node
  instead, for consistency).
"""
import datetime
import math
from collections import deque

import cv2
import numpy as np

from osgar.node import Node


class ObstacleDetector3DZones(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('obstacle_zones', 'ground_hazard', 'depth_profile')

        self.base_rows = tuple(config.get('rows', [200, 300]))
        self.center_cols = tuple(config.get('center_cols', [200, 460]))
        self.left_cols = tuple(config.get('left_cols', [0, 150]))
        self.right_cols = tuple(config.get('right_cols', [490, 640]))

        # depth_profile (see module docstring, "free-space profile" section)
        # - a finer-grained N-bin distance scan across free_space_cols,
        # published purely as additional data. left_cols[0]/right_cols[1]
        # as the default span reuses whatever lateral field the L/C/R
        # zones already consider relevant, rather than introducing a
        # fourth independently-tuned column range.
        self.free_space_cols = tuple(config.get('free_space_cols', [self.left_cols[0], self.right_cols[1]]))
        self.free_space_bins = config.get('free_space_bins', 9)
        # see _sanitize_profile - meters a bin must read farther than
        # BOTH its immediate neighbors before being treated as an
        # isolated (likely artifact) reading rather than a real opening.
        # 0 disables this.
        self.profile_isolated_bin_margin = config.get('profile_isolated_bin_margin', 0.5)

        self.percentile = config.get('percentile', 5)  # 0 = classic min, like the original
        self.min_valid_frac = config.get('min_valid_frac', 0.05)

        # --- minimum GENUINELY MEASURED data (see _dist) ---
        # oak_camera_v3's far-mask rewrites invalid pixels in the upper
        # frame to far_fill_value_mm so that sky reads as "far" instead of
        # tripping min_valid_frac's fail-safe and freezing the robot. That
        # remap is necessary and stays. Its flaw is that a window can then
        # pass min_valid_frac on fill alone - the fill counts as valid -
        # so a window containing NO measurement whatsoever reports a
        # confident large distance.
        #
        # Field case (2026-08-29 run 125548, frontal collision at
        # 0.46m/s): an obstacle closing on the camera progressively
        # stopped returning stereo - real pixels in the centre window went
        # 51% -> 39% -> 8.8% -> 0.0% over one second - and once at zero the
        # window was 100% fill and read 15.00m. Not the far-mask erasing a
        # visible obstacle; the far-mask supplying confidence where there
        # was no longer any data at all.
        #
        # min_real_frac requires a floor of genuinely measured pixels
        # before any reading is trusted. Deliberately LOW, because the
        # legitimate sky case does not come anywhere near it: measured
        # over healthy Stromovka runs the centre window is 44-55% real
        # ground pixels (it sits below the horizon by design), and frames
        # under 2% real occur in only 0.8-2.8% of a healthy run - versus
        # 76.3% of the run where the camera genuinely saw nothing. So this
        # does NOT re-break the "sky above the horizon makes the robot
        # refuse to drive" case the far-mask exists to solve.
        # 0 disables the check.
        # Applied to the depth_profile BINS, whose consumer treats an
        # unknown bin as unknown (never as an obstacle) and which is what
        # the blind detector reads - so tightening it here costs nothing
        # and is what lets "the camera sees nothing anywhere" be noticed.
        self.min_real_frac = config.get('min_real_frac', 0.02)
        # Applied to the L/C/R ZONES. Defaults to 0 (off) DELIBERATELY,
        # unlike the profile above, because these feed the hard-stop and
        # avoidance triggers and the same sensor condition means different
        # things in different places. Measured across frames where the
        # centre window held no real pixels at all:
        #     doorway run  - a side zone still saw real data 52.9% of the
        #                    time; nothing anywhere only 47.1%
        #     crash run    - a side saw real data 1.7%; nothing anywhere
        #                    92.3%
        # So a blank centre alone does NOT identify the dangerous case,
        # and rejecting on it re-decides carefully tuned doorway
        # behaviour: switching this on changed 31 of 259 doorway frames
        # and up to 77deg of commanded steering. The dangerous case is
        # "nothing anywhere", which the blind detector catches through the
        # profile and ground band instead - at no cost to the zones.
        # Raise this only if you intend to re-tune close-quarters
        # behaviour with it on.
        self.min_real_frac_zones = config.get('min_real_frac_zones', 0.0)
        # must match the oak module's depth_far_mask_value_mm
        self.far_fill_value_mm = config.get('far_fill_value_mm', 15000)

        # --- how hard the centre fail-safe bites (item: fail-safe level) ---
        # What the CENTRE zone reports when its window cannot be trusted -
        # too little valid data (min_valid_frac) or too little genuinely
        # measured data (min_real_frac). The sides already report None
        # ("unknown", treated as far) and are unaffected.
        #
        #   0.0  - an obstacle at zero distance. Any consumer stops
        #          immediately. Maximum safety, the original behaviour.
        #   1.5  - "assume this much room". Above every threshold the
        #          consumer actually uses at Matty's speeds (turning_dist
        #          settles near 0.9), so missing data stops nothing - but
        #          it still feeds the adaptive-speed calculation, so the
        #          robot slows rather than charging blind.
        #   >3.0 - above turning_dist's ceiling: missing data produces no
        #          reaction of any kind.
        #
        # This is a genuine risk dial, not a tuning parameter. Raising it
        # trades away exactly the protection that min_real_frac above
        # exists to provide: in the 2026-08-29 run 125548 collision, the
        # centre window lost all real pixels one second before impact, and
        # it is this fail value that decides whether that second is spent
        # stopping or driving. Whoever raises it is accepting that
        # outcome deliberately.
        #
        # Consumers must NOT test for 0.0 to detect "no data" once this is
        # configurable - on_depth publishes the fact separately, see below.
        self.center_fail_dist_m = config.get('center_fail_dist_m', 0.0)

        # --- context-dependent fail-safe (open ground vs tight quarters) ---
        # One fail distance cannot serve both. Relaxing it stops the robot
        # from braking on missing data in the open, which is what a
        # competition run wants - but in tight quarters the SAME relaxation
        # silently removes most of the avoidance, because there the
        # fail-safe is not a rare safety net, it is doing the steering.
        # Measured on the 2026-08-29 park run (poles), comparing fail 0.0
        # against 1.2 everywhere: 595 commands changed and avoidance
        # episodes fell 59 -> 13, with 64.5% of those changes happening
        # while something had recently been measured within 2m (33.9%
        # within 1m) - i.e. squarely in the tight passages, not in the open
        # stretches the relaxation was meant for.
        #
        # So the fail distance is chosen per frame from how enclosed the
        # robot has recently been: the smallest GENUINELY MEASURED zone
        # distance over the last center_fail_recent_window_sec. That is
        # history, not the current frame - the whole problem is that the
        # current frame has no usable data, and the side zones are no help
        # either (in that same run, 98.3% of centre-fail frames had both
        # sides far/unknown too, making a doorway look exactly like open
        # ground).
        #
        # center_fail_dist_open_m defaults to center_fail_dist_m, i.e. no
        # context switching at all unless explicitly configured.
        self.center_fail_dist_open_m = config.get('center_fail_dist_open_m',
                                                  self.center_fail_dist_m)
        self.center_fail_open_recent_dist_m = config.get('center_fail_open_recent_dist_m', 2.0)
        self.center_fail_recent_window = datetime.timedelta(
            seconds=config.get('center_fail_recent_window_sec', 5.0))
        self._recent_measured = deque()  # (time, smallest measured zone distance)

        # --- synthetic far-fill cross-check (see _sanitize_zones) ---
        # oak_camera_v3.py's depth_far_mask_* remaps invalid pixels in the
        # UPPER part of the frame to a large constant (depth_far_mask_value_mm,
        # 15000 by default) so that sky / genuinely-out-of-range background
        # reads as "far" instead of tripping min_valid_frac's fail-safe and
        # freezing the robot. That remap is necessary and stays - but its
        # "is this a real blackout?" gate is evaluated over the WHOLE masked
        # region at once, while the fill is applied per-pixel. So a single
        # zone window that is locally blind (a low-texture or sunlit wall
        # close enough to kill the stereo match) still gets filled, as long
        # as the rest of the region has enough valid pixels to keep the
        # global gate happy. Field logs (2026-08-20, tight indoor passage)
        # show exactly this: zone windows 51-78% synthetic fill, ~1% real
        # measurement, reported as a confident 15.00m "clear" - while the
        # neighbouring zones were reading 0.4-0.8m.
        #
        # far_fill_suspect_m: a reading at/above this is treated as "this is
        # the synthetic fill constant, not a measurement". Keep it just under
        # the oak module's depth_far_mask_value_mm/1000 (15.0 -> 13.5).
        # far_fill_contradiction_m: another zone must have MEASURED something
        # closer than this for the suspect reading to be downgraded.
        # Set far_fill_suspect_m to 0 to disable this entirely.
        self.far_fill_suspect_m = config.get('far_fill_suspect_m', 13.5)
        self.far_fill_contradiction_m = config.get('far_fill_contradiction_m', 2.0)

        # Software stand-in for the OAK's own on-device SpeckleFilter -
        # deliberately NOT the on-device filter itself, which has been
        # observed to run the camera out of memory on this hardware and
        # must stay disabled in the oak module's own config. Runs here on
        # the host instead, at negligible cost given how small each zone/
        # bin window is. Strips small isolated blobs of "valid" (nonzero)
        # pixels via binary erosion before min_valid_frac/percentile ever
        # see them - see _valid_mask(). This targets a specific failure
        # mode percentile/min_valid_frac alone do NOT catch: a window
        # that is mostly correctly-invalid (out of stereo range - e.g.
        # pressed close against a glossy/low-texture surface, or an
        # irregular surface like a wire-mesh gabion wall where only a
        # sliver of pixels get a clean match) but has a small sliver of
        # spuriously-VALID pixels reading an implausibly FAR distance.
        # With only that sliver counted, mask.mean() can clear even a
        # generous min_valid_frac, and percentile-5 of an all-far sliver
        # is still far - the window gets reported as confidently open
        # when it's actually a near-invalid wall. A lone sliver narrower
        # than speckle_erode_px in any direction is erased entirely by
        # the erosion and correctly falls back to "unknown" instead. A
        # genuinely large, spatially coherent valid region (a real wall/
        # floor/ground reading) survives erosion basically intact - only
        # its outer speckle_erode_px//2-pixel border shrinks away.
        # speckle_erode_px=0 disables this (mask used as-is, old
        # behaviour). Not field-calibrated - a real object narrower than
        # this in the depth image (a thin, distant pole) would also get
        # erased; watch ground_valid_frac/zone readings in the field
        # before tuning it down.
        speckle_erode_px = config.get('speckle_erode_px', 3)
        self._erode_kernel = (np.ones((speckle_erode_px, speckle_erode_px), np.uint8)
                               if speckle_erode_px > 0 else None)

        # Asymmetric spatial fill - a deliberately one-sided stand-in for
        # the OAK's own SpatialFilter (also kept off-device, same memory
        # reason as SpeckleFilter above). A normal spatial filter fills
        # an invalid pixel with whatever's nearby, near OR far - which is
        # exactly what made a specular-reflection or wire-mesh false-far
        # sliver dangerous (see speckle_erode_px above): propagating that
        # into a mostly-invalid, actually-close window made it read as
        # confidently open. This filter can only ever ADD support for
        # "something is close nearby" - an invalid pixel is filled with
        # the NEAREST reading among its valid neighbors, but ONLY if that
        # neighbor is itself no farther than spatial_fill_max_dist_mm.
        # A far reading is never a fill source, full stop, regardless of
        # whether it's the "nearest" (in pixel-distance, not depth) valid
        # neighbor around - so this cannot recreate the false-far-sliver
        # failure mode, in either run order relative to the erosion above.
        #
        # Deliberately run BEFORE erosion (see _process()), not after -
        # the field case this targets is a physically thin object (a
        # table leg) whose true depth return is naturally sparse: a few
        # scattered valid-but-isolated near pixels, each individually
        # smaller than speckle_erode_px and so erased by erosion on its
        # own. Filling first bridges those scattered near pixels into one
        # connected-enough blob that the erosion pass no longer wipes out
        # entirely. Running erosion first would erase the leg's sparse
        # signal before fill ever got a chance to bridge it - fixing
        # nothing. The near-source-only restriction above is what makes
        # fill-before-erode safe to do despite the erosion module's
        # original false-far-sliver motivation.
        #
        # spatial_fill_px=0 disables this (old behaviour: erosion only).
        # Neither value is field-calibrated - watch zone/depth_profile
        # readings against a real thin object AND a real glossy/mesh
        # surface before trusting either default.
        spatial_fill_px = config.get('spatial_fill_px', 5)
        self._fill_kernel = (np.ones((spatial_fill_px, spatial_fill_px), np.float32)
                              if spatial_fill_px > 0 else None)
        self.spatial_fill_max_dist_mm = config.get('spatial_fill_max_dist_mm', 2000)
        self._FILL_SENTINEL = np.float32(1e6)  # far larger than any real/clipped depth (mm)

        # pitch-compensated row window (see module docstring)
        self.vertical_fov_deg = config.get('vertical_fov_deg', 55)
        self.max_pitch_shift_deg = config.get('max_pitch_shift_deg', 20)
        self.pitch_shift_sign = config.get('pitch_shift_sign', 1)
        self.pitch_smoothing_alpha = config.get('pitch_smoothing_alpha', 0.05)
        self.min_shift_change_px = config.get('min_shift_change_px', 12)
        self.camera_tilt_deg = config.get('camera_tilt_deg', 0)  # fixed mount tilt, see docstring
        # The OAK's own IMU die is mounted at some fixed rotation inside
        # the housing that does NOT line up with "camera level" - field
        # testing found it reads ~80 deg of pitch at genuine standstill,
        # not ~0. This is a constant, additive calibration offset
        # (subtracted from every raw pitch reading below, in
        # on_orientation_list), NOT a sign or axis problem - confirmed by
        # hand-tilt testing: roll/pitch/yaw each track the correct
        # physical motion (rotating the camera moves roll_deg, tilting it
        # moves pitch_deg monotonically, yaw_deg tracks heading), just
        # pitch_deg's zero point is offset by this fixed amount. See the
        # module docstring for how to (re)capture this value.
        self.pitch_offset_rad = math.radians(config.get('pitch_offset_deg', 0))
        # Physical-plausibility backstop, separate from max_pitch_error_deg:
        # field testing (2026-08) caught a sharp turn producing roll=-64deg
        # while pitch_error_deg (the BNO08x's own confidence field) stayed
        # at 0.0 the whole time - the sensor's own error estimate does NOT
        # catch this failure mode. Matty has no suspension and no
        # independent roll DOF, so real roll should stay well under this on
        # any terrain it can actually drive on; a reading beyond it means
        # the fused "down" vector is corrupted right now (classic failure
        # of accel+gyro-only fusion under lateral/centripetal acceleration,
        # e.g. mid-turn) - since pitch comes from the SAME quaternion, it's
        # equally untrustworthy this sample. 30 deg is a starting point,
        # not calibrated - loosen if genuine rough-terrain roll gets
        # rejected, tighten if turn-corrupted pitch still leaks through.
        self.max_plausible_roll_rad = math.radians(config.get('max_plausible_roll_deg', 30))
        # rotationVectorAccuracy (see on_orientation_list) is the BNO08x's
        # own ESTIMATED ORIENTATION ERROR, in radians - LOWER is better,
        # the opposite sense its name suggests. Samples above this are
        # dropped rather than fed into pitch/smoothed_pitch. 15 deg is a
        # starting point, NOT calibrated - log the raw values on real rough
        # ground first (see module docstring) and tighten/loosen from there.
        self.max_pitch_error_rad = math.radians(config.get('max_pitch_error_deg', 15))
        self.last_pitch_error_rad = None  # most recent sample's error, whether accepted or rejected - see on_orientation_list/verbose log, for tuning max_pitch_error_deg in the field
        self.pitch = 0.0            # radians, offset-corrected - see on_orientation_list. Feeds the actual shift math.
        self.last_raw_pitch = 0.0   # radians, UNCORRECTED (no pitch_offset_rad applied) - diagnostics/recalibration only, see verbose log
        self.last_roll = 0.0        # radians, diagnostics only - see on_orientation_list/verbose log
        self.last_yaw = 0.0         # radians, diagnostics only - see on_orientation_list/verbose log
        self.smoothed_pitch = 0.0   # radians, EMA of the above
        self._pitch_initialized = False  # see on_orientation_list - snap instead of EMA-ramp on the first reading
        self.applied_shift_px = 0.0
        self.rows = self.base_rows

        # depth_profile's own row window - defaults to base_rows (the
        # same band the L/C/R zones use) for backward compatibility, but
        # can be given a taller span via config. The free-space nudge
        # (tulak_obstacle.py's _free_space_steering) is a continuous,
        # every-frame signal with no confirm-frame debounce, so in
        # principle it should be the FIRST thing to react to an
        # approaching obstacle - but if the obstacle's near surface sits
        # just outside the (narrow, ~85px) center-zone row band at the
        # camera's current angle/distance, depth_profile stays "9/9 open"
        # right up until the obstacle finally sweeps into that same thin
        # band, by which point turning_dist is already close behind and
        # there's little lead time left. A taller free_space_rows band
        # gives the profile more vertical context to catch that surface
        # earlier, at the cost of being a coarser "is there something
        # somewhere in this taller slice" read rather than a precise
        # eye-level one - deliberately NOT the full frame height (that
        # would mix floor and sky into the same average and mean nothing).
        # NOT calibrated - a starting point to check against a real depth
        # viewer, same as every other row/col window in this module.
        self.free_space_base_rows = tuple(config.get('free_space_rows', self.base_rows))
        self.free_space_rows = self.free_space_base_rows

        # ground / drop-off check (see module docstring - NOT calibrated)
        self.ground_base_rows = tuple(config.get('ground_rows', [370, 395]))
        self.ground_cols = tuple(config.get('ground_cols', list(self.center_cols)))
        self.max_ground_dist = config.get('max_ground_dist', 1.0)
        self.near_object_suppress_dist = config.get('near_object_suppress_dist', 0.3)
        self.ground_rows = self.ground_base_rows

    @staticmethod
    def _quat_to_rpy(w, x, y, z):
        """Aerospace yaw-pitch-roll (ZYX) Euler angles, in radians, from
        the OAK's fused rotation quaternion (w=real, x=i, y=j, z=k).
        Returns (roll, pitch, yaw). Which of these three is actually
        Matty's physical nose-up/down pitch is NOT verified against how
        the IMU die sits inside the OAK-D Pro housing on this mount - all
        three are computed (not just pitch) specifically so a hand-tilt
        test can identify which one tracks which physical motion, rather
        than assuming "pitch" is correct. See on_orientation_list and the
        "Pitch-compensated row window" section of the module docstring.
        Symptom of picking the wrong one: the shift saturates at
        max_pitch_shift_deg's clamp even while the chassis is roughly
        level, because the axis being read is a large, unrelated angle
        (e.g. a ~90 deg mounting roll) rather than the small real pitch."""
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sin_pitch = max(-1.0, min(1.0, 2 * (w * y - z * x)))  # clamp - guards asin() right at +-90 deg
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def on_orientation_list(self, data):
        """data: batched samples from the OAK-D Pro's own onboard IMU (see
        oak_camera_v3.py's is_imu_enabled path), each entry
        [timestamp_sec, error_rad, i, j, k, real]. Using the camera's own
        IMU here (instead of the chassis ESP32 IMU this used to read from
        'rotation') means the pitch measured is the one that actually
        matters for this module - how the CAMERA is oriented, not the
        chassis - and each sample carries the BNO08x's own real-time
        estimate of how much to trust it, something the chassis IMU never
        exposed. That matters on a suspension-less chassis: a sample the
        sensor's own fusion algorithm already flagged as degraded
        (typically exactly the vibration/shock case) is dropped here
        before it ever reaches the smoothing EMA, instead of only being
        smoothed over after the fact.

        CAUTION (a real gotcha, not a hypothetical): despite depthai
        calling this field '...Accuracy', it is NOT the categorical
        IMUReport.Accuracy enum (0=UNRELIABLE..3=HIGH, higher=better) - it
        is IMUReportRotationVectorWAcc.rotationVectorAccuracy, the SH-2
        firmware's own ESTIMATED ORIENTATION ERROR IN RADIANS, where
        LOWER means better (near 0 = confident, growing toward pi =
        worthless) - see the CEVA BNO08x datasheet's 'Rotation Vector
        Accuracy Estimate'. Comparing it the wrong way round silently
        inverts this whole safeguard - accepting exactly the degraded
        samples it exists to reject.

        A whole batch landing above max_pitch_error_rad is not an error -
        the previously trusted pitch is simply kept for this cycle rather
        than pulled toward a low-confidence reading."""
        if data:
            # most recent sample's error, regardless of whether it passes
            # the threshold below - purely for the verbose log, so
            # max_pitch_error_deg can actually be tuned against real data
            self.last_pitch_error_rad = data[-1][1]
        for timestamp, error_rad, i, j, k, real in reversed(data):
            if error_rad > self.max_pitch_error_rad:
                continue
            roll, raw_pitch, yaw = self._quat_to_rpy(real, i, j, k)
            # last_roll/last_yaw/last_raw_pitch are updated here regardless
            # of the plausibility check below, so a corrupted reading is
            # still VISIBLE in the verbose log/HUD for debugging - only
            # self.pitch (the value that actually drives the shift) is
            # withheld from a sample that fails it.
            self.last_roll, self.last_raw_pitch, self.last_yaw = roll, raw_pitch, yaw
            if abs(roll) > self.max_plausible_roll_rad:
                # see max_plausible_roll_rad in __init__ - a corrupted
                # fused orientation (typically mid-turn) poisons pitch too,
                # since both come from the same quaternion. Try the next
                # (older) sample in this batch instead of using this one.
                continue
            # pitch_offset_rad corrects the fixed IMU-mount-vs-camera-level
            # misalignment (see __init__) - applied here, before smoothing/
            # shift/clamp, so everything downstream operates on a properly-
            # zeroed value same as before this offset existed.
            self.pitch = raw_pitch - self.pitch_offset_rad
            if not self._pitch_initialized:
                # snap instead of letting the EMA ramp up from 0 - see
                # "Startup: robot powers on already tilted" in the
                # module docstring for why the slow EMA is exactly
                # wrong here
                self.smoothed_pitch = self.pitch
                self._pitch_initialized = True
            return

    def _shift_window(self, base_rows, shift_px, img_height):
        r0, r1 = base_rows
        height = r1 - r0
        r0 = int(round(r0 + shift_px))
        r0 = max(0, min(img_height - height, r0))  # keep the window fully in-frame
        return r0, r0 + height

    def _maybe_update_rows(self, img_height):
        vfov_rad = math.radians(self.vertical_fov_deg)
        px_per_rad = img_height / vfov_rad

        # dynamic part: chassis pitch from the IMU (ramps, bumps). Sign
        # convention is whatever the IMU uses - pitch_shift_sign is the
        # field-calibration knob for that. Clamped so a transient/bad
        # reading can't swing the window further than max_pitch_shift_deg.
        raw_dynamic_shift_px = self.pitch_shift_sign * self.smoothed_pitch * px_per_rad
        max_dynamic_shift_px = math.radians(self.max_pitch_shift_deg) * px_per_rad
        dynamic_shift_px = max(-max_dynamic_shift_px, min(max_dynamic_shift_px, raw_dynamic_shift_px))

        # static part: fixed camera-to-chassis mount tilt (see module
        # docstring). A known mechanical constant, not the IMU's - always
        # applied in full, not subject to the dynamic clamp above (that
        # clamp bounds how far ramps/bumps can swing the window; it isn't
        # meant to limit the permanent baseline) or to pitch_shift_sign
        # (that's only for the IMU's own sign-convention uncertainty).
        static_shift_px = math.radians(self.camera_tilt_deg) * px_per_rad

        target_shift_px = dynamic_shift_px + static_shift_px

        if abs(target_shift_px - self.applied_shift_px) < self.min_shift_change_px:
            return  # not a real change - keep the current window, skip the log
        self.applied_shift_px = target_shift_px

        self.rows = self._shift_window(self.base_rows, target_shift_px, img_height)
        self.ground_rows = self._shift_window(self.ground_base_rows, target_shift_px, img_height)
        self.free_space_rows = self._shift_window(self.free_space_base_rows, target_shift_px, img_height)
        print(self.time, 'pitch %.1f deg (smoothed) + %.1f deg (fixed mount tilt), depth RoI shifted %+d px -> rows=%s ground_rows=%s free_space_rows=%s' % (
            math.degrees(self.smoothed_pitch), self.camera_tilt_deg, round(target_shift_px), self.rows, self.ground_rows, self.free_space_rows))

    def _fill_near(self, selection):
        """Fill invalid (0) pixels with the nearest reading among valid
        neighbors within spatial_fill_px, but ONLY where that neighbor is
        no farther than spatial_fill_max_dist_mm - see _fill_kernel in
        __init__ for the full rationale (asymmetric: can only manufacture
        support for "near", never "far"). Implemented as a grayscale
        erosion (= neighborhood MINIMUM) over an image where every pixel
        that doesn't qualify as a near source (invalid, OR valid but
        farther than the cutoff) is replaced with a sentinel far larger
        than any real depth - so the neighborhood minimum only "sees"
        qualifying near sources, and pixels with no such neighbor within
        reach keep the sentinel and are left untouched below."""
        if self._fill_kernel is None:
            return selection
        near_source = np.where((selection > 0) & (selection <= self.spatial_fill_max_dist_mm),
                                selection, self._FILL_SENTINEL).astype(np.float32)
        nearest_within_reach = cv2.erode(near_source, self._fill_kernel)
        fillable = (selection == 0) & (nearest_within_reach < self._FILL_SENTINEL)
        if not fillable.any():
            return selection
        filled = selection.copy()
        filled[fillable] = nearest_within_reach[fillable]
        return filled

    def _process(self, selection):
        """Runs the asymmetric near-only fill (_fill_near) THEN the
        speckle erosion, in that order - see _fill_kernel's docstring in
        __init__ for why this order is what actually helps a sparse thin
        object survive erosion, and why it's still safe against
        resurrecting a false-far-sliver despite running before erosion.
        Returns (selection_filled, valid_mask) - use them together
        (selection_filled[valid_mask]), not the original raw selection,
        so a filled-in near reading actually participates in the
        percentile distance below."""
        selection = self._fill_near(selection)
        mask = selection > 0
        if self._erode_kernel is not None:
            mask = cv2.erode(mask.astype(np.uint8), self._erode_kernel).astype(bool)
        return selection, mask

    def _track_recent_measured(self, *zones):
        """Remember the smallest GENUINELY MEASURED zone distance per frame,
        over a trailing window - see the context-dependent fail-safe notes
        in __init__. Fill values and fail substitutes are excluded: only
        something the sensor actually saw counts as evidence of being
        enclosed."""
        far_m = self.far_fill_value_mm / 1000.0
        measured = [d for d in zones if d is not None and 0.0 < d < far_m]
        if measured:
            self._recent_measured.append((self.time, min(measured)))
        while (self._recent_measured
               and self.time - self._recent_measured[0][0] > self.center_fail_recent_window):
            self._recent_measured.popleft()

    def _center_fail_value(self):
        """Which fail distance applies right now - the strict one in tight
        quarters, the relaxed one in the open. Nothing measured recently at
        all counts as OPEN: that is the camera seeing nothing rather than
        the robot being boxed in, and it is the blind detector's job (which
        looks at the profile and the ground band, independently of this)."""
        if self.center_fail_dist_open_m == self.center_fail_dist_m:
            return self.center_fail_dist_m          # context switching not configured
        if not self._recent_measured:
            return self.center_fail_dist_open_m
        recent = min(d for _, d in self._recent_measured)
        if recent < self.center_fail_open_recent_dist_m:
            return self.center_fail_dist_m          # enclosed - keep the strict fail-safe
        return self.center_fail_dist_open_m

    def _real_mask(self, selection, mask):
        """Valid pixels that are an actual measurement, i.e. excluding
        far-mask fill - see min_real_frac in __init__."""
        if not self.far_fill_value_mm:
            return mask
        return mask & (selection < self.far_fill_value_mm)

    def _dist(self, data, rows, cols, fail_value, min_real_frac=None):
        r0, r1 = rows
        c0, c1 = cols
        selection, mask = self._process(data[r0:r1, c0:c1])
        if mask.mean() < self.min_valid_frac:
            return fail_value
        if min_real_frac is None:
            min_real_frac = self.min_real_frac
        if min_real_frac > 0 and self._real_mask(selection, mask).mean() < min_real_frac:
            # window is essentially all synthetic fill - no measurement at
            # all behind it, so it must not be reported as a distance. See
            # min_real_frac in __init__.
            return fail_value
        return float(np.percentile(selection[mask], self.percentile) / 1000)

    def _sanitize_profile(self, profile):
        """A stereo mismatch on a flat, low-texture surface (a plain
        painted wall is a field-observed case) can produce a coherent,
        MEDIUM-sized false-far patch - wide enough to survive the
        pixel-level speckle erosion in _valid_mask (that only strips
        blobs narrower than speckle_erode_px), but this module has no
        other defense against it. The one thing that reliably tells it
        apart from a real opening: a gap actually wide enough for the
        robot (a doorway) spans MULTIPLE bins - it has at least one
        neighbor bin that agrees it's open. An artifact blob narrower
        than one bin doesn't. Downgrades a bin reading more than
        profile_isolated_bin_margin farther than BOTH immediate
        neighbors to the nearer of those two neighbors' own reading,
        instead of trusting the outlier outright. A genuine multi-bin
        gap is untouched - it always has at least one agreeing neighbor,
        so the "isolated on both sides" condition never fires for it.
        Only ever pulls a value DOWN (more cautious), never invents a
        closer reading than what's actually there, and never touches a
        bin next to a None (unknown) neighbor - nothing to corroborate
        OR contradict with there, so it's left alone rather than
        guessed at. Edge bins (index 0 and the last) have only one
        neighbor and are left untouched for the same reason - one-sided
        evidence is weaker, and the risk of suppressing a genuine
        edge-of-doorway reading is higher than for an interior bin with
        two neighbors to check against."""
        n = len(profile)
        if n < 3 or self.profile_isolated_bin_margin <= 0:
            return profile
        sanitized = list(profile)
        for i in range(1, n - 1):
            d, left, right = profile[i], profile[i - 1], profile[i + 1]
            if d is None or left is None or right is None:
                continue
            neighbor_max = max(left, right)
            if d > neighbor_max + self.profile_isolated_bin_margin:
                sanitized[i] = neighbor_max
        return sanitized

    def _sanitize_zones(self, left, center, right, center_failed=False):
        """Cross-zone version of _sanitize_profile's "an isolated far
        reading with no neighbour agreeing is probably an artifact" idea,
        applied to the L/C/R zones - the SAFETY-CRITICAL path, which until
        now had no equivalent protection at all (see far_fill_suspect_m in
        __init__ for the field case that motivated this).

        Deliberately narrow, because the thing it must NOT break is the
        entire reason the far-mask exists: open sky and genuinely
        far-away background must keep reading FAR, never "close obstacle".
        Two properties guarantee that:

          - it only ever fires on a reading at/above far_fill_suspect_m,
            i.e. one that IS the synthetic fill constant rather than
            anything the sensor actually measured;
          - it only fires when a DIFFERENT zone has MEASURED something
            closer than far_fill_contradiction_m, and the value it
            substitutes in is that measured distance. It cannot invent a
            close reading out of nothing.

        So the open-field case - every zone far, whether by real
        measurement or by fill - has nothing to contradict it and is
        passed through completely untouched. Only the physically
        implausible arrangement (one zone claiming 15m while a neighbour
        genuinely measures 0.4m, i.e. a 15m corridor exactly one zone
        wide between two close surfaces) gets pulled down.

        The 0.0 centre fail-safe is deliberately NOT counted as evidence
        of something close: it is itself a fail-value, not a measurement,
        and letting it drag the side zones to 0.0 would be exactly the
        "manufacture a phantom obstacle" behaviour this method promises
        not to do. Only ever makes a reading more cautious, never less."""
        zones = [left, center, right]
        if not self.far_fill_suspect_m or self.far_fill_suspect_m <= 0:
            return zones
        suspect = [d is not None and d >= self.far_fill_suspect_m for d in zones]
        if not any(suspect):
            return zones
        # a substituted centre fail distance is not a measurement and must
        # never be used as evidence that something is close (it would drag
        # a genuine far side down with it) - same reason 0.0 was excluded
        measured_close = [d for i, d in enumerate(zones)
                          if d is not None and 0.0 < d < self.far_fill_contradiction_m
                          and not (i == 1 and center_failed)]
        if not measured_close:
            return zones  # nothing contradicts - open sky / far background, leave alone
        nearest = min(measured_close)
        out = []
        for d, is_suspect in zip(zones, suspect):
            if is_suspect:
                print(self.time, 'zone reading %.2fm looks like far-mask fill but another zone '
                                  'measured %.2fm - downgrading (see _sanitize_zones)' % (d, nearest))
                out.append(nearest)
            else:
                out.append(d)
        return out

    def _depth_profile(self, data):
        """free_space_bins distances across free_space_cols x
        free_space_rows (own pitch-compensated row window - see __init__
        for why it's separate from, and normally taller than, self.rows)
        and same per-bin _dist() logic as the L/C/R zones, just finer-
        grained. None for a bin with too little valid data (same
        fail_value=None convention as left/right - "unknown", not
        "assume worst", since this is only ever used for a gradual
        steering nudge, never a hard stop). Passed through
        _sanitize_profile before returning - see that method."""
        c0, c1 = self.free_space_cols
        edges = np.linspace(c0, c1, self.free_space_bins + 1).astype(int)
        profile = [self._dist(data, self.free_space_rows, (edges[i], edges[i + 1]), fail_value=None)
                   for i in range(self.free_space_bins)]
        return self._sanitize_profile(profile)

    def _ground_reading(self, data):
        r0, r1 = self.ground_rows
        c0, c1 = self.ground_cols
        selection, mask = self._process(data[r0:r1, c0:c1])
        # count only genuine measurements: ground_rows normally sits below
        # the far-mask's row limit so no fill reaches here, but this stays
        # correct if that limit is ever raised - and the consumer now uses
        # this fraction as its "is the camera seeing anything at all?"
        # signal (see tulak_obstacle's _update_blind_state), which fill
        # would otherwise answer falsely
        mask = self._real_mask(selection, mask)
        valid_frac = float(mask.mean())
        if valid_frac < self.min_valid_frac:
            return valid_frac, None
        return valid_frac, float(np.percentile(selection[mask], self.percentile) / 1000)

    def on_depth(self, data):
        assert data.shape == (400, 640), data.shape
        self.smoothed_pitch += self.pitch_smoothing_alpha * (self.pitch - self.smoothed_pitch)
        self._maybe_update_rows(data.shape[0])

        # ask for None so an untrusted centre is DISTINGUISHABLE from a real
        # measurement, then substitute the configured fail distance - see
        # center_fail_dist_m. Testing the published number against 0.0 would
        # stop working the moment that value is changed.
        # zones use min_real_frac_zones (0 by default - see __init__ for why
        # they are deliberately NOT held to the profile's stricter bar)
        zmr = self.min_real_frac_zones
        center_raw = self._dist(data, self.rows, self.center_cols, fail_value=None, min_real_frac=zmr)
        center_failed = center_raw is None
        left = self._dist(data, self.rows, self.left_cols, fail_value=None, min_real_frac=zmr)
        right = self._dist(data, self.rows, self.right_cols, fail_value=None, min_real_frac=zmr)
        self._track_recent_measured(center_raw, left, right)
        center = self._center_fail_value() if center_failed else center_raw
        # drop synthetic far-mask fill that a neighbouring zone contradicts,
        # BEFORE anything downstream (hard stop, turn-side choice, and - via
        # the consumer's scan_samples - best-heading scoring) sees it
        left, center, right = self._sanitize_zones(left, center, right, center_failed)
        self.publish('obstacle_zones', [left, center, right])
        self.publish('depth_profile', self._depth_profile(data))

        ground_valid_frac, ground_dist = self._ground_reading(data)
        ground_bad = (ground_valid_frac < self.min_valid_frac) or \
                      (ground_dist is not None and ground_dist > self.max_ground_dist)
        # center_failed is the direct equivalent of the old "centre reads
        # 0.0" test - a blanked centre is exactly the pressed-against-an-
        # object case this suppression exists for, and it must keep working
        # whatever center_fail_dist_m is set to
        if ground_bad and (center_failed or center < self.near_object_suppress_dist):
            print(self.time, 'ground reading looks bad but center is very close (%.2fm) - '
                              'treating as a close obstacle, not a ground hazard' % center)
            ground_hazard = False
        else:
            ground_hazard = ground_bad
        self.publish('ground_hazard', [ground_hazard, round(ground_valid_frac, 3), ground_dist])

        if self.verbose:
            error_deg = round(math.degrees(self.last_pitch_error_rad), 1) if self.last_pitch_error_rad is not None else None
            # roll_deg/yaw_deg are printed alongside pitch_deg specifically
            # for the hand-tilt axis-mapping test - see _quat_to_rpy. If
            # pitch_deg stays pinned near max_pitch_shift_deg while the
            # chassis sits level, check whether roll_deg or yaw_deg is the
            # one actually swinging when you tilt the camera nose up/down.
            print(self.time, 'roll_deg', round(math.degrees(self.last_roll), 1),
                  'pitch_deg', round(math.degrees(self.smoothed_pitch), 1),
                  'pitch_raw_deg', round(math.degrees(self.last_raw_pitch), 1),
                  'pitch_offset_deg', round(math.degrees(self.pitch_offset_rad), 1),
                  'yaw_deg', round(math.degrees(self.last_yaw), 1),
                  'pitch_error_deg', error_deg, '(max', round(math.degrees(self.max_pitch_error_rad), 1), ')',
                  'rows', self.rows, 'zones', left, center, right,
                  'ground_valid_frac', round(ground_valid_frac, 2), 'ground_dist', ground_dist)

# vim: expandtab sw=4 ts=4
