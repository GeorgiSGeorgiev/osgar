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
import math

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

        self.percentile = config.get('percentile', 5)  # 0 = classic min, like the original
        self.min_valid_frac = config.get('min_valid_frac', 0.05)

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
        print(self.time, 'pitch %.1f deg (smoothed) + %.1f deg (fixed mount tilt), depth RoI shifted %+d px -> rows=%s ground_rows=%s' % (
            math.degrees(self.smoothed_pitch), self.camera_tilt_deg, round(target_shift_px), self.rows, self.ground_rows))

    def _dist(self, data, rows, cols, fail_value):
        r0, r1 = rows
        c0, c1 = cols
        selection = data[r0:r1, c0:c1]
        mask = selection > 0
        if mask.mean() < self.min_valid_frac:
            return fail_value
        return float(np.percentile(selection[mask], self.percentile) / 1000)

    def _depth_profile(self, data):
        """free_space_bins distances across free_space_cols, same row
        window (self.rows - already pitch-compensated) and same per-bin
        _dist() logic as the L/C/R zones, just finer-grained. None for a
        bin with too little valid data (same fail_value=None convention
        as left/right - "unknown", not "assume worst", since this is only
        ever used for a gradual steering nudge, never a hard stop)."""
        c0, c1 = self.free_space_cols
        edges = np.linspace(c0, c1, self.free_space_bins + 1).astype(int)
        return [self._dist(data, self.rows, (edges[i], edges[i + 1]), fail_value=None)
                for i in range(self.free_space_bins)]

    def _ground_reading(self, data):
        r0, r1 = self.ground_rows
        c0, c1 = self.ground_cols
        selection = data[r0:r1, c0:c1]
        mask = selection > 0
        valid_frac = float(mask.mean())
        if valid_frac < self.min_valid_frac:
            return valid_frac, None
        return valid_frac, float(np.percentile(selection[mask], self.percentile) / 1000)

    def on_depth(self, data):
        assert data.shape == (400, 640), data.shape
        self.smoothed_pitch += self.pitch_smoothing_alpha * (self.pitch - self.smoothed_pitch)
        self._maybe_update_rows(data.shape[0])

        center = self._dist(data, self.rows, self.center_cols, fail_value=0.0)  # preserve old fail-safe
        left = self._dist(data, self.rows, self.left_cols, fail_value=None)
        right = self._dist(data, self.rows, self.right_cols, fail_value=None)
        self.publish('obstacle_zones', [left, center, right])
        self.publish('depth_profile', self._depth_profile(data))

        ground_valid_frac, ground_dist = self._ground_reading(data)
        ground_bad = (ground_valid_frac < self.min_valid_frac) or \
                      (ground_dist is not None and ground_dist > self.max_ground_dist)
        if ground_bad and center < self.near_object_suppress_dist:
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
