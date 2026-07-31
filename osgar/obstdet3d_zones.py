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

  --- Pitch-compensated row window ---

  `rows` (and `ground_rows`, see below) are now *base* windows that
  get shifted up/down each frame based on current pitch (from the
  platform's 'rotation' topic), using the mono camera's vertical FOV
  to convert pitch angle to a pixel shift. left_cols/right_cols/
  center_cols/ground_cols are NOT touched - pitch moves things
  vertically, not sideways.

  Two things keep this from reacting to every bump on a suspension-
  less chassis:
    - the pitch used for the shift is smoothed (pitch_smoothing_alpha,
      an exponential moving average) so a single-frame jolt barely
      moves it, while a sustained ramp climb does
    - the window only actually updates (and only logs) once the
      smoothed pitch implies a shift of at least min_shift_change_px
      pixels since the last update - small noise changes nothing

  When the window does move, it prints a line (unconditionally, not
  only under verbose) so you can see it happening from the OS console.

  This needs two things verified for your actual hardware/mount, same
  spirit as the "not calibrated" note on left/right columns above:
    - vertical_fov_deg: defaults to 55, the OV9282 mono VFOV Luxonis
      lists for the STANDARD (non-"W"/wide) OAK-D Pro.
    - pitch_shift_sign: defaults to +1. If the window moves toward sky
      instead of toward the ramp when you tilt the robot nose up,
      flip this to -1. Watch the printed 'rows' value while tilting
      the camera by hand to check.

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
  at 0 and only snaps to the very first 'rotation' reading it gets,
  after which it settles normally via the slow EMA (pitch_smoothing_
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
        bus.register('obstacle_zones', 'ground_hazard')

        self.base_rows = tuple(config.get('rows', [200, 300]))
        self.center_cols = tuple(config.get('center_cols', [200, 460]))
        self.left_cols = tuple(config.get('left_cols', [0, 150]))
        self.right_cols = tuple(config.get('right_cols', [490, 640]))

        self.percentile = config.get('percentile', 5)  # 0 = classic min, like the original
        self.min_valid_frac = config.get('min_valid_frac', 0.05)

        # pitch-compensated row window (see module docstring)
        self.vertical_fov_deg = config.get('vertical_fov_deg', 55)
        self.max_pitch_shift_deg = config.get('max_pitch_shift_deg', 20)
        self.pitch_shift_sign = config.get('pitch_shift_sign', 1)
        self.pitch_smoothing_alpha = config.get('pitch_smoothing_alpha', 0.05)
        self.min_shift_change_px = config.get('min_shift_change_px', 12)
        self.camera_tilt_deg = config.get('camera_tilt_deg', 0)  # fixed mount tilt, see docstring
        self.pitch = 0.0            # radians, latest raw sample from 'rotation'
        self.smoothed_pitch = 0.0   # radians, EMA of the above
        self._pitch_initialized = False  # see on_rotation - snap instead of EMA-ramp on the first reading
        self.applied_shift_px = 0.0
        self.rows = self.base_rows

        # ground / drop-off check (see module docstring - NOT calibrated)
        self.ground_base_rows = tuple(config.get('ground_rows', [370, 395]))
        self.ground_cols = tuple(config.get('ground_cols', list(self.center_cols)))
        self.max_ground_dist = config.get('max_ground_dist', 1.0)
        self.near_object_suppress_dist = config.get('near_object_suppress_dist', 0.3)
        self.ground_rows = self.ground_base_rows

    def on_rotation(self, data):
        # platform publishes [9000 - yaw, -pitch, roll], all in centidegrees -
        # data[1] is -pitch, whatever sign convention the platform's IMU uses
        self.pitch = math.radians(data[1] / 100.0)
        if not self._pitch_initialized:
            # snap instead of letting the EMA ramp up from 0 - see
            # "Startup: robot powers on already tilted" in the module
            # docstring for why the slow EMA is exactly wrong here
            self.smoothed_pitch = self.pitch
            self._pitch_initialized = True

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
            print(self.time, 'pitch_deg', round(math.degrees(self.smoothed_pitch), 1),
                  'rows', self.rows, 'zones', left, center, right,
                  'ground_valid_frac', round(ground_valid_frac, 2), 'ground_dist', ground_dist)

# vim: expandtab sw=4 ts=4
