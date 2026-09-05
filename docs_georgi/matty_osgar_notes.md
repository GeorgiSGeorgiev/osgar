# Available Matty-02 streams

* 'app.desired_steering'
* 'app.scan'
* 'app.set_leds'
* 'platform.esp_data'
* 'platform.emergency_stop'
* 'platform.pose2d'
* 'platform.bumpers_front'
* 'platform.bumpers_rear'
* 'platform.gps_serial'
* 'platform.joint_angle'
* 'platform.rpy'
* 'platform.rotation'
* 'platform.orientation'
* 'timer.tick'
* 'gps.position'
* 'gps.rel_position'
* 'gps.nmea_data'
* 'serial.raw'
* 'oak.depth'
* 'oak.color'
* 'oak.orientation_list'
* 'oak.detections'
* 'oak.left_im'
* 'oak.right_im'
* 'oak.depth_seq'
* 'oak.color_seq'
* 'oak.detections_seq'
* 'oak.left_im_seq'
* 'oak.right_im_seq'
* 'oak.nn_mask'

# Matty02 dimensions

* Distance between rear wheels: 23cm

* Box size: 17.5cmx22.5cm (lengthxwidth, width corresponds to the direction of the wheel axis)

* Between the two boxes there is 14.5cm space where sits also the joint right in the middle of these 14.5cm

* Distance between the front wheels axis and the rear wheels axis is 32cm

* The bumpers are 3cm to the front and the back from the boxes.

* Total length: 56.5cm

* The wheels add additional 6.5cm to each side

* Total width: 35.5cm

# Tulak Obstacle V0:

1. Working outdoors obstacle avoidance, stable, almost no FPs
2. Limited dead-end situations
3. GPS somehow working (needs 5m of driving for accurate estimate)
4. QR code reading working
5. No advanced navigation using OpenStreetMap
6. No waypoint placing
7. Doesn't work indoors anymore or in cramped spaces (aggressive overcorrection)

# Tulak Obstacle V1:

1. Stabilized indoors
   1. Now Matty is the smooth operator.
   2. Was able to do full rounds around the apartment.
   3. Still overcorrecting (probably the 30 deg/s setting).
   4. When reversing, it correctly follows the same path, but then it often decides to repeat the same actions.
   5. GPS still unstable (mainly under trees)- the circle situation
   6. Maybe there is too much smoothness - Matty slows down too often
   7. Bumpers fixed, now they behave more like bumpers, actually stopping the actions.

# Tulak Obstacle V2:

1. Stable smooth indoors and outdoors obstacle avoidance.
2. Still slightly unstable above horizon mask: flickers and triggers FP failsafe stops.
3. GPS reasonably stable.
4. Compass has less than 5.1 degrees error.
5. Too often slowing-down solved by adding weights to the side windows.
6. Added more randomness to the obstacle avoidance when stuck.
7. Obstacle avoidance overreaction minimized.
8. Still no stable usage of GPS nor compass: just as a PoC that is unstable.
9. GPS correction of compass not necessary. It introduced more instabilities as Matty didn't drive straight long enough.
10. Read GPS quality (not used yet).

# Tulak Obstacle V3

1. PoC of navigation algorithm using OSM.
2. Solved mask flickering completely.
3. Utilize GPS quality for improved navigational stability.
4. Fix compass offsets (small software calibration).