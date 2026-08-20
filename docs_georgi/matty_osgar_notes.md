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


Tulak Obstacle V0:

1. Working outdoors obstacle avoidance, stable, almost no FPs
2. Limited dead-end situations
3. GPS somehow working (needs 5m of driving for accurate estimate)
4. QR code reading working
5. No advanced navigation using OpenStreetMap
6. No waypoint placing
7. Doesn't work indoors anymore or in cramped spaces (aggressive overcorrection)

Tulak Obstacle V1:

1. Stabilized indoors
   1. Now Matty is the smooth operator.
   2. Was able to do full rounds around the apartment.
   3. Still overcorrecting (probably the 30 deg/s setting).
   4. When reversing, it correctly follows the same path, but then it often decides to repeat the same actions.
   5. GPS still unstable (mainly under trees)- the circle situation
   6. Maybe there is too much smoothness - Matty slows down too often
   7. Bumpers fixed, now they behave more like bumpers, actually stopping the actions.

Tulak Obstacle V2:

