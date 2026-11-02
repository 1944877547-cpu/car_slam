# RF2O source and local changes

- Upstream: https://github.com/MAPIRlab/rf2o_laser_odometry
- Branch: `ros2`
- Commit: `b38c68e46387b98845ecbfeb6660292f967a00d3`
- Original authors: Mariano Jaimez and Javier G. Monroy, MAPIR group.
- License: GPL v3; original `LICENSE` and algorithm source notices are retained.

The range-flow scan-matching algorithm is retained. The ROS adapter was updated for car_swarm:

- Wait for the laser-to-base TF before initializing; use a valid identity quaternion.
- Prime the first scan and restore both base and laser poses when initializing.
- Validate timestamps, scan geometry, ranges and finite match results before publishing.
- Accept clockwise N10 scans by reversing data internally, preserving the public acquisition order and timing; no motion compensation is applied.
- Correct the scan angular center using `angle_min`/`angle_increment` and the actual beam count.
- Publish base-frame planar velocities, including lateral motion and laser lever-arm compensation.
- Set configurable, positive twist covariance; the values are initial estimates requiring calibration.
- Keep TF publishing disabled by default so the EKF owns `odom -> base_link`.
- Exit if valid odometry stops, allowing the enclosing base launch to shut down its motor interface.
- Avoid zero-gradient weight division and repetitive per-scan INFO output.
- Update the package manifest for Humble and compile the algorithm once.

The laser odometry remains an estimate: sparse geometry, moving objects, large scan gaps and motion distortion can degrade it. It does not provide motor speed feedback.
