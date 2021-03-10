# Obstacle stop planner refine

## Purpose

- Sets a stop velocity when there are obstacle point clouds on the target path
- Sets a deceleration velocity when there is a vehicle in front of the ego-vehicle’s driving lane

## Inner-workings/Algorithms

### In the case of being obstacle point clouds on the target path

- (1) Generates a detection area considering the ego vehicle’s shape
- (2) Searches obstacle pointclouds in the detection area　
	- 2 step search: circular search + polygon search (to reduce calculation cost)
- (3) Plans a stop velocity
	- There are two types of stop margins
		- Basically, the ego vehicle stops *stop_margin_* (def: 5 m) before the obstacle
		- Considering a case that we would like to approach near the obstacle (e.g. crosswalk), if there is another stop plan between the obstacle and *stop_margin_*, the ego vehicle approaches to *min_behavior_stop_margin_* (def: 2 m) and stops.

![osp1](./img/osp1.svg "(1)") ![osp2](./img/osp2.svg "(2)") ![osp3](./img/osp3.svg "(3)")

### In the case of being a vehicle in front of the ego-vehicle’s driving lane

- (1) Creates a detection area considering the vehicle’s shape
- (2) Detects object pointclouds in the detection area
- (3) Infers a velocity for the vehicle
	- For velocity estimation, use DynamicObject when *use_object_to_estimate_vel* is true and use pointclouds when *use_pcl_to_estimate_vel* is true.
	- If both parameters are true, both inputs are used but DynamicObject is prioritized in the event that  both velocity estimations succeed. When both parameters are false, the vehicle’s velocity is not estimated.
	- If velocity estimation failed, then the obstacle stop planner (described on the previous slide) will be executed instead.
- (4) Plans & sets a deceleration velocity
	- As a default, the ego-vehicle will  follow the front vehicle, maintaining a distance of: *standard_dist* = *min_dist_stop* + *idling_distance* + *braking_distance* - *obj_braking_distance*
		- min_dist_stop = min_dist_standard[4.0m]
			- Margin(Constant)
		- idling_distance = standard_idling_time[0.5s] * v_e
			- Reaction distance of the ego-vehicle (v_e * t)
		- braking_distance = (-½) * v_e * v_e / min_standard_acceleration[-1.0m/ss] 
			- Braking distance of the ego-vehicle -  ½ * (v_e^2/*a_e)
		- obj_braking_distance =  (-½) * v_o * v_o / obstacle_min_standard_acceleration[-1.5m/ss]
			- Braking distance of the front vehicle - ½ * (v_o^2/a_o)
		- (v_e: velocity of the ego vehicle, v_o: velocity  of the front vehicle)
	- If the distance from the ego-vehicle to the front vehicle becomes less than the distance specified by the equation below, then a stop velocity before the front vehicle will be set *emergency_dist* = *em_min_dist_stop* + *em_idling_distance* + *em_braking_distance* - *em_obj_braking_distance*
		- em_min_dist_stop = min_dist_stop[4.0m]
		- em_idling_distance = emergency_stop_idling_time[0.5s] * v_e
		- em_braking_distance = (-½) * v_e * v_e / emergency_stop_acceleration[-5.0m/ss]
		- em_obj_braking_distance =  (-½) * v_o * v_o / emergency_stop_acceleration[-5.0m/ss]

![osp4](./img/osp4.svg)

## Inputs/Outputs

Input:

- trajectory : `autoware_planning_msgs::Trajectory`
- pointcloud : `sensor_msgs::PointCloud2`

Output:

- trajectory : `autoware_planning_msgs::Trajectory`

## Parameters




## Assumptions/Known limits


(Following items are optional)

## Error detection and handling

## Security considerations

## Performance characterization

## Future extensions/Unimplemented parts

## References/External links

