This data is collected from the CARLA simulator and is hopefully useful for determining a model
for how the controls/input (i.e. throttle and steering values on [-1, 1] or something) interact
with the state of the car (i.e. position, velocity, etc). The data is stored in multiple csv files,
each of which represent a complete run of the car around the track (except for pid_data_3, which
ran into a wall soon after starting).

The data includes:
	car position (x, y, z in world frame),
	car velocity (x, y, z as calculated in pid_controller.py in the LatPIDController.run_in_series() method),
	next waypoint position (x, y, z in world frame),
	vector from the car to the next waypoint (x, y, z taken to be the desired direction of motion),
	steering (a value on [-1, 1] that describes the steering control input)
	throttle (a value on [-1, 1] that describes the throttle control input)

The time between each measurement is assumed by the controller to be 0.03 seconds.

Some metadata on the data collected:
1: default K values 
2: slightly modified K values (similar behavior as 1)
3: submitted zero control for 20 steps out of every 100 steps in order to observe zero-input behavior.
	unfortunately, the car crashed into a wall shortly after starting
4: smaller K values for latitude control (reduced by a magnitude from 1)
5: smaller K values for both latitude and longitude control (reduced by a magnitude from 1).
	this resulted in throttle values that are not constantly 1, which is very nice.
6: K values from 5 but with larger Ki value for longitudinal control in an effort to make the car
	accelerate more when going uphill. This did not perform as desired. Instead the car had
	far more volatile speeds and touchier throttling during normal driving with no perceived
	improvement in uphill travel.