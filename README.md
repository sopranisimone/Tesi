# Tesi

launch: ws ur_cable_manipulation demo_planner.launch

Force sensor callback handling inside SensorReader files. It handles sensor calibration, reference frame transformation, moving average and low pass filtering of the sensed forces.
ft_sensor plugin publishes on ft_sensor topic, subscriber has callback inside SensorReader files.
