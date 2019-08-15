# Fetch - Grasp, Pick, Hold, Shake and Place

This package launches gazebo world, Fetch robot, table and block on the table. The Fetch robot performs Grasp, Pick, Hold, Shake and Place behaviors on the block and record each behavior in a separate rosbag file.

<img src="pics/Fetch_grasp_pick_hold_shake_place.gif" align="middle">

`roslaunch fetch_tufts pickplace_playground.launch`
`roslaunch fetch_tufts pick_and_place_tufts.launch num_of_run:="natural number"`

Example:
`roslaunch sawyer_tufts pick_and_place_tufts.launch num_of_run:=2`
