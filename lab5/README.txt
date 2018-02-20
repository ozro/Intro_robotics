To run the code:

Load the configuration space map and the obstacle wavefront map that were used during the demo
>> load cspace_014wth5wp
>> load wavemap_014wth5wp

Call the path generation function with starting position [start_x start_y] and ending position [goal_x goal_y]
Both are measured in inches from the top left corner of the configuration space map.
The final flag is for the visualization of the A* path finding process. This is very slow so usually it is set to false.
>> path_gen(cspace014, wavemap, 0.14, [start_x start_y], [goal_x, goal_y], false);

Afterwards two figures will show, the first one being the path outputed to waypoints.txt.
The second figure shows the estimated path cost f(n) = g(n) + h(n) that A* was using to find the path.

Other functions used:
map.mat was created by hand as the baseline map of the physical obstacle course.

Create a configuration space map for a robot with radius r and save the output to cspace
>> load map
>> cspace = create_CSpace(map, r);

To create an obstacle wavefront map
>> wavemap = wavefront(cspace);