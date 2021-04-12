##1. Explain the functionality of what's provided in motion_planning.py and planning_utils.py
motion_planning.py is implemented with a event-driven based on a finite state automated machine code similar to backyard_flyer.py. A Planning state is added to automatically generate waypoints in motion_planning.py which is hardcoded in backyard_flyer.py.

now we Plan before ,Arm and Take off.  Arm is now the callback method and planning is then exectuted during the method that calculates the way point for the drone to follow.

The planning_utils.py 
contains the functions necessary to find the path with a herustic so its not greedy, prune it, and return it 
 . The A* graph search algorithm is similar to   Djikstra's shortest path algo but it's not greedy due to the heurstic.  We also create the grid and obstacles here. We then prune based on Breseham module to trim uneeded waypoints from path.

1. global home
The colliders.csv file already contains the home_lat and home_long position we can  read the top non-header line and get the global postion by using self.set_home_position() function.

2. Set your current local position
Current local position can be found using global_to_local with self.global_home .

3. Set grid start position from local position
This local position is in local-position and we use the grid to start!

4. Set grid goal position from geodetic coords
The grid goal position may be set with command line arguments.

python motion_planning.py --lon 500 --lat 500 --goal_alt 5
All these value have been made optional.

This allows you to control the target and altitude

5. Modify A* to include diagonal motion (or replace A* altogether)
provided A* algorithm allows only horizontal or diagnoal motion, so I included diagonal directions with a cost of sqrt(2) to optimize the search.

6. Cull waypoints
pruned using Breseham, to remove uncessary middle waypoints. 


1. Does it work?
Yes!

