import argparse
import time
import msgpack
from enum import Enum, auto
import numpy as np
import random
from planning_utils import a_star,prune_func, heuristic, create_grid

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection,lon=None,lat=None,alt=5):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.goal_alt=alt
        self.target_loc=(lon,lat)
        
        self.map_file = 'map/colliders.csv'
        random.seed()

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 10

        self.target_position[2] = TARGET_ALTITUDE

        # Done: read lat0, lon0 from colliders into floating point values
        def converter(s):
            l=str(s, 'utf-8').split(' ')
            d={l[0]:float(l[1])}
            return d
        
        data = np.genfromtxt(self.map_file, max_rows=1, delimiter=',', converters={0:converter,1:converter}, dtype=object, autostrip=True)
        print(data)
        global_home_position = dict()
        for d in data:
            global_home_position.update(d)
        # Done: set home position to (lon0, lat0, 0)
        self.set_home_position(global_home_position['lon0'],
                              global_home_position['lat0'],
                              0.0)

        pos_of_craft = global_to_local(self.global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(self.map_file, delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        grid_start=(int(pos_of_craft[0])-north_offset,int(pos_of_craft[1]) -east_offset)
        

        can_be_reached = False
        new=self.target_loc

        while not can_be_reached and new==(None,None):
            converage_radius = 150
            random_goal_coordinate = local_to_global([pos_of_craft[0]+random.uniform(-converage_radius, converage_radius),
                            pos_of_craft[1]+random.uniform(-converage_radius, converage_radius),
                            0.0], self.global_home)
            # goal_coordinate = [lat,lon,up]
            goal_coordinate = random_goal_coordinate
            goal_position = global_to_local(goal_coordinate, self.global_home)

            self.target_loc =(int(goal_position[0])-north_offset, int(goal_position[1])-east_offset)
            #print(grid[439][540])
            can_be_reached = grid[self.target_loc[0]][self.target_loc[1]] != 1
        
        print('global goal position {}'.format(goal_position))
        # Run A* to find a path from start to goal
        # Done: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, self.target_loc)
        start=time.time()
        path_unpruned, _ = a_star(grid, heuristic, grid_start, self.target_loc)
        print(time.time()-start)
        # Done: prune path to minimize number of waypoints
        pruned_path = prune_func(grid, path_unpruned)
        # visualize the path generated.Close plot window to proceed execution


        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, self.goal_alt, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--lon', type=int, help="Goal longitude",required=False)
    parser.add_argument('--lat', type=int, help="Goal latitude",required=False)
    parser.add_argument('--alt', default=5, type=int, help="Goal altitude",required=False)
    args = parser.parse_args()
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn,args.lon,args.lat,args.alt)
    time.sleep(1)

    drone.start()
