from PIL import Image
from math import sqrt
import heapq

# Map specific fixed values
OPEN_LAND = (248, 148, 18)
ROUGH_MEADOW = (255, 192, 0)
EASY_MOV_FOREST = (255, 255, 255)
SLOW_RUN_FOREST = (2, 208, 60)
WALK_FOREST = (2, 136, 40)
IMPASSABLE_VEG = (5, 73, 24)
LAKE_SWMP_MRSH = (0, 0, 255)
PAVED_RD = (71, 51, 3)
FOOTPATH = (0, 0, 0)
FOOTPATH_1 = (0, 0, 0)
TRAVERSED = (255,0,0)
TRAVERSED_OTHER = (255,0,255)
OUT_OF_BNDS = (205, 0, 101)
PX_X_LEN = 10.29
PX_Y_LEN = 7.55
PX_POS_TOP_LEFT = (0, 0)
PX_POS_TOP_RIGHT = (394, 0)
PX_POS_BOT_LEFT = (0, 499)
PX_POS_BOT_RIGHT = (394, 499)
PX_POS_LEFT_X = 0
PX_POS_RIGHT_X = 394
PX_POS_TOP_Y = 499
PX_POS_BOT_Y = 0
CONTROL = [(282,329), (292,310), (299,296), (306,275), (306,312), (317,241), (325,328), (339,224), (372,243)]
SPEED = {1:5, 2:4.5, 3:4, 4:3.5, 5:3, 6:2, 7:1, 8:0.5, 9:0.25, 10:0.1, 10000000000000:10000000000000}

#
#   The map class represents the map for orienteering
#   events at Mendon Ponds Park.
#
class Map:
    __slots__ = 'im','map_px','map_el','route_data','route_type','scoreo_time'
    def init(self):
        self.im = None
        self.map_px = None
        self.route_data = None
        self.route_type = None

    #
    #   This function loads the image file
    #   of map into a pixel array, so that
    #   we can access the pixel values for manipulation
    #
    def load_map_img(self):
        print('Loading map image of Mendon Ponds Park...')
        self.im = Image.open("terrain.png")
        if self.im.mode != "RGB":
            self.im = self.im.convert("RGB")
        self.map_px = self.im.load()

    #
    #   This function loads the elevation file
    #
    def load_map_data(self):
        print('Loading elevation details of Mendon Ponds Park...')
        self.map_el = []
        with open('mpp.txt') as file:
            lines = file.readlines()
        for line in lines:
            word = line.split()
            self.map_el.append(word)

    def display_img_array(self):
        im_size = self.im.size
        for rows in range(im_size[0]):
            for cols in range(im_size[1]):
                print(self.map_px[rows,cols])

    def display_data_array(self):
        for rows in range(len(self.map_el)):
            for cols in range(len(self.map_el[rows])):
                print(self.map_el[rows][cols])

    #
    #   This function loads the point data
    #   of a route from a route file
    #
    def load_route(self, route_name, self_or_other):
        print('Loading route details of', route_name, '...')
        self.route_data = []
        with open(route_name) as file:
            lines = file.readlines()
        self.route_type = (lines[0].split())[0]
        print('This is a',self.route_type,'event')
        if(self.route_type == 'Classic'):
            for idx in range(1, len(lines)):
                xy_values = tuple(lines[idx].split())
                self.route_data.append(xy_values)
            self.classic(self_or_other)
        elif(self.route_type == 'ScoreO'):
            self.scoreo_time = (lines[1].split())[0]
            print('Time limit:', self.scoreo_time)
            for idx in range(2, len(lines)):
                xy_values = tuple(lines[idx].split())
                self.route_data.append(xy_values)
            #self.scoreo(self_or_other)

    def display_route_data(self):
        print('The point details of the route are...')
        for points in self.route_data:
            print('x: ', points[0], 'y: ', points[1])

    #
    #   This function returns the speed quotient with which
    #   a terrain can be traversed.
    #
    def get_spd_as_per_terrain_type(self, terrain, is_other_person):
        if(terrain == OPEN_LAND): #Speed_rank = 1, running speed = 5 m/s
            if(is_other_person):
                return 1
            return 1
        elif(terrain == ROUGH_MEADOW): #Speed_rank = 6, speed = 2
            if (is_other_person):
                return 7
            return 6
        elif(terrain == EASY_MOV_FOREST): #Speed_rank = 4, speed = 3.5
            if (is_other_person):
                return 5
            return 4
        elif (terrain == SLOW_RUN_FOREST): #Speed_rank = 5, speed = 3
            if (is_other_person):
                return 6
            return 5
        elif (terrain == WALK_FOREST): #Speed_rank = 7, walking speed = 1 m/s
            if (is_other_person):
                return 8
            return 7
        elif (terrain == IMPASSABLE_VEG): #Speed_rank = 9, speed = 0.25
            if (is_other_person):
                return 10
            return 9
        elif (terrain == LAKE_SWMP_MRSH): #Speed_rank = 8, speed = 0.5
            if (is_other_person):
                return 10
            return 8
        elif (terrain == PAVED_RD): #Speed_rank = 2, speed = 4.5
            if (is_other_person):
                return 1
            return 2
        elif (terrain == FOOTPATH): #Speed_rank = 3, speed = 4
            if (is_other_person):
                return 1
            return 3
        elif (terrain == FOOTPATH_1): #Speed_rank = 3, speed = 4
            if (is_other_person):
                return 1
            return 3
        elif (terrain == OUT_OF_BNDS): #Speed_rank = 10, speed = 0.1
            return 10000000000000
        elif (terrain == TRAVERSED): #Speed_rank = 10, speed = 0.1
            return 9
        elif (terrain == TRAVERSED_OTHER): #Speed_rank = 10, speed = 0.1
            return 9

    #
    #   This function returns the euclidean distance
    #   between two points A and B
    #
    def get_euclidean_distance(self,A,B):
        dist = 0
        diff_x = B[0]-A[0]
        diff_y = B[1]-A[1]
        dist = sqrt((diff_x*diff_x) + (diff_y*diff_y))
        return dist

    def get_euclidean_distance_local(self, A, B):
        dist = 0
        diff_x = (B[0] - A[0]) * PX_X_LEN
        diff_y = (B[1] - A[1]) * PX_Y_LEN
        dist = sqrt((diff_x * diff_x) + (diff_y * diff_y))
        return dist
    #
    #   This is the heuristic function, which computes the
    #   estimated remaining cost of each state in the state space
    #   to the goal state
    #
    def heuristic(self, pt_A_location_X , pt_A_location_Y, pt_B_location_X, pt_B_location_Y, pq, dict, visited, parents, time_dict, is_other_person):

        # check in all 8 directions
        E_score = 0;
        NE_score = 0;
        N_score = 0;
        NW_score = 0;
        W_score = 0;
        SW_score = 0;
        S_score = 0;
        SE_score = 0;

        #print("X:",pt_A_location_X,"B:",pt_A_location_Y)
        # I. check the euclidean distance on all points to pt.B
        # II. check the terrain of all pts
        # III. check elevation of all pts

        # check East
        if pt_A_location_X != PX_POS_RIGHT_X:
            # get the distance between points in meters
            dist =  self.get_euclidean_distance((pt_A_location_X + 1, pt_A_location_Y),
                                                 (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X + 1, pt_A_location_Y],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X + 1, pt_A_location_Y),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist/SPEED[speed]
            time_dict[(pt_A_location_X + 1, pt_A_location_Y)] = time
            E_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X + 1][pt_A_location_Y])
            dict[E_score] = (pt_A_location_X + 1, pt_A_location_Y)
            if (pt_A_location_X + 1, pt_A_location_Y) not in visited:
                heapq.heappush(pq,E_score)
                visited.append((pt_A_location_X + 1, pt_A_location_Y))
                parents[dict[E_score]] = (pt_A_location_X, pt_A_location_Y)

        # check North East
        if((pt_A_location_X, pt_A_location_Y) != PX_POS_TOP_RIGHT) and\
                                        (pt_A_location_X != PX_POS_RIGHT_X) and\
                                ((pt_A_location_X, pt_A_location_Y) != PX_POS_BOT_RIGHT) and\
                        ((pt_A_location_X, pt_A_location_Y) != PX_POS_TOP_LEFT) and\
                (pt_A_location_Y != PX_POS_TOP_Y):
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X + 1, pt_A_location_Y + 1),
                                                  (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X + 1, pt_A_location_Y + 1],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X + 1, pt_A_location_Y + 1),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X + 1, pt_A_location_Y + 1)] = time
            NE_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X + 1][pt_A_location_Y + 1])
            dict[NE_score] = (pt_A_location_X + 1, pt_A_location_Y + 1)
            if (pt_A_location_X + 1, pt_A_location_Y + 1) not in visited:
                heapq.heappush(pq, NE_score)
                visited.append((pt_A_location_X + 1, pt_A_location_Y + 1))
                parents[dict[NE_score]] = (pt_A_location_X, pt_A_location_Y)

        # check North
        if pt_A_location_Y != PX_POS_TOP_Y:
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X, pt_A_location_Y + 1),
                                                 (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X, pt_A_location_Y + 1],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X, pt_A_location_Y + 1),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X, pt_A_location_Y + 1)] = time
            N_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X][pt_A_location_Y + 1])
            dict[N_score] = (pt_A_location_X, pt_A_location_Y + 1)
            if (pt_A_location_X, pt_A_location_Y + 1) not in visited:
                heapq.heappush(pq, N_score)
                visited.append((pt_A_location_X, pt_A_location_Y + 1))
                parents[dict[N_score]] = (pt_A_location_X, pt_A_location_Y)

        # check North West
        if((pt_A_location_X, pt_A_location_Y) != PX_POS_TOP_LEFT) and\
                (pt_A_location_X != PX_POS_LEFT_X) and\
                ((pt_A_location_X, pt_A_location_Y) != PX_POS_BOT_LEFT) and\
                ((pt_A_location_X, pt_A_location_Y) != PX_POS_TOP_RIGHT) and\
                (pt_A_location_Y != PX_POS_TOP_Y):
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X - 1, pt_A_location_Y + 1),
                                                  (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X - 1, pt_A_location_Y + 1],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X - 1, pt_A_location_Y + 1),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X - 1, pt_A_location_Y + 1)] = time
            NW_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X - 1][pt_A_location_Y + 1])
            dict[NW_score] = (pt_A_location_X - 1, pt_A_location_Y + 1)
            if (pt_A_location_X - 1, pt_A_location_Y + 1) not in visited:
                heapq.heappush(pq, NW_score)
                visited.append((pt_A_location_X - 1, pt_A_location_Y + 1))
                parents[dict[NW_score]] = (pt_A_location_X, pt_A_location_Y)

        # check West
        if pt_A_location_X != PX_POS_LEFT_X:
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X - 1, pt_A_location_Y),
                                                 (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X - 1, pt_A_location_Y],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X - 1, pt_A_location_Y),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X - 1, pt_A_location_Y)] = time
            W_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X - 1][pt_A_location_Y])
            dict[W_score] = (pt_A_location_X - 1, pt_A_location_Y)
            if (pt_A_location_X - 1, pt_A_location_Y) not in visited:
                heapq.heappush(pq, W_score)
                visited.append((pt_A_location_X - 1, pt_A_location_Y))
                parents[dict[W_score]] = (pt_A_location_X, pt_A_location_Y)

        # check South West
        if((pt_A_location_X != PX_POS_TOP_LEFT[0] & pt_A_location_Y != PX_POS_TOP_LEFT[1]) and
               (pt_A_location_X != PX_POS_BOT_LEFT[0] & pt_A_location_Y != PX_POS_BOT_LEFT[1]) and
               (pt_A_location_X != PX_POS_BOT_RIGHT[0] & pt_A_location_Y != PX_POS_BOT_RIGHT[1]) and
               (pt_A_location_X != PX_POS_LEFT_X) and (pt_A_location_Y != PX_POS_BOT_Y)):
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X - 1, pt_A_location_Y - 1),
                                                  (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X - 1, pt_A_location_Y - 1],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X - 1, pt_A_location_Y - 1),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X - 1, pt_A_location_Y - 1)] = time
            SW_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X - 1][pt_A_location_Y - 1])
            dict[SW_score] = (pt_A_location_X - 1, pt_A_location_Y - 1)
            if (pt_A_location_X - 1, pt_A_location_Y - 1) not in visited:
                heapq.heappush(pq, SW_score)
                visited.append((pt_A_location_X - 1, pt_A_location_Y - 1))
                parents[dict[SW_score]] = (pt_A_location_X, pt_A_location_Y)

        # check South
        if pt_A_location_Y != PX_POS_BOT_Y:
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X, pt_A_location_Y - 1),
                                                 (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X, pt_A_location_Y - 1],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X, pt_A_location_Y - 1),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X, pt_A_location_Y - 1)] = time
            S_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X + 1][pt_A_location_Y])
            dict[S_score] = (pt_A_location_X + 1, pt_A_location_Y)
            if (pt_A_location_X + 1, pt_A_location_Y) not in visited:
                heapq.heappush(pq, S_score)
                visited.append((pt_A_location_X + 1, pt_A_location_Y))
                parents[dict[S_score]] = (pt_A_location_X, pt_A_location_Y)

        # check South East
        if((pt_A_location_X, pt_A_location_Y) != PX_POS_BOT_LEFT) and\
                                        ((pt_A_location_X, pt_A_location_Y) != PX_POS_BOT_RIGHT) and\
                                ((pt_A_location_X, pt_A_location_Y) != PX_POS_TOP_RIGHT) and\
                        (pt_A_location_Y != PX_POS_BOT_Y) and (pt_A_location_X != PX_POS_RIGHT_X):
            # get the distance between points in meters
            dist = self.get_euclidean_distance((pt_A_location_X + 1, pt_A_location_Y - 1),
                                                  (pt_B_location_X, pt_B_location_Y))
            # get the speed on the terrain in meters/second
            speed = self.get_spd_as_per_terrain_type(self.map_px[pt_A_location_X + 1, pt_A_location_Y - 1],is_other_person)
            local_dist = self.get_euclidean_distance_local((pt_A_location_X + 1, pt_A_location_Y - 1),
                                                           (pt_A_location_X, pt_A_location_Y))
            time = local_dist / SPEED[speed]
            time_dict[(pt_A_location_X + 1, pt_A_location_Y - 1)] = time
            SE_score = dist + speed + float(self.map_el[pt_A_location_X][pt_A_location_Y]) - float(self.map_el[pt_A_location_X + 1][pt_A_location_Y - 1])
            dict[SE_score] = (pt_A_location_X + 1, pt_A_location_Y - 1)
            if (pt_A_location_X + 1, pt_A_location_Y - 1) not in visited:
                heapq.heappush(pq, SE_score)
                visited.append((pt_A_location_X + 1, pt_A_location_Y - 1))
                parents[dict[SE_score]] = (pt_A_location_X, pt_A_location_Y)

        heapq.heapify(pq)
        #print("dist: ", local_dist)
        #print("speed: ",speed)
        #print("time: ",time)

        return pq,dict,visited,parents,time_dict

    #
    #   This function is the implementation of
    #   A* search
    #
    def search(self, pt_A_location_X, pt_A_location_Y, pt_B_location_X, pt_B_location_Y, is_other_person):

        print('A to B')
        print('A x:',pt_A_location_X,'A y:',pt_A_location_Y)
        print('B x:', pt_B_location_X, 'B y:', pt_B_location_Y)
        total_cost = 0

        # A* Search : Optimal with admissible heuristic
        priority_q = []
        point_dict = {}
        time_dict = {}

        parents = {}
        visited = []
        total_cost = 0
        heapq.heappush(priority_q, 0)
        time_dict[(pt_A_location_X, pt_A_location_Y)] = 0
        point_dict[0] = (pt_A_location_X, pt_A_location_Y)
        visited.append((pt_A_location_X, pt_A_location_Y))
        while(len(priority_q) != []):
            state_score = heapq.heappop(priority_q)
            current = point_dict[state_score]
            total_cost = total_cost + time_dict[current]
            if current == (pt_B_location_X, pt_B_location_Y):
                print("..)")
                return parents, total_cost
            priority_q, point_dict, visited, parents, time_dict = self.heuristic(current[0], current[1], pt_B_location_X, pt_B_location_Y, priority_q, point_dict, visited, parents, time_dict, is_other_person)
        if(len(priority_q) == []):
            print("..(")
        print('\n')

    #
    #   This is the classic event
    #   where we have to visit each point
    #   in sequence
    #
    def classic(self,self_or_other):
        i = 0
        if(self_or_other == "s"):
            is_other_person = True
        else:
            is_other_person = False
        for line_id in range(0,len(self.route_data)-1):
            parents, total_cost = self.search(int((self.route_data[line_id])[0]), int((self.route_data[line_id])[1]), int((self.route_data[line_id+1])[0]), int((self.route_data[line_id+1])[1]), is_other_person)
            # Re-Draw the map with the path
            src = ((self.route_data[i])[0], (self.route_data[i])[1])
            dest = ((self.route_data[i + 1])[0], (self.route_data[i + 1])[1])
            for k in parents.keys():
                if k[0] == int(dest[0]) and k[1] == int(dest[1]):
                    prev = parents.get(k)
            while prev[0] != int(src[0]) or prev[1] != int(src[1]):
                if is_other_person:
                    self.map_px[int(prev[0]),int(prev[1])] = (255,0,0)
                else:
                    self.map_px[int(prev[0]), int(prev[1])] = (255, 0, 255)
                prev = parents[prev]
            i = i + 1
        self.im.show()

    #
    #   This is the score-o event
    #   where we have to visit as many points as possible
    #   from start to finish in a given amount of time
    #
    def scoreo(self):
        adj_list = []
        controls = []
        for line_id_current in range(0,len(self.route_data)):
            list_neighbors = []
            src = ((self.route_data[line_id_current])[0], (self.route_data[line_id_current])[1])
            controls.append(src)
            for line_id_neighbor in range(0,len(self.route_data)):
                dest = ((self.route_data[line_id_neighbor])[0], (self.route_data[line_id_neighbor])[1])
                parents, total_cost = self.search(int(src[0]),int(src[1]),int(dest[0]),int(dest[1]))
                list_neighbors.append(total_cost)
            adj_list.append(list_neighbors)

        time_limit = int(self.scoreo_time)

        src_dest = ((self.route_data[0])[0], (self.route_data[0])[1])
        max_controls_visited = 0
        plot_to_max_controls = {}
        for i in range(0,len(controls)):
            print(controls[i])

        for i in range(0,len(adj_list)):
            plot_points = []
            max_controls_visited = max_controls_visited + 1
            plot_points.append(controls[i])
            plot_to_max_controls[controls[i]] = plot_points
            for j in range(0,len(adj_list[i])):
                #print(adj_list[i][j])
                if(adj_list[i][j] < time_limit and adj_list[i][j] != 0):
                    total_cost = total_cost + adj_list[i][j]
                    max_controls_visited = max_controls_visited + 1
                    plot_points.append(controls[j])
                    plot_to_max_controls[controls[i]] = plot_points
                    if(time_limit - adj_list[j][i] == adj_list[i][j] - 10):
                        pass
                    for k in range(0,len(adj_list[j])):
                        if (adj_list[j][k] < time_limit and adj_list[j][k] != 0
                            and controls[k] not in plot_points):
                            total_cost = total_cost + adj_list[j][k]
                            max_controls_visited = max_controls_visited + 1
                            plot_points.append(controls[k])
                            plot_to_max_controls[controls[i]] = plot_points
                            if (time_limit - adj_list[j][i] == adj_list[i][j] - 10):
                                pass
                            for l in range(0, len(adj_list[k])):
                                if (adj_list[k][l] < time_limit and adj_list[k][l] != 0
                                    and controls[l] not in plot_points):
                                    total_cost = total_cost + adj_list[k][l]
                                    max_controls_visited = max_controls_visited + 1
                                    plot_points.append(controls[l])
                                    plot_to_max_controls[controls[i]] = plot_points
                                    if (time_limit - adj_list[j][i] == adj_list[i][j] - 10):
                                        pass



if __name__ == '__main__':
    m = Map()
    m.load_map_img()
    m.load_map_data()
    file = input("Enter the filename\n")
    self_or_other = input("Search for self(s) or other(o)\n")
    m.load_route(file,self_or_other)

