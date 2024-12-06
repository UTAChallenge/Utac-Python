#This is a template code. Please save it in a proper .py file.
import numpy as np
import time
import math
import rtmaps.types
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class

def detect_points_linear(array_in,bounding_box): # function we use to detect the points inside bounding boxes

    #这里获取的boundingbox是根据路径计算出前方两侧的坐标点之后进行过滤
    #google trad
    #The boundingbox obtained here is filtered after calculating the coordinate points on both sides of the front based on the path
    x_min = bounding_box[0,0] # extract the values of the box
    x_max = bounding_box[0,1]
    y_min = bounding_box[1,0]
    y_max = bounding_box[1,1]
    z_min = bounding_box[2,0]
    z_max = bounding_box[2,1]
    #变成刚好3的倍数，用于转制，reshape the xyz points to x,y,z for easier use
    #Google trad 
    #Becomes a multiple of exactly 3 for conversion, reshape the xyz points to x,y,z for easier use
    points = array_in.reshape(int(np.size(array_in)/3),3)

    # filter points inside the x bounds
    #Google trad 
    #Filter out points in the first column that are not between x_min and x_max
    filtered_points = points[(points[:,0]>x_min) & (points[:,0]<x_max)] #滤掉第一列中不在x_min到x_max之间的点
    # filter points inside the y bounds
    #Google trad 
    #Filter out points in the second column that are not between y_min and y_max
    filtered_points = filtered_points[(filtered_points[:,1]>y_min) & (filtered_points[:,1]<y_max)]#滤掉第二列中不在y_min到y_max中间的点
    # filter points inside the z bounds
    #Google trad
    #Filter out points in the third column that are not between z_min and z_max
    filtered_points = filtered_points[(filtered_points[:,2]>z_min) & (filtered_points[:,2]<z_max)]#滤掉第三列中不在z_min到z_max中间的点

    
    # points appearing that are inside the car 这个位置不太懂，可能是用来排除雷达本体或者车子？
    # 下面的例子相当于排除掉了{1.35<=x<=1.4,0.5<=y<=0.56,0.8<=z<=0.9}这一块小范围的云点
    #Google trad
    # points appearing that are inside the car I don’t understand this position, maybe it is used to exclude the radar itself or the car?
    # The following example is equivalent to excluding the small range of cloud points {1.35<=x<=1.4,0.5<=y<=0.56,0.8<=z<=0.9}
    filtered_points = filtered_points[(filtered_points[:,0]<1.35) | (filtered_points[:,0]>1.4)]
    filtered_points = filtered_points[(filtered_points[:,1]<-.5) | (filtered_points[:,1]>-.56)]
    filtered_points = filtered_points[(filtered_points[:,2]<.8) | (filtered_points[:,2]>.9)]
    
    return filtered_points


#-----------------------------------------------------------------------------------------------------------------
#Obstacle detection, detecting the presence of obstacles on a quadrilateral formed by the blue dots in front of the car

def is_inside(p1,p2,pt): # p1 is the origin, p2 another point of the border
    #P1是第一个两侧的蓝点，P2是第二个两侧的蓝点
    a = (p2[0] - p1[0]) * (pt[1] - p1[1])
    b = (p2[1] - p1[1]) * (pt[0] - p1[0])
    return (a - b) <= 0

def is_inside_quadrilateral(rect, lidar):
    obstacles = []
    for i in range (0,lidar.shape[0]) :
        if is_inside(rect[0,:],rect[1,:],lidar[i,:]) and is_inside(rect[1,:],rect[2,:],lidar[i,:]) \
                and is_inside(rect[2,:],rect[3,:],lidar[i,:]) and is_inside(rect[3,:],rect[0,:],lidar[i,:]):
            obstacles.append(lidar[i,:])
    return obstacles

def find_bounds(filtered_points) :
    #The boundingbox used to work out the obstacles
    y_bound_min = 99.0 # init at a high value for the min
    y_bound_max = -99.0 # and at a low value for the max
    x_bound_min = 99.0
    x_bound_max = -99.0
    #Enter the maximum and minimum values of the x, y boundingbox for all filter point definitions.
    if filtered_points.shape[0] > 0:
        x_bound_min = np.min(filtered_points[:,0])
        x_bound_max = np.max(filtered_points[:,0])
        y_bound_min = np.min(filtered_points[:,1])
        y_bound_max = np.max(filtered_points[:,1])

    return y_bound_min, y_bound_max, x_bound_min, x_bound_max

def brakes(x_bound):
    if x_bound > 17:
        braking = 0
    elif x_bound > 15 and x_bound <=17:
        braking = 0.2
    elif x_bound > 12 and x_bound <= 15:
        braking = 0.8
    elif x_bound > 8 and x_bound <= 12:
        braking = 3
    elif x_bound > 5 and x_bound <= 8:
        braking = 5
    else:
        braking = 8
    # the value was used before the integration of longi_speed block !!!

    return float(braking)

def create_points_box(t1, t2, car_size, dist_security,index):
    #The first input of t1 is [2.5,0,0],t2 is traj[0], then t1 is traj[0], t2 is traj[1]
    #Here the const value can be modified to extend the detection range (width) of the bounding box
    const = car_size + dist_security/1.5  # car size + 20cm security after the car
    #Calculate the difference between the x, y axes of the two path points to find the angle of entrapment
    yt = t2[1] - t1[1]
    xt = t2[0] - t1[0]
    #p0 and p1 are two three-dimensional points to be determined
    p0 = np.asarray([None, None, None])
    p1 = np.copy(p0)
    delta = math.atan2(xt,yt) #x, y offset angle
    # coordinates of point P0 in a corner of the box
    #The blue 3-D dot on the right
    p0[0] = const * math.cos(delta) + t1[0]
    p0[1] = - const * math.sin(delta) + t1[1]
    p0[2] = 0.
    #The blue 3-D dot on the left
    # coordinates of point P1 in a corner of the box
    p1[0] = - const * math.cos(delta) + t1[0]
    p1[1] = const * math.sin(delta) + t1[1]
    p1[2] = 0.
    if index == 0:
        return p0, p1
    else:
        return p1, p0

def append_rect(l,points_rect,k):
    l.append(points_rect[k,0])
    l.append(points_rect[k,1])
    l.append(points_rect[k,2])
    l.append(points_rect[k+1,0])
    l.append(points_rect[k+1,1])
    l.append(points_rect[k+1,2])

def find_points(traj, lidar, car_size, dist_security):
    #Input path points, starting from the first point to be traced
    obstacles = []
    traj_size = traj.shape[0] #reading of matrix length
    l = []
    z_min_max = [-0.25,2] #heights
    x, y = np.array((0,0,0)), np.array((0,0,0))

    if traj_size < 1:
        return 0., 99., -99., 99, np.asarray(obstacles), l #convert obstacles to array

    points_rect = np.zeros((4,3))
    traj2sec = [[2.5,0,0]] #Set the blue point closest to the sides of the car
    last_point = traj[0]

    for point_counter in range (1,traj_size) :

        points_rect[0:2] = create_points_box(traj2sec[-1], last_point, car_size, dist_security, 0)
        traj2sec.append(last_point)
        last_point = traj[point_counter]

        #Enter the middle path point and the next path point to determine the position of the blue points on both sides (coordinates and angular deflection),
        # as well as the width of the point by entering the width of the vehicle and the safety distance
        points_rect[2:4] = create_points_box(traj2sec[-1], last_point, car_size, dist_security, 1)

        bounding_box = np.array([[np.min(points_rect[:,0]),np.max(points_rect[:,0])],
                                 [np.min(points_rect[:,1]),np.max(points_rect[:,1])],z_min_max])# xmin xmax, ymin ymax, zmin, zmax

        filtered = detect_points_linear(lidar,bounding_box)
        obstacles = obstacles + is_inside_quadrilateral(points_rect, filtered)
        append_rect(l,points_rect,0) #Added here are the 5 blue path points on either side of the front of the car

    # create the last boundaries
    points_rect[0] = points_rect[3]
    points_rect[1] = points_rect[2]
    append_rect(l,points_rect,0)
    traj2sec.append(last_point)
    last_point = traj[-1]
    x = last_point[0] - traj[traj.shape[0]-2,0]
    y = last_point[1] - traj[traj.shape[0]-2,1]
    points_rect[2:4,0] = points_rect[2:4,0] + x
    points_rect[2:4,1] = points_rect[2:4,1] + y
    append_rect(l,points_rect,2)
    bounding_box = np.array([[np.min(points_rect[:,0]),np.max(points_rect[:,0])],
                             [np.min(points_rect[:,1]),np.max(points_rect[:,1])],z_min_max])# xmin xmax, ymin ymax, zmin, zmax
    filtered = detect_points_linear(lidar,bounding_box)
    obstacles = obstacles + is_inside_quadrilateral(points_rect, filtered)
    
    obstacles = np.asarray(obstacles)
    y_bound_min, y_bound_max, x_bound, x_bound_max = find_bounds(obstacles)

    if obstacles.shape[0] <= 10 : # if no obstacles found
        braking = 0.
    else:
        braking = brakes(x_bound)
    
    filtered_points = obstacles.reshape(-1)

    return braking, y_bound_min, y_bound_max, x_bound_max, filtered_points, l

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("lidar", rtmaps.types.ANY) # define input for the lidar points
        self.add_input("Dodge_1_Stop_0", rtmaps.types.ANY) # define input of the will to dodge or stop at obstacle
        self.add_input("straight_trajectory", rtmaps.types.ANY)
        
        self.add_output("out_braking", rtmaps.types.AUTO) # define output for the braking value
        self.add_output("decalage_out", rtmaps.types.AUTO) # define output for the size of the object to dodge
        self.add_output("obstacle_out", rtmaps.types.AUTO) # define output for the detection of an obstacle
        self.add_output("points_in_bound_out", rtmaps.types.AUTO,buffer_size=300000) # output for the points inside the right bounding box
        self.add_output("points_rect", rtmaps.types.AUTO,buffer_size=200)
        self.add_output("clignotant", rtmaps.types.AUTO)
        
        self.add_property("car_width", 1.4) # the width of the car
        self.add_property("security_width", 0.2) # the width between the car and the end of the detection of objects

# Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["out_braking"].write(0.0)
        self.t_obstacle = 0
        self.y_bound_decal = 0
        self.obstacle = 0
        self.decalage = 0.
        self.car_width = self.properties["car_width"].data
        self.dist_security = self.properties["security_width"].data
        
        self.str_traj = np.array(())


# Core() is called every time you have a new input
    def Core(self):
        # t = time.time()
        try:
            input_array = self.inputs["lidar"].ioelt.data # create an ioelt from the incoming lidar points
        except:
            input_array=[]
            print("Waiting Lidar")

        try:
            Dodge_1_Stop_0 = float(self.inputs["Dodge_1_Stop_0"].ioelt.data) # create an ioelt from the input
        except:
            Dodge_1_Stop_0 = 0

        #The trajectory entered in this piece is the yellow line derived from the starting point of the car
        try:
            str_traj = self.inputs["straight_trajectory"].ioelt.data
            self.str_traj = str_traj.reshape(int(np.size(str_traj)/3),3) # reshape the xyz points to x,y,z for easier use
        except:
            pass
        
        lidar = np.array(input_array) # create an numpy array
        lidar = lidar.reshape(int(lidar.shape[0]/3),3)

        #The output is the guide line, the blue dots on both sides, whether to stop or not, and the re-routing of the road plan in case of obstacles.
        out_braking, y_bound_min_out, y_bound_max_out, x_bound_out, points_in_bound, \
        points_rect = find_points(self.str_traj[1:], lidar, self.car_width, self.dist_security)
              
        t_new = time.time() # time of the execution of the code

        if out_braking > 0 : #and points_in_bound.shape[0]>3 : # more than 1 point inside the box
            print("!!! OBSTACLE !!!")
            self.obstacle = 1
            self.t_obstacle = time.time() # time of the obstacle detection
            if y_bound_max_out - y_bound_min_out > self.y_bound_decal : # if the size measured is bigger than the one detected on the previous frames
                self.y_bound_decal = y_bound_max_out - y_bound_min_out # then we replace it


        if Dodge_1_Stop_0 == 1: # if asked to dodge, out becomes 0.0 so we don't brake
            out_braking = 0.0

        #If the obstacle is removed 1.5s later, move on.
        if t_new - self.t_obstacle > 1.5 : # if 0.5 secs has elapsed since the last
            # detection, go back to init state
            self.obstacle = 0.
            self.y_bound_decal = 0.
            self.decalage = 0.
        
        clignotant = 0
        #When encountering an obstacle, avoid it by moving to the left at a distance of 1.5m
        if self.obstacle == 1 and Dodge_1_Stop_0 == 1:
            self.decalage = 1.5 + self.y_bound_decal  # definition of the distance to dodge
            clignotant = 1  # Clignotant GAUCHE ; 2 = clignotant DROIT

        #Be sure to add this positive integer conversion, or you won't be able to output the correct integers
        self.obstacle = int(self.obstacle)

        #print(f"brake : {out_braking}")

        self.outputs["out_braking"].write(out_braking) # and write it to the output刹车值
        self.outputs["decalage_out"].write(self.decalage) # and write it to the output避障距离
        self.outputs["obstacle_out"].write(self.obstacle) # and write it to the output是否障碍物
        if points_in_bound.shape[0] > 0:
            self.outputs["points_in_bound_out"].write(points_in_bound)
        else:
            print("No points in bounds.")
        if len(points_rect) > 0:
            self.outputs["points_rect"].write(points_rect)
        else:
            print("No points in rect.")
        self.outputs["clignotant"].write(clignotant)


# Death() will be called once at diagram execution shutdown
    def Death(self):
        # self.finalPointCloud.close()
        pass