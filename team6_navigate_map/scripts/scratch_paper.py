import numpy as np
import rospy

class Lidar:
	def __init__(self):

		#self._twist = Twist()
		self.wall_status = 1 # 1 means there is a wall in front of us. 0 means there is no wall.

		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.LidarCallback, queue_size=10)
		#self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

	def LidarCallback(self, lidar_data):
		pause = False
		min_distance = self.parse_distances(lidar_data)
		self.wall_status = self.check_for_wall(min_distance)
		
		if pause:
			rospy.sleep(10)	

	def check_for_wall(self, min_distance, distance_thresh=0.5): #is there a wall or empty block in front of us?
		if min_distance>distance_thresh:
			wall_status = 0 #there is not a wall in front of us
		else:
			wall_status = 1 #there is a wall in front of us

		return wall_status

	def parse_distances(self, lidar_data, heading=0):
		HEADING_THRESH=10

	#		try:
		heading_range = np.array(range(int(heading) - HEADING_THRESH, 
									int(heading) + HEADING_THRESH + 1))

		## wrap angles
		lidar_ang = heading_range % 360		
		lidar_ang = (lidar_ang + 360) % 360 # force positive 

		lidar_ang = [ang-360 if ang > 180 else ang for ang in lidar_ang]

		dists = [lidar_data.ranges[i] for i in lidar_ang]
		filtered_dists=[]
		for point in dists:
			point = 1000 if point == 0 else point
			filtered_dists.append(point)

		min_dist = np.min(filtered_dists)

		return min_dist

class Blowjob:
	#def __init__(self):
			#self._twist = Twist()
			#self.wall_status = 1

			#self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.LidarCallback, queue_size=10)
			#self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

	def make_waypoint_matrices():
		waypoint_matrix_x=np.empty((6,3))
		waypoint_matrix_y=np.empty((6,3))

		for i in range(6):
			for k in range(3):
				waypoint_matrix_x[i][k]=(3.3-.6*i)
				waypoint_matrix_y[i][k]=(0.6-0.6*k)
				
		return waypoint_matrix_x, waypoint_matrix_y


	def decide_orientation(old_orientation, sign_instruction):
		#old orientation is the old quaternion orientation from the navstack goal. sign instruction is the result 
		# of the image classifier reading the signs on the walls.

		stuff

	def choose_new_waypoint(old_waypoint, sign_instruction):
		#old_waypoint is the old x,y,z orientation from acm in the navstack.

		stuff