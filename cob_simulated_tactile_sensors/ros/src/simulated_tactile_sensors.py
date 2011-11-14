#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_simulated_tactile_sensors')
import rospy
import math
from schunk_sdh.msg import *
from gazebo_plugins.msg import *
from geometry_msgs.msg import *


class GazeboTactilePad():
	
	def __init__(self, topic_name, finger_length, finger_width):
		self.finger_length = finger_length
		self.finger_width = finger_width
		
		# get parameters from parameter server
		self.cells_x = rospy.get_param("~cells_x", 6)
		self.cells_y = rospy.get_param("~cells_y", 14)
		rospy.logdebug("size of the tactile matrix is %ix%i patches", self.cells_x, self.cells_y)
		
		self.sensitivity = rospy.get_param("~sensitivity", 350.0)
		rospy.logdebug("sensitivity: %f", self.sensitivity)
		
		self.range = rospy.get_param("~output_range", 3500)
		rospy.logdebug("output range: %f", self.range)
		
		self.filter_length = rospy.get_param("~filter_length", 10)
		rospy.logdebug("filter length: %i", self.filter_length)
		
		rospy.Subscriber(topic_name, ContactsState, self.contact_callback)
		rospy.logdebug("subscribed to bumper states topic: %s", topic_name)
		
		self.smoothed_matrix = self.create_empty_force_matrix()


	'''
	Update the pad with the Gazebo contact information.
	
	:param states: gazebo_plugins.msg.ContactsState
	'''
	def contact_callback(self, states):
		matrix = self.create_empty_force_matrix()
		matrix = self.update(states, matrix)
		matrix = self.smooth(matrix)
		matrix = self.time_average(self.smoothed_matrix, matrix, self.filter_length)
		
		self.smoothed_matrix = matrix


	'''
	Update the provided matrix with the contact information from the Gazebo
	simulator.
	
	:param states: gazebo_plugins.msg.ContactsState
	:param matrix: Float[]
	'''
	def update(self, states, matrix):
		# no contact, so we don't update the matrix
		if (len(states.states) == 0):
			return matrix
		
		for i in range(len(states.states)):
			state = states.states[i]
			for j in range(len(state.contact_positions)):
				# check if the contact is on the same side as the sensor
				if (state.contact_positions[j].x < 0.0):
					continue
				
				# normalize the contact position along the tactile sensor
				# we assume that the tactile sensor occupies the complete
				# surface of the inner finger side, so finger size is equal to
				# sensor size
				normalized_x = (state.contact_positions[j].y / self.finger_width) + 0.5
				normalized_y = state.contact_positions[j].z / self.finger_length
				
				# from the normalized coordinate we can now determine the index
				# of the tactile patch that is activated by the contact
				x = round(normalized_x * self.cells_x)
				y = round(normalized_y * self.cells_y)
				
				force = -state.wrenches[j].force.x
				if (force < 0.0): force = 0.0
				
				index = int(self.get_index(x, y))
				
				matrix[index] += force
		
		return matrix


	'''
	Create a new matrix that contains the current force on each cell. Initialize
	all cells with zeros.
	
	:return: Float[]
	'''
	def create_empty_force_matrix(self):
		matrix = []
		for i in range(self.cells_x * self.cells_y):
			matrix.append(0.0)
		
		return matrix


	'''
	Run a moving average on the provided matrix.
	
	:param matrix: Float[]
	:return: Float[]
	'''
	def smooth(self, matrix):
		smoothed_matrix = self.create_empty_force_matrix()
		
		for x in range(0, self.cells_x):
			for y in range(0, self.cells_y):
				sum = 0.0
				count = 0
				for dx in range(-1, 2):
					for dy in range(-1, 2):
						index_x = x + dx
						index_y = y + dy
						
						if (index_x < 0 or index_x >= self.cells_x): continue
						if (index_y < 0 or index_y >= self.cells_y): continue
						
						index = self.get_index(index_x, index_y)
						sum += matrix[index]
						count += 1
				index = self.get_index(x, y)
				smoothed_matrix[index] = sum / count
		
		return smoothed_matrix


	'''
	Calculate the average matrix from the force buffer.
	
	:return: Float[]
	'''
	def time_average(self, matrix_buffer, current_matrix, filter_length):
		matrix = self.create_empty_force_matrix()
		sample_factor = 1.0 / filter_length
		
		for i in range(self.cells_x * self.cells_y):
			force = (1.0 - sample_factor) * matrix_buffer[i]
			force += sample_factor * current_matrix[i]
			matrix[i] = force
		
		return matrix


	'''
	Get the current forces as tactile matrix. matrix_id is the identifier of the
	tactile matrix and determines which pad produced the data.
	
	:param matrix_id: Integer
	:return: schunk_sdh.msg.TactileMatrix
	'''
	def tactile_matrix(self, matrix_id):
		matrix = TactileMatrix()
		matrix.matrix_id = matrix_id
		matrix.cells_x = self.cells_x
		matrix.cells_y = self.cells_y
		
		m = self.smoothed_matrix
		
		for i in range(self.cells_x * self.cells_y):
			force = m[i] * self.sensitivity
			if (force < 0.0): force = 0.0
			if (force > self.range): force = self.range
			matrix.tactile_array.append(force)
				
		return matrix


	'''
	Map the two-dimensional coordinate of a tactile patch to an index in the
	one-dimensional data array. The coordinates are bound to the upper and lower
	limits.
	
	:param x: Integer
	:param y: Integer
	:return: Integer
	'''
	def get_index(self, x, y):
		y = self.cells_y - y - 1
		if (x >= self.cells_x): x = self.cells_x - 1
		if (x < 0): x = 0
		if (y >= self.cells_y): y = self.cells_y - 1
		if (y < 0): y = 0
		
		return y * self.cells_x + x




class GazeboVirtualTactileSensor():
	'''
	Constants that determine which indices the finger parts have in the
	schunk_sdh.msg.TactileSensor matrix.
	'''
	ID_FINGER_12 = 0
	ID_FINGER_13 = 1
	ID_THUMB_2 = 2
	ID_THUMB_3 = 3
	ID_FINGER_22 = 4
	ID_FINGER_23 = 5
	
	
	def __init__(self):
		self.pads = []
		self.pads.append(GazeboTactilePad("finger_12/state", 0.0865, 0.03))
		self.pads.append(GazeboTactilePad("finger_13/state", 0.0675, 0.03))
		self.pads.append(GazeboTactilePad("thumb_2/state", 0.0865, 0.03))
		self.pads.append(GazeboTactilePad("thumb_3/state", 0.0675, 0.03))
		self.pads.append(GazeboTactilePad("finger_22/state", 0.0865, 0.03))
		self.pads.append(GazeboTactilePad("finger_23/state", 0.0675, 0.03))
		
		self.pub = rospy.Publisher("tactile_data", TactileSensor)
		rospy.loginfo("'tactile_data' topic advertized")


	'''
	Publish the current state of the simulated tactile sensors.
	'''
	def publish(self):
		msg = TactileSensor()
		msg.header.stamp = rospy.Time.now()
		for i in range(6):
			msg.tactile_matrix.append(self.pads[i].tactile_matrix(i))
		self.pub.publish(msg)




if __name__ == "__main__":
	rospy.init_node('tactile_sensors')
	rospy.sleep(0.5)
	
	sensor = GazeboVirtualTactileSensor()
	
	while not rospy.is_shutdown():
		sensor.publish()
		rospy.sleep(0.05)
