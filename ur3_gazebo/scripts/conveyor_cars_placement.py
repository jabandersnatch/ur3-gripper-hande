#!/usr/bin/python

import random
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import os
import rospkg

'''
This script is used to spawn several cars in the gazebo environment.
This will be a class-based script, so that we can spawn multiple cars at once.
'''

class CarSpawner(object):
    # Constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('car_spawner')
        # Get the path to the car model aka ~/ros?ws/src/ros_ur3/car_description/urdf/car.xacro
        self.path_to_car = os.path.join(rospkg.RosPack().get_path('car_description'), 'urdf', 'car.xacro')
        # Create a service proxy to spawn the car
        try:
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
            self.spawn_car = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        except rospy.ServiceException as e:
            rospy.logerr('Spawn car service call failed: {0}'.format(e))
            self.spawn_car = None
        # Create a service proxy to delete the car
        try:
            rospy.wait_for_service('/gazebo/delete_model')
            self.delete_car = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        except rospy.ServiceException as e:
            rospy.logerr('Delete car service call failed: {0}'.format(e))
        # Create a publisher to publish the car's pose
        self.car_pose_pub = rospy.Publisher('/car_pose', Pose, queue_size=1)
        # Create a ServiceProxy to wrench the car
        try:
            rospy.wait_for_service('/gazebo/apply_body_wrench')
            self.wrench_car = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        except rospy.ServiceException as e:
            rospy.logerr('Wrench car service call failed: {0}'.format(e))
            self.wrench_car = None

    # Spawn a car
    def spawn(self, car_name, car_pose):
        # Open the car model file
        with open(self.path_to_car, 'r') as car_file:
            car_xml = car_file.read()
        # Spawn the car
        if self.spawn_car is not None:
            self.spawn_car(car_name, car_xml, 'car_description', car_pose, 'world')
        # Publish the car's pose
        self.car_pose_pub.publish(car_pose)

    # Delete a car
    def delete(self, car_name):
        # Delete the car
        self.delete_car(car_name)

    # Wrench a car
    def wrench(self, car_name, wrench):
        # Wrench the car
        if self.wrench_car is not None:
            # The arguments are body_name, reference_frame, reference_point, wrench, start_time, duration
            self.wrench_car(car_name, wrench.reference_frame, wrench.reference_point, wrench.wrench,
                            wrench.start_time, wrench.duration)

    def get_model_names(self):
        try:
            model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
            if model_states is not None:
                return model_states
            else:
                rospy.logerr('Failed to retrieve model names from /gazebo/model_states topic.')
                return []
        except rospy.ROSException:
            rospy.logerr('Failed to retrieve model names from /gazebo/model_states topic.')
            return []

if __name__ == '__main__':
    '''
    I want to spawn 10 cars on random positions in the gazebo environment. the positions may vary from (x,y) = (-1, -1) to (1,1) = (1, 1). meters.
    '''

    # Create a car spawner object
    car_spawner = CarSpawner()

    # Spawn and move cars repeatedly
    for i in range(5):
        # Spawn a car
        car_pose = Pose()
        car_pose.position.z = 0.805
        car_pose.position.x = -1.3 * (rospy.get_param('spawn_radius', 0.25) * random.random() - 0.5)
        car_pose.position.y = -0.25
        car_pose.orientation.x = 0
        car_pose.orientation.y = 0
        car_pose.orientation.z = 0
        car_pose.orientation.w = 1
        car_spawner.spawn('car' + str(i), car_pose)
        rospy.loginfo('Spawned car{0} at {1}'.format(i, car_pose))

        # Apply wrench to move the car
        wrench = ApplyBodyWrenchRequest()
        wrench.reference_frame = 'world'
        wrench.reference_point.x = 0
        wrench.reference_point.y = 0
        wrench.reference_point.z = 0
        wrench.wrench.force.x = 0
        wrench.wrench.force.y = 1.35
        wrench.wrench.force.z = 0
        wrench.start_time = rospy.Time.now()
        wrench.duration = rospy.Duration(3)
        car_name = 'car' + str(i) + '::dummy'
        car_spawner.wrench(car_name, wrench)

        # delete the cars after 2 seconds

        rospy.sleep(2)

        car_spawner.delete('car' + str(i))
        rospy.loginfo('Deleted car{0}'.format(i))



        rospy.sleep(5)  # Wait for 5 seconds before spawning the next car

    # Print the names of all models
    rospy.loginfo('Model names: {0}'.format(car_spawner.get_model_names()))
