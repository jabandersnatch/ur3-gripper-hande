#!/usr/bin/python

import random
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel     
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

if __name__ == '__main__':
    '''
    I want to spawn 10 cars on random positions in the gazebo environment. the positions may vary from (x,y) = (-1, -1) to (1,1) = (1, 1). meters.
    '''

    # Create a car spawner object
    car_spawner = CarSpawner()

    # Spawn 10 cars
    for i in range(10):
        # Create a random pose for the car
        car_pose = Pose()
        car_pose.position.x = 2 * (rospy.get_param('spawn_radius', 1) * random.random() - 0.5)
        car_pose.position.y = 2 * (rospy.get_param('spawn_radius', 1) * random.random() - 0.5)
        car_pose.position.z = 0.1
        car_pose.orientation.x = 0
        car_pose.orientation.y = 0
        car_pose.orientation.z = 0
        car_pose.orientation.w = 1
        # Spawn the car
        car_spawner.spawn('car' + str(i), car_pose)

    # Wait for 10 seconds
    rospy.sleep(10)

    # Delete 2 cars
    for i in range(2):
        car_spawner.delete('car' + str(i))


# Path: src/ros_ur3/car_spawner/launch/spawn_cars.launch