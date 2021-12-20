import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
import math
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle", 100)
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization", 500)
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view", 4.71)
        self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale", 1.0)

        ####################################
        ##TODo
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0
 
        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)
    
    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.
        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.
        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        
        self.sensor_model_table = np.zeros((self.table_width,self.table_width))

        z_max = self.table_width-1

        for z_t_star in xrange(self.table_width):
            P_hit = 0.0         # The accumulated area under pdf_hit for z = (0, ..., z_t_max-1)
            norm_factor = 0.0   # norm_factor * P_hit = 1

            # z is the (discrete index of) observed range from the lidar unit
            for z in xrange(self.table_width):
                pdf_hit = 0.0
                if z <= z_max and z>=0:
                    pdf_hit = np.exp(- float(z - z_t_star)**2.0/(2.0*self.sigma_hit**2.0))  #distribution
                    pdf_hit = pdf_hit / (np.sqrt(2.0*np.pi*self.sigma_hit**2.0))
                P_hit = P_hit + pdf_hit
                
                self.sensor_model_table[z,z_t_star] = pdf_hit
           
            # normalize data
            norm_factor = 1 / P_hit
            self.sensor_model_table[:, z_t_star] *= norm_factor
        # changew for reading later.... mutl
        
        self.sensor_model_table *= self.alpha_hit
        
        for z_t_star in xrange(self.table_width):
            norm_factor  = 0.0
            
            # z is the observed range from the lidar unit
            for z in xrange(self.table_width):
                
                prob = self.sensor_model_table[int(z),int(z_t_star)] 
                     
                # short reading
                if z <= z_t_star and z>=0 and z_t_star!=0:
                    prob = prob + ((2.0/float(z_t_star)) * self.alpha_short * (1.0-(float(z)/float(z_t_star))))

                # error max range
                if int(z) == int(z_max):
                    prob = prob + self.alpha_max

                # random measurement
                if z <= int(z_max) and z>=0:
                    prob = prob + self.alpha_rand * 1.0/float(z_max)
                
                norm_factor  = norm_factor + prob
                
                self.sensor_model_table[int(z),int(z_t_star)] = prob
                
            # normalize data
            self.sensor_model_table[:,int(z_t_star)] /= float(norm_factor)          
            #self.sensor_model_table.transpose()    
                
    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.
        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]
            observation: A vector of lidar data measured
                from the actual lidar.
        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        #if not self.map_set:
           # return

        ####################################
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        scans = self.scan_sim.scan(particles)
        
        #declare z_max
        z_max = self.table_width-1

        ## get the shape of observation to use with downsampling
        #size = observation.shape
        #print(size)

        # first downsample the observation lidar to have the same number columns as num_beams_per_particle in params.yaml file
        #observation = observation[::int(size[0]/self.num_beams_per_particle)]
        #print(observation.shape)
        
        
        ## the likelihood of a scan is computed as the product of the likelihoods of each of n range measurements in the scan
        ## must loop through each value in the scan, and multiply all the probabilities together to get the likelihood of the scan
        

        ## need to scale lidar scans from meters to pixels
        observation/= float(self.map_resolution*self.lidar_scale_to_map_scale)
        scans/= float(self.map_resolution*self.lidar_scale_to_map_scale)
        
        ## get the shape of scans to know how much to loop to access each element
        size = scans.shape

        # squishing parameter
        squish = 2.2
        
        # emply np array to hold calculated probabilities
        probabilities = np.zeros(size[0])
        
        observation[observation>z_max]=z_max
        observation[observation<0]=0

        scans[scans>z_max]=z_max
        scans[scans<0]=0
                
        #loop through all the rows
        for i in xrange(size[0]):
            prob = 1.0
            running_probs = np.zeros(size[1])
            # loop through the columns of each row
            for j in xrange(size[1]):
                z = observation[j]
                # checking if values are out of range and clipping if needed
                z_star = scans[i, j]
                z_round = int(np.rint(z))
                z_star_round = int(np.rint(z_star))
                #print(z_round)
                #print(z_star_round)
                running_probs[j] = (self.sensor_model_table[z_round, z_star_round])

                #prob *= ((self.sensor_model_table[int(z),int(z_star)]))
            
            probabilities[i] = prob
            probabilities[i] = np.prod(running_probs)

        return np.power(probabilities, 1.0/squish)

        #if not self.map_set:
           # return
        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)
        self.map_resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
