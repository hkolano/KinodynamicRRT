'''
Kinodynamic RRT* Planner main class

For SDM Project on Kinodynamic RRT*, Winter 2021
Hannah Kolano, Nidhi Parayil, Akshaya Agarwal

Last modified by Hannah Kolano 2/25/2021
'''

import numpy as np
import random
import matlab.engine

class KinoPlanner:

    def __init__(self):
        print("Initializing Planner.")
        self.eng = self.init_matlab()
        print("MATLAB instance initialized.")
        # self.test_interpolation()

    @staticmethod
    def init_matlab():
        # Start new instance of matlab: matlab.engine.start_matlab()
        # Connect to existing instance of matlab: matlab.engine.connect_matlab()
        print("Starting MATLAB instance.")
        return matlab.engine.start_matlab()

    def get_path_torque_matlab(self, th_start, th_end, dth_start, dth_end, plotting):
        '''
        Inputs: th_start and th_end (Joint angles for the two points to interpolate between, vertical np array)
                dth_start and dth_end (Joint velocities for the two points, vertical np array)
                plotting (binary, if true --> matlab plots arm, position, velocity, acceleration, and torque)
        Calls the MATLAB script findTrajectory.m
        :returns: A sum of the torque required to traverse that path in a quintic trajectory
        '''
        [is_valid, time_scale, torque_sum] = self.eng.findTrajectory(th_start, th_end, dth_start, dth_end, plotting, 1, nargout = 3)
        return torque_sum

    def get_path_torque_random(self, th_start, th_end, dth_start, dth_end):
        '''
        Inputs: th_start and th_end in column vectors (Joint angles for the two points to interpolate between)
                dth_start and dth_end in column vectors (Joint velocities for the two points)
        Substitute for get_path_torque_MATLAB when MATLAB is not connected
        :returns: A sum of the torque required to traverse that path in a quintic trajectory
        '''
        # Pseudo-distance function between states
        pseudo_dists = [0]*5
        for i in range(5):
            pseudo_dists[i] = np.sqrt((th_end[i] - th_start[i])**2 + (dth_start[i] - dth_end[i])**2)
        return sum(pseudo_dists)*3/2

    def test_interpolation(self):
        '''
        Test function for making sure the right formats are being sent to the MATLAB engine
        :return:
        '''
        print("Testing the interpolation.")
        theta_1 = [0.0, 0.0, 0.0, 0.0, 0.0]
        theta_2 = [-45 * np.pi / 180, -70 * np.pi / 180, -100 * np.pi / 180, 10 * np.pi / 180, 0 * np.pi / 180]
        dtheta_1 = [0.0, 0.0, 0.0, 0.0, 0.0]
        dtheta_2 = [5 * np.pi / 180, -10 * np.pi / 180, -15 * np.pi / 180, -20 * np.pi / 180, 0 * np.pi / 180]

        found_torques = self.get_path_torque_random(theta_1, theta_2, dtheta_1, dtheta_2)
        print("random torque returned {}".format(found_torques))

        calc_torques = self.get_path_torque_matlab(theta_1, theta_2, dtheta_1, dtheta_2, 0)
        print("Matlab torque calculator returned {}".format(calc_torques))

    def sample_space(self):
        pass


if __name__ == "__main__":
    kinoplanner = KinoPlanner()