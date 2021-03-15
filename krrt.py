import numpy as np
import random
from KinodynamicPlanner import KinoPlanner

class cspace:
    def __init__(self):
        self.theta_lim = {
        '0':{
        'min':(-170.0/180.0*np.pi),
        'max':(170.0/180.0*np.pi),
        'vmax':(30.0/180.0*np.pi),
        },
        '1':{
        'min':(-70/180.0*np.pi),
        'max':(120/180.0*np.pi),
        'vmax':(30.0/180.0*np.pi)
        },
        '2':{
        'min':(-160/180.0*np.pi),
        'max':(30/180.0*np.pi),
        'vmax':(30.0/180.0*np.pi),
        },
        '3':{
        'min':(-160.0/180.0*np.pi),
        'max':(160.0/180.0*np.pi),
        'vmax':(50.0/180.0*np.pi),
        }
        }
        self.cspace,self.cvspace = self.create_cspace()
        [self.start_state,self.goal_state] = np.random.choice(np.array(self.cspace),2, replace=False)

    def create_cspace(self):
        cspace_states = []
        cspace_vstates = []

        new_state_v = []
        for idx in range(0,50):
            new_state=[]
            for i in range(0,4):
                new_state.append(str(round(random.uniform(self.theta_lim[str(i)]['min'],
                self.theta_lim[str(i)]['max']),2)))
            cspace_states.append(",".join(new_state))

        for idx in range(0,10):
            new_vstate=[]
            for i in range(0,4):
                new_vstate.append(str(round(random.uniform(0,
                self.theta_lim[str(i)]['vmax']),2)))
            cspace_vstates.append(",".join(new_vstate))

        return cspace_states,cspace_vstates

    def get_new_state(self, existing_states):
        not_sampled_states = [x for x in self.cspace
                                if (x not in existing_states) and
                                (x != self.start_state)]
        return np.random.choice(not_sampled_states)

    def get_new_vstate(self):
        return np.random.choice(self.cvspace)

class krrt:

    def __init__(self):
        print("initialising krrt")
        self.existing_states = {}
        self.curr_state = None
        self.prev_state = None
        self.planner=KinoPlanner()
        self.r = 7 # some arbitrary value


    def rrtstar_cost(self, parent_state, new_state, v1, v2):
        par_state = np.array(parent_state.split(',')).astype(np.float)
        curr_state = np.array(new_state.split(',')).astype(np.float)
        # print ('extract float states: ', prev_state, curr_state)
        par_vstate = np.array(v1.split(',')).astype(np.float)
        curr_vstate = np.array(v2.split(',')).astype(np.float)
        cost = sum(np.absolute(np.subtract(curr_state,par_state))) + sum(np.absolute(np.subtract(curr_vstate,par_vstate)))
        # print('rrt cost: ', cost)
        return cost
        # test()

    def krrtstar_cost(self, parent_state, new_state, v1, v2):

        # Array of parrent positions in theta(rad)

        par_state = np.array(parent_state.split(',')).astype(np.float)
        curr_state = np.array(new_state.split(',')).astype(np.float)
        # print ('extract float states: ', prev_state, curr_state)
        par_state=np.append(par_state,0.0)
        curr_state=np.append(curr_state,0.0)
        par_state=par_state.tolist()
        curr_state=curr_state.tolist()
        # Array of parrent velocity in theta_dot(rad/secs)
        par_vstate = np.array(v1.split(',')).astype(np.float)
        curr_vstate = np.array(v2.split(',')).astype(np.float)
        par_vstate=np.append(par_vstate,0.0)
        curr_vstate=np.append(curr_vstate,0.0)
        par_vstate=par_vstate.tolist()
        curr_vstate=curr_vstate.tolist()
        # print("states in the cost function", par_state,curr_state,curr_vstate,par_vstate)
        ############ Add the code for cost below. You can call Hannah's cost function here
        dummycost = self.rrtstar_cost(parent_state, new_state, v1, v2)
        if dummycost < self.r:
            valid_status, cost = self.planner.get_path_torque_matlab(par_state, curr_state, par_vstate, curr_vstate, 0)
        else:
            valid_status = 0
            cost = 100
        #print("krrt cost = ",cost)
        ## Sample cost from rrtstar
        #cost=planner.get_path_torque_matlab([0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 2.0, 2.0, 3.0, 2.0], [0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0, 1.0], 1)
        #cost = sum(np.absolute(np.subtract(curr_state,par_state))) + sum(np.absolute(np.subtract(curr_vstate,par_vstate)))
        #print('krrt cost: ', cost)
        return int(valid_status), cost

    def rrtstar_algorithm(self, cspace_obj):
        self.curr_state=cspace_obj.start_state

        # Include x,y,z from forward kinematics in dictionary

        # self.existing_states[self.curr_state] ={
        #                                         'parent':self.curr_state,
        #                                         'cost_p':0,
        #                                         'cost_tot':0
        # }

        # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
        while self.curr_state != cspace_obj.goal_state:
            self.curr_state = cspace_obj.get_new_state(self.existing_states)
            # print('self.existing_states: ',self.existing_states)
            # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
            if len(self.existing_states)==0:
                velocity = cspace_obj.get_new_vstate()
                self.existing_states[self.curr_state] ={
                                                        'parent':cspace_obj.start_state,
                                                        'cost_p':self.rrtstar_cost(cspace_obj.start_state,self.curr_state,'0.0,0.0,0.0,0.0',velocity),
                                                        'cost_tot':self.rrtstar_cost(cspace_obj.start_state,self.curr_state,'0.0,0.0,0.0,0.0',velocity),
                                                        'vel':velocity
                }
                #print('self.existing_states: ',self.existing_states)
                #print("start state",cspace_obj.start_state)
            else:
            # find parent within maximum step allowed at a time.
            # Need to get position from forward kinematics to determine the states within max step
                # print("i am in else")
                pot_parrent = {}

                if self.curr_state==cspace_obj.goal_state:
                    velocity = '0.0,0.0,0.0,0.0'
                else:
                    velocity = cspace_obj.get_new_vstate()

                for key, value in self.existing_states.items():
                    pot_parrent[key] = self.rrtstar_cost(key, self.curr_state,
                                                        v1=self.existing_states[key]['vel'], v2 = velocity)
                parent = min(pot_parrent, key = pot_parrent.get)
                self.existing_states[self.curr_state] ={
                                                        'parent':parent,
                                                        'cost_p':pot_parrent[parent],
                                                        'cost_tot':self.existing_states[parent]['cost_tot'] + pot_parrent[parent],
                                                        'vel':velocity
                }
                # pot_cost = {}
                for key, value in self.existing_states.items():
                    pot_cost = self.existing_states[self.curr_state]['cost_p'] + self.rrtstar_cost(key, self.curr_state,
                                                                                                 v1 = self.existing_states[key]['vel'],
                                                                                                 v2 = self.existing_states[self.curr_state]['vel'])
                    all_parents = []
                    temp_curr_state = self.curr_state
                    while temp_curr_state != cspace_obj.start_state:
                        prev_state = temp_curr_state
                        temp_curr_state = self.existing_states[prev_state]['parent']
                        all_parents.append(temp_curr_state)

                    if (pot_cost < value['cost_tot'] and (self.existing_states[key]['parent'] != cspace_obj.start_state) and
                    (self.curr_state!=key) and (self.curr_state!=cspace_obj.goal_state) and
                    (self.existing_states[self.curr_state]['parent'] != key) and
                    (key not in all_parents)):
                       self.existing_states[key]['parent'] = self.curr_state
                       self.existing_states[key]['cost_p'] = self.rrtstar_cost(key, self.curr_state,
                                                                            v1 = self.existing_states[key]['vel'],
                                                                            v2 = self.existing_states[self.curr_state]['vel'])
                       self.existing_states[key]['cost_tot'] = pot_cost
                # print('self.existing_states: ',self.existing_states)
                # print("start state",cspace_obj.start_state)

        # find path
        path = []
        curr_state = cspace_obj.goal_state
        prev_state = curr_state
        # print("current",curr_state)
        # print("previous",prev_state)
        state_parent ={}
        pv_states = {}
        for key,val in self.existing_states.items():
            state_parent[key] = val['parent']
            pv_states[key] = val['vel']
        # print('self.existing_states: ',self.existing_states)
        # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
        path.append(cspace_obj.goal_state + ',' + '0.0,0.0,0.0,0.0')
        while curr_state != cspace_obj.start_state:
            prev_state = curr_state
            # print('self.existing_states: ',state_parent)
            # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
            curr_state = self.existing_states[prev_state]['parent']
            if curr_state == cspace_obj.start_state:
                curr_state_str = curr_state + ',' + '0.0,0.0,0.0,0.0'
            else:
                curr_state_str = curr_state + ',' + self.existing_states[curr_state]['vel']
            path.append(curr_state_str)
        # print("the beautiful path ",path)
        # print('existing_states: ', self.existing_states)
        return path, self.existing_states[cspace_obj.goal_state]['cost_tot'], pv_states


    def krrtstar_algorithm(self, cspace_obj, sampled_states):
        self.existing_states = {}
        self.curr_state = cspace_obj.start_state

        # Include x,y,z from forward kinematics in dictionary

        # self.existing_states[self.curr_state] ={
        #                                         'parent':self.curr_state,
        #                                         'cost_p':0,
        #                                         'cost_tot':0
        # }

        # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
        for new_pos_state,new_vel_state in sampled_states.items():
        # while self.curr_state != cspace_obj.goal_state:
            self.curr_state = new_pos_state
            # print('self.existing_states: ',self.existing_states)
            # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
            if len(self.existing_states)==0:
                velocity = new_vel_state
                valid, temp_cost = self.krrtstar_cost(cspace_obj.start_state,self.curr_state,'0.0,0.0,0.0,0.0',velocity)
                if valid == 1:
                    self.existing_states[self.curr_state] ={
                                                            'parent':cspace_obj.start_state,
                                                            'cost_p':temp_cost,
                                                            'cost_tot':temp_cost,
                                                            'vel':velocity
                    }
                #print('self.existing_states: ',self.existing_states)
                #print("start state",cspace_obj.start_state)
            else:
            # find parent within maximum step allowed at a time.
            # Need to get position from forward kinematics to determine the states within max step
                # print("i am in else")
                pot_parrent = {}

                if self.curr_state==cspace_obj.goal_state:
                    velocity = '0.0,0.0,0.0,0.0'
                else:
                    velocity = new_vel_state

                for key, value in self.existing_states.items():
                    # print('finding pot parents: ')
                    valid, temp_cost = self.krrtstar_cost(key, self.curr_state,
                                                        v1=self.existing_states[key]['vel'], v2 = velocity)
                    if valid == 1:
                        pot_parrent[key] = temp_cost
                # print('finding pot parents done: ')
                if len(pot_parrent)>=1:
                    parent = min(pot_parrent, key = pot_parrent.get)
                    self.existing_states[self.curr_state] ={
                                                            'parent':parent,
                                                            'cost_p':pot_parrent[parent],
                                                            'cost_tot':self.existing_states[parent]['cost_tot'] + pot_parrent[parent],
                                                            'vel':velocity
                    }
                    # pot_cost = {}


                    ##### rewiring #######
                    for key, value in self.existing_states.items():
                        #print('cost parents')
                        pot_cost = 0
                        valid, temp_cost = self.krrtstar_cost( self.curr_state, key, v2 = self.existing_states[self.curr_state]['vel'], v1 = self.existing_states[key]['vel'])

                        if valid == 1:
                            #### mid night changes
                            # pot_cost = self.existing_states[self.curr_state]['cost_p'] + temp_cost
                            pot_cost = self.existing_states[self.curr_state]['cost_tot'] + temp_cost

                        if pot_cost!=0:
                            # print('cost parents done')
                            all_parents = []
                            temp_curr_state = self.curr_state
                            while temp_curr_state != cspace_obj.start_state:
                                prev_state = temp_curr_state
                                temp_curr_state = self.existing_states[prev_state]['parent']
                                all_parents.append(temp_curr_state)

                            if (pot_cost < value['cost_tot'] and (self.existing_states[key]['parent'] != cspace_obj.start_state) and
                            (self.curr_state!=key) and (self.curr_state!=cspace_obj.goal_state) and
                            (self.existing_states[self.curr_state]['parent'] != key) and
                            (key not in all_parents)):
                               self.existing_states[key]['parent'] = self.curr_state
                               # print('rewiring parents')
                               self.existing_states[key]['cost_p'] = temp_cost
                               # print('rewiring parents done')
                               self.existing_states[key]['cost_tot'] = pot_cost
                # print('self.existing_states: ',self.existing_states)
                # print("start state",cspace_obj.start_state)
        # print('finding path')
        # find path
        path = []
        curr_state = cspace_obj.goal_state
        prev_state = curr_state
        # print("current",curr_state)
        # print("previous",prev_state)
        state_parent ={}
        pv_states = {}
        for key,val in self.existing_states.items():
            state_parent[key] = val['parent']
            pv_states[key] = val['vel']
        # print('self.existing_states: ',self.existing_states)
        # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
        path.append(cspace_obj.goal_state + ',' + '0.0,0.0,0.0,0.0')
        while curr_state != cspace_obj.start_state:
            prev_state = curr_state
            # print('self.existing_states: ',state_parent)
            # print("start and goal state",cspace_obj.start_state, cspace_obj.goal_state)
            curr_state = self.existing_states[prev_state]['parent']
            if curr_state == cspace_obj.start_state:
                curr_state_str = curr_state + ',' + '0.0,0.0,0.0,0.0'
            else:
                curr_state_str = curr_state + ',' + self.existing_states[curr_state]['vel']
            path.append(curr_state_str)
        # print("the beautiful path ",path)
        # print('existing_states: ', self.existing_states)
        return path, self.existing_states[cspace_obj.goal_state]['cost_tot'], pv_states


    def test(self, cspace):
        print ('cspace variables: ',cspace.start_state, cspace.goal_state)

if __name__ == "__main__":
    a =krrt()
    a.approx_cost()
