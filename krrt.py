import numpy as np
import random
from KinodynamicPlanner import KinoPlanner

class cspace:
    def __init__(self):
        self.theta_lim = {}
        for i in range(0,4):
            self.theta_lim[str(i)]={
            'min':0,
            'max':np.pi
            }
        self.cspace = self.create_cspace()
        [self.start_state,self.goal_state] = np.random.choice(np.array(self.cspace),2, replace=False)

    def create_cspace(self):
        cspace_states = []
        for idx in range(0,10):
            new_state=[]
            for i in range(0,4):
                new_state.append(str(round(random.uniform(self.theta_lim[str(i)]['min'],
                self.theta_lim[str(i)]['max']),2)))
            cspace_states.append(",".join(new_state))
        return cspace_states

    def get_new_state(self, existing_states):
        not_sampled_states = [x for x in self.cspace
                                if (x not in existing_states) and
                                (x != self.start_state)]
        return np.random.choice(not_sampled_states)

class krrt:

    def __init__(self):
        print("initialising krrt")
        self.existing_states = {}
        self.curr_state = None
        self.prev_state = None


    def dummy_cost(self, parent_state, new_state):
        par_state = np.array(parent_state.split(',')).astype(np.float)
        curr_state = np.array(new_state.split(',')).astype(np.float)
        # print ('extract float states: ', prev_state, curr_state)
        print('cost: ', sum(np.subtract(curr_state,par_state)))
        return sum(np.subtract(curr_state,par_state))
        # test()

    def algorithm(self, cspace_obj):
        self.curr_state=cspace_obj.start_state

        # Include x,y,z from forward kinematics in dictionary

        # self.existing_states[self.curr_state] ={
        #                                         'parent':self.curr_state,
        #                                         'cost_p':0,
        #                                         'cost_tot':0
        # }
        c = 0
        while c<3: # self.curr_state != cspace_obj.goal_state:
            c = c+1
            self.curr_state = cspace_obj.get_new_state(self.existing_states)
            if len(self.existing_states)==0:
                self.existing_states[self.curr_state] ={
                                                        'parent':cspace_obj.start_state,
                                                        'cost_p':self.dummy_cost(cspace_obj.start_state,self.curr_state),
                                                        'cost_tot':self.dummy_cost(cspace_obj.start_state,self.curr_state)
                }
            else:
            # find parent within maximum step allowed at a time.
            # Need to get position from forward kinematics to determine the states within max step
                pot_parrent = {}
                for key, value in self.existing_states.items():
                    pot_parrent[key] = self.dummy_cost(key, self.curr_state)
                parent = min(pot_parrent, key = pot_parrent.get)
                print ('parent: ', parent)
                self.existing_states[self.curr_state] ={
                                                        'parent':parent,
                                                        'cost_p':pot_parrent[parent],
                                                        'cost_tot':self.existing_states[parent]['cost_tot'] + pot_parrent[parent]
                }
                # pot_cost = {}
                for key, value in self.existing_states.items():
                    pot_cost = self.existing_states[self.curr_state] + self.dummy_cost(key, self.curr_state)
                    if pot_cost < value['cost_tot']:
                       self.existing_states[key] ={
                                                                'parent':self.curr_state,
                                                                'cost_p':self.dummy_cost(key, self.curr_state),
                                                                'cost_tot':pot_cost
                        }
            #self.prev_state = self.curr_state

        # find path
        path = []
        curr_state = cspace_obj.goal_state
        prev_state = curr_state
        while curr_state != cspace_obj.start_state:
            prev_state = curr_state
            curr_state = self.existing_states[prev_state]['parent']
            path.append(curr_state)

        print('existing_states: ', self.existing_states)
        return path, self.existing_states[cspace_obj.goal_state]['cost_tot']

    def test(self, cspace):
        print ('cspace variables: ',cspace.start_state, cspace.goal_state)

if __name__ == "__main__":
    a =krrt()
    a.approx_cost()
