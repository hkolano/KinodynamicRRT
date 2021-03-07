import numpy as np
import math
from krrt import cspace, krrt
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

def trans(a,d,alpha,theta_offset,theta):
    t=[[math.cos(theta_offset+theta), -math.sin(theta_offset+theta)*math.cos(alpha), math.sin(theta_offset+theta)*math.sin(alpha), a*math.cos(theta_offset+theta)],
        [math.sin(theta_offset+theta), math.cos(theta_offset+theta)*math.cos(alpha), math.cos(theta_offset+theta)*math.sin(alpha), a*math.sin(theta_offset+theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]]
    return t

if __name__=='__main__':
    obj =cspace()
    # print(obj.cspace, obj.cvspace, obj.start_state, obj.goal_state)
    # print('print new state: ', obj.get_new_state(['[2.2513282020545375, 1.394223646582816, 1.2991342648614268, 2.0174834179563006]', '[0.25411404487791694, 1.481987633854952, 2.2628142142188703, 0.32861234967968894]']))

    obj_krrt = krrt()
    # obj_krrt.test(obj)
    rrtstar_path, rrt_cost, all_states = obj_krrt.rrtstar_algorithm(obj)
    print('rrtstar path: ',rrtstar_path)
    print('rrtstar cost: ', rrt_cost)
    print('rrtstar all_states', all_states)

    # obj_krrt.krrtstar_algorithm(obj, all_states)
    krrtstar_path, krrt_cost, _pv = obj_krrt.krrtstar_algorithm(obj, all_states)
    print('krrtstar path: ',krrtstar_path)
    print('krrtstar cost: ', krrt_cost)
    print('pv: ',_pv)





    dummy_path = ['0.26,0.6,1.46,2.95', '1.44,1.44,1.4,1.73', '1.89,0.88,0.87,2.19']


    ##### forward kenimatics #####

    a=[20, 150.71, 20, 0, 0]
    d=[46.2, 0, 0, -180, 0]
    alpha=[46.2, 0, 0, -180, 0]
    theta_offset=[np.pi, -math.atan2(145.3, 40), -math.atan2(145.3, 40), np.pi/2, -np.pi/2]








    # t0=[[math.cos(theta_offset[0]), -math.sin(theta_offset[0])*math.cos(alpha[0]), math.sin(theta_offset[0])*math.sin(alpha[0]), a[0]*math.cos(theta_offset[0])],
    #     [math.sin(theta_offset[0]), math.cos(theta_offset[0])*math.cos(alpha[0]), math.cos(theta_offset[0])*math.sin(alpha[0]), a[0]*math.sin(theta_offset[0])],
    #     [0, math.sin(alpha[0]), math.cos(alpha[0]), d[0]],
    #     [0, 0, 0, 1]]
    # t1=[[math.cos(theta_offset[1]), -math.sin(theta_offset[1])*math.cos(alpha[1]), math.sin(theta_offset[1])*math.sin(alpha[1]), a[1]*math.cos(theta_offset[1])],
    #     [math.sin(theta_offset[1]), math.cos(theta_offset[1])*math.cos(alpha[1]), math.cos(theta_offset[1])*math.sin(alpha[1]), a[1]*math.sin(theta_offset[1])],
    #     [0, math.sin(alpha[1]), math.cos(alpha[1]), d[1]],
    #     [0, 0, 0, 1]]
    # t2=[[math.cos(theta_offset[2]), -math.sin(theta_offset[2])*math.cos(alpha[2]), math.sin(theta_offset[2])*math.sin(alpha[2]), a[2]*math.cos(theta_offset[2])],
    #     [math.sin(theta_offset[2]), math.cos(theta_offset[2])*math.cos(alpha[2]), math.cos(theta_offset[2])*math.sin(alpha[2]), a[2]*math.sin(theta_offset[2])],
    #     [0, math.sin(alpha[2]), math.cos(alpha[2]), d[2]],
    #     [0, 0, 0, 1]]
    # t3=[[math.cos(theta_offset[3]), -math.sin(theta_offset[3])*math.cos(alpha[3]), math.sin(theta_offset[3])*math.sin(alpha[3]), a[3]*math.cos(theta_offset[3])],
    #     [math.sin(theta_offset[3]), math.cos(theta_offset[3])*math.cos(alpha[3]), math.cos(theta_offset[3])*math.sin(alpha[3]), a[3]*math.sin(theta_offset[3])],
    #     [0, math.sin(alpha[3]), math.cos(alpha[3]), d[3]],
    #     [0, 0, 0, 1]]
    # t4=[[math.cos(theta_offset[4]), -math.sin(theta_offset[4])*math.cos(alpha[4]), math.sin(theta_offset[4])*math.sin(alpha[4]), a[4]*math.cos(theta_offset[4])],
    #     [math.sin(theta_offset[4]), math.cos(theta_offset[4])*math.cos(alpha[4]), math.cos(theta_offset[4])*math.sin(alpha[4]), a[4]*math.sin(theta_offset[4])],
    #     [0, math.sin(alpha[4]), math.cos(alpha[4]), d[4]],
    #     [0, 0, 0, 1]]



    ###### Plot forward kinematics points ########
    # positions = []
    # for idx,value in enumerate(dummy_path):
    #     theta = ['0.0']
    #     theta.extend(value.split(','))
    #     # print('theta: ',theta)
    #     t = [[],[],[],[],[]]
    #
    #     for i in range(5):
    #         # print('all params: ',a[i],d[i],alpha[i],theta_offset[i],float(theta[i]))
    #         t[i]=trans(a[i],d[i],alpha[i],theta_offset[i],float(theta[i]))
    #     # print(t)
    #     t0_4 = np.dot(np.dot(np.dot(np.dot(t[0],t[1]),t[2]),t[3]),t[4])
    #     positions.append(list(t0_4[:-1,3]))
    #
    #
    # print('positions: ', positions, np.array(positions)[:,0])
    # x = list(np.array(positions)[:,0])
    # y = list(np.array(positions)[:,1])
    # z = list(np.array(positions)[:,2])
    # ax = plt.gca(projection="3d")
    # ax.scatter(x,y,z, c='r',s=100)
    # ax.plot(x,y,z, color='r')
    # plt.show()
