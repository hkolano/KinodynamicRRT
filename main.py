from krrt import cspace, krrt
if __name__=='__main__':
    obj =cspace()
    print(obj.cspace, obj.start_state, obj.goal_state)
    print('print new state: ', obj.get_new_state(['[2.2513282020545375, 1.394223646582816, 1.2991342648614268, 2.0174834179563006]', '[0.25411404487791694, 1.481987633854952, 2.2628142142188703, 0.32861234967968894]']))

    obj_krrt = krrt()
    obj_krrt.test(obj)
    obj_krrt.algorithm(obj)
