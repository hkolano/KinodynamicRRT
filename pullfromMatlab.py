'''
pullfromMatlab.py
Attempt to call dynamics functions written in MATLAB from python

Last modified by Hannah Kolano 2/16/2021
'''

import matlab.engine
import time

start_time = time.time()
# eng = matlab.engine.connect_matlab()
eng = matlab.engine.start_matlab()
end_time = time.time()
print("time to start up: {}".format(end_time-start_time))

start_time = time.time()
tf = eng.sqrt(42.0)
end_time = time.time()
print("time to call function: {}".format(end_time-start_time))
print(tf)