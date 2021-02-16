# KinodynamicRRT

## Connecting MATLAB
The dynamics model exists on MATLAB, not in a python script. 
Therefore, we need to be able to call MATLAB from python. 
Hannah tested this method on her Windows 10 box with Pycharm. 
1. Make sure MATLAB is installed on your machine. 
2. In the MATLAB Command Window, type `matlabroot`. This should give the install path. 
3. In the Windows Command Prompt (or, in my case, the Terminal in Pycharm), type:
```bash
cd "matlabroot\extern\engines\python"
python setup.py install
```
Then, from python, you should be able to call 
```python
import matlab.engine
eng = matlab.engine.start_matlab()
```
And access MATLAB functions with `eng.foo()` within the python script. 
Starting a MATLAB Engine takes about 10 seconds. 
After a few seconds, there is very little 
An alternative method is to instead call:
```python
eng = matlab.engine.connect_matlab()
```
