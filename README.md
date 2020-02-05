# State_Estimation
Comparison of Dead Reckoning, Least square, Extended Kalman and Particle filter

This program is written on python2.7 and excuted in Ubuntu16.04 

Steps to run this simulation on your system 
- First Install **python** 
- Open the terminal/CMD and type the following commands to get started
    - Type ```git clone https://github.com/naveenbiitk/State_Estimation.git ```
    - This will create a copy of the above code on your current folder in terminal
    - Now go inside the ```python_1``` folder in the terminal
    - run the code by ```python main.py```
    
You can see the simulation environment, the input control is corrupted with manual noise and the environment is assumed to be fitted with 4 becon, which provides correction to the point-robot. 

## Dead reckoning
<img src="https://github.com/naveenbiitk/State_Estimation/blob/master/python_1/dr.png" align="center" width="400" alt="dr pic"/>

## Least square
<img src="https://github.com/naveenbiitk/State_Estimation/blob/master/python_1/lq.png" align="center" width="400" alt="lq pic"/>

## Particle filter
<img src="https://github.com/naveenbiitk/State_Estimation/blob/master/python_1/pf.png" align="center" width="400" alt="pf pic"/>

## Extended Kalman filter
<img src="https://github.com/naveenbiitk/State_Estimation/blob/master/python_1/ekf.png" align="center" width="400" alt="ekf pic"/>

Further theory and reference can be seen in [here](https://github.com/AtsushiSakai/PythonRobotics)
