# QuadRotor Simulation and Control

This repository hosts several python scripts that accompany my study of the quadrotor. I developed scripts for simulating and control of the quadrotor in 1D (movement only in vertical axis), 2D (movement in Y-Z plane and roll) and 3D (6DoF). I made use of the excellent Python Control Systems Library. See below some images that are generated by these scripts. Also, as time allows, I will be writing on the derivation of the dynamic equations. I will link to these writings as I am able to write them.

## 1D Quadrotor Simulation and Control

Script: full_control_estimator.py

![alt text](https://github.com/juanmed/quadrotor_sim/blob/master/Images/1D_Control.png)

## 2D Quadrotor Simulation and Control

Script:2D_full_control.py (in 2D_Quadrotor folder)

![alt text](https://github.com/juanmed/quadrotor_sim/blob/master/Images/2D_ControLA.png)
![alt text](https://github.com/juanmed/quadrotor_sim/blob/master/Images/2D_ControlB.png)

## 3D Quadrotor Simulation and Control

Script: 3D_control.py (in 3D Quadrotor folder)

![alt text](https://github.com/juanmed/quadrotor_sim/blob/master/Images/3D_ControlA.png)
![alt text](https://github.com/juanmed/quadrotor_sim/blob/master/Images/3D_ControlB.png)



As an alternative to Matlab, I am using the Python Control Library [2]. Although the book [1] I am using presents code in Matlab, Matlab has a price (which is worth it) that I cannot afford. Also I am comfortable with python. 

I got inspiration on how to model the (otherwise written in paper) systems in Python from

https://www.cds.caltech.edu/~murray/wiki/index.php/Python-control/Example:_Vertical_takeoff_and_landing_aircraft

Which is also the webpage for an excellent Control Systems book[3]

On inspiration for the derivation of the linearized 3D model, I got inspiration from:
http://control.asu.edu/
and the presentations available at
http://control.asu.edu/Classes/MMAE441/Aircraft/441Lecture9.pdf
http://control.asu.edu/Classes/MMAE441/Aircraft/441Lecture10.pdf

On the derivation of several dynamic equations the following sources were useful:

This paper was in particular very useful: https://arxiv.org/pdf/1609.06088.pdf

https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-07-dynamics-fall-2009/lecture-notes/MIT16_07F09_Lec29.pdf

https://physics.stackexchange.com/questions/73961/angular-velocity-expressed-via-euler-angles




Inspiration and good ideas for obtaining the dynamic equations for the quadrotor were obtained from:
http://charlestytler.com/quadcopter-equations-motion/
https://www.mtwallets.com/quad-coptor-platform-equations-of-motion-dynamic-model/

For checking several of my results I used the following paper but I do not really recommend it as its very confusing to read:
https://ieeexplore.ieee.org/document/6417914


Some really interesting profiles of people in control are:
https://sites.google.com/site/brucefranciscontact/
https://www.comm.utoronto.ca/~frank/research.html





[1] Franklin, Powell, Emami-Naeini. Dynamic Control of Feedback Systems, 7th Edition.
[2] https://python-control.readthedocs.io/en/latest/
[3] http://www.cds.caltech.edu/~murray/amwiki/index.php?title=Second_Edition


