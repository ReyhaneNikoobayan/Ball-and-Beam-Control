# Ball and Beam control
The final project of the course Fundamentals of Automatic Controller Design, defined by Professor [Aria Alasti ](https://sharif.ir/~aalasti/)at Sharif University of Technology And it was carried out by me and my teammate Mohammad Javad Shamseddin Saeed
# Project Description
The Ball and Beam system is one of the most well-known and simplest control systems. This system includes a long beam that allows the ball to move inside it. The control objective in this system is to precisely control the position of the ball in the middle of the beam. For this purpose, an ultrasonic sensor is embedded to detect the position and speed of the ball at every moment, and a servo motor is placed in the middle or at the edges of the beam to generate rotational motion and control the position of the ball.

![image](https://github.com/user-attachments/assets/edd71c73-f209-4929-9d43-9a394e3752fc)

![image](https://github.com/user-attachments/assets/f9004b1a-df0d-4208-97b3-5732d5767917)

# Project Requirments

The control design should have these two qualities:

1.Overshoot percentage less than 20%     

2.Settling time under 8 seconds

# Project Contents
system identification : Using a simulation model and system identification methods, obtain the transfer function 
stabilization with IMC : Using the SISO toolbox, design a PID controller that meets the above conditions and designing IMC or lead as Auxiliary controller to reach stability too
PID Tuner : Using the PID-Tuner tool, design a controller that meets the above conditions
PID methods : Designing PID with method such as ZN, R-ZN, A-H, ..
Optim PID : Using the optim PID tool, design a PID controller that meets the above conditions
2DOF PID : Designing a PID controller with two degrees of freedom to achieve the desired outcomes
Robust disturbance rejection control : designing a controller that is robust against  input disturbance of 30Hz with the amplitude of 7 or more.
