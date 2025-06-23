# Navigation with Graph of Free Regions
<p align ="center">
<img src="fig/dof_property.jpg" width=75%>
</p>

Overview of mechanical design: (a) Illustration of the joint DoF design for FLORES, along with the positioning of electrical components. The magnified view on the lower side shows the joint layout of the front left leg. (b) Range of motion for the front hip joint. (c) Range of motion for the rear hip joint.

<p align ="center">
<img src="fig/system_framework.png" width=90%>
</p>

Overview of our system architecture and training pipeline. The training process consists of two phases: Phase I involves sim-to-sim transfer, where the HIM policy is first trained in Isaac Gym with reward shaping, and then is transferred and validated in MuJoCo as an intermediate step; Phase II performs Sim2Real deployment using ROS2 control framework to establish robot communications. The hardware framework (right panel) shows the onboard PC receiving user input and combining it with current observations to feed into the trained RL model, which outputs actions for robot control. Red lines indicate power connections, and black arrows show the information flow.

## Results
<p align ="center">
<img src="fig/sim_maze.gif" width=75%>
</p>

## Mechanical parts
All machined parts are manufactured with outsourced CNC service. The .stl file of all the parts can be found in Mechanical folder.


## Authors


