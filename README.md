# biodynamo-flocking-simulation
A short instruction is given on how to replicate the conducted simulations.

Make sure to use a compatible version of BioDynaMo (e.g. v1.01.96-a06bafed).
The external library cnpy has to be installed https://github.com/rogersce/cnpy.
In it's current implementationhe, the code will load the wind field into the simulation, even if ```apply_wind_field``` is set to ```false```.
This does siginificantly increase laoading time for now. However, you can use the the 1x1x1 placeholder windfield in src/pywind/data. This will be used by default.
When using a custom wind field make sure to replace this file ```src/pywind/data/wind.npy``` with the new one.

We used the random number mofule of BioDynaMo. It looks like this always creates the exact same sequence of numbers.
Therefore, when choosing the same simulation settings the exact same simulation should be executed as in the thesis.

The simulation parameters values and default ```bdm.json``` have the values as listed in the beginning of chapter 5 set by default. 
But make sure to double check in case and experiment with them.

We will give a small instruction on how to replicate the conducted simulations.
The most important / changed from the default parameters are highlighted for each simulation.
The data used for our plots is also given in the corresponding output/data_xxx folders alongside a matlab script to plot / visualize them.

Alternatively you can export data from a simulation via the parameters ```export_distances``` and ```export_velocity```.
These will create a csv file in output/data each. Each line of the file is one agent and in each comlumn the average neigbor distance or agent's velocity is saved.
Keep in mind this will slow downcomputation time.
The included data folders are:
- data_free_space: data for figure 10: Plot of the reduced average distance...
- data_wind: data for Figure 14: "Wind resistance..." and figure 15: "Plot of the average velocity within a flock of 500 drifting agents..."

To visualize the simulation in Paraview, import data as usual and set the scaling of agents to ```actual_diameter_```.
For the simulations in presence of pbstacles you have to load them first, so before loading the agents' state. Therefore load stat ```output/obstacles.pvsmv```, afterwards load agent state as usual.
You can now select the wanted obstace as this file has all 3 incorporated.


------------------------------------------------------------------------------------------------------------
5.1  Flocking in Free Space: Simulation without extend cohesion term (figure 7)
------------------------------------------------------------------------------------------------------------
```"n_boids": 250```  
```"starting_sphere_radius": 100```  
```"c_a_3": 0 ```  
```"c_y": 0.0```  
```"simulation_setup": "free_space"```  
```"apply_wind_field": false```  
```"limit_speed": true```  

------------------------------------------------------------------------------------------------------------
5.1  Flocking in Free Space: Collaps of a flock (figure 8)
------------------------------------------------------------------------------------------------------------
same settings as in "5.1  Flocking in Free Space: Experient 1 (fig 7)"  
```"boid_interaction_radius": 150```  

------------------------------------------------------------------------------------------------------------
5.1  Flocking in Free Space: Simulation with extend cohesion term (figure 9)
------------------------------------------------------------------------------------------------------------
same settings as in "5.1  Flocking in Free Space: Simulation with extend cohesion term (figure 9)"  
```"c_a_3": 0.05```  

------------------------------------------------------------------------------------------------------------
5.1  Flocking in Free Space: Avg neighbor distance (figure 10)
------------------------------------------------------------------------------------------------------------
```"n_boids": 500```  
```"starting_sphere_radius": 200```  
```"simulation_setup": "free_space_avg_dist"```  
```"max_bound": 50000```  
```"min_bound": -50000```  
```"c_a_3": 0 / 0.05```  
```"c_y": 0.05```  
```"export_distances": true```  
in sim_param.h:  ```Double3 pos_gamma = {100000, 0, 0};```  

------------------------------------------------------------------------------------------------------------
5.2 Flocking in the Presence of Obstacles (figure 11, 12, 13)
------------------------------------------------------------------------------------------------------------
```"simulation_setup": "obstacle_spherical" / "obstacle_cuboid" / "obstacle_wall"```  
```"max_bound": 1500```  
```"min_bound": -1500```  
in sim_param.h:  ```Double3 pos_gamma = {1000, 0, 0}```  

------------------------------------------------------------------------------------------------------------
5.2 Flocking in the Presence of External Forces (Wind Field Implemenation)
------------------------------------------------------------------------------------------------------------
```"simulation_setup": "wind"```  
```"apply_wind_field": true```  
```"limit_speed": false```  
