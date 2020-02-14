# baxter-generative-model-moveit
Learning collision-free space on the Baxter robot, with application to motion planning. This package contains 
the modules for interacting with ROS. 

The learning modules is contained in this package: https://github.com/jamesdi1993/baxter-generative-learning 

## Main directories and files
- experiments/ experiments for the project.
  -- vae_motion_planning: main file and experiment for the project
- data_collections/ various scripts for dataset generation and validation;
- env/ environments to generate dataset and run experiments on;
- forward_kinematics/ client for computing forward kin
- objects/ stl files for objects in environment
- sampler/ sampler for generating samples in free space
- state_validity_check/ state validity checkers; Only check for joint limits and collision checks in this project
- utils/ utility files;
- visualization/ clients and helpers for interacting with Rviz to visualize motion plans
