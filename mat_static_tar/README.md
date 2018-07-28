# resilient-target-tracking

we simulate a "en_range * en_range" squared environment where there are "Nt" targets and "Nr" robots. 

Each robot has four trajecotries, up, down, left and right. Each robot can only choose one of its trajectories (partition matroid).
In the robot set, there are "N_failure" failed robots and "N_resilience" resilient robots. 

The target can be tracked by robot if the target is within "epsilon" distance of one of its trajecotry. 

The curvature is calculated by only considering the greedy resilience process instead of the power set (the selected robot-trajectory set). 
