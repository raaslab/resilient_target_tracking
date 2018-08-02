%trajectory assign algorithm, include both bruturesilient

function traj_assign_fun(~,~)

global N_uavs N_targets uavs_pos uavs_ini_pos targets_pos gz_model_states_sub ...
       uav1_state_sub uav2_state_sub uav3_state_sub uav4_state_sub uav5_state...       

 
 uavs_pos_local = zeros(N_uavs,3);

%receive the gazebo_model_states
 gz_model_states = receive(gz_model_states_sub,3); 


% receive the uav pos
uav1_state = receive(uav1_state_sub,1);
uav2_state = receive(uav2_state_sub,1);
uav3_state = receive(uav3_state_sub,1);
uav4_state = receive(uav4_state_sub,1);

%calculate the uav local states from mavros receiving
uavs_pos_local(1,:) = [uav1_state.Pose.Position.X, uav1_state.Pose.Position.Y, uav1_state.Pose.Position.Z];
uavs_pos_local(2,:) = [uav2_state.Pose.Position.X, uav2_state.Pose.Position.Y, uav2_state.Pose.Position.Z];
uavs_pos_local(3,:) = [uav3_state.Pose.Position.X, uav3_state.Pose.Position.Y, uav3_state.Pose.Position.Z];
uavs_pos_local(4,:) = [uav4_state.Pose.Position.X, uav4_state.Pose.Position.Y, uav4_state.Pose.Position.Z];

uavs_pos = uavs_pos_local + uavs_ini_pos; 
 
 %calcaulate the positions of the N_uavs targets, use services
%  [uavs_pos(1,:), ~, ~] = getState(uav1_state); 
%  [uavs_pos(2,:), ~, ~] = getState(uav2_state);
%  [uavs_pos(3,:), ~, ~] = getState(uav3_state);
%  [uavs_pos(4,:), ~, ~] = getState(uav4_state);
 %[uavs_pos(5,:), ~, ~] = getState(uav5_state);

 
 % for targets !!! make sure to check if the ID corresponds
 % well!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 for i = 1: N_targets
     
     targets_pos(i,:) = [gz_model_states.Pose(i+1).Position.X, ...
                         gz_model_states.Pose(i+1).Position.Y,...
                         gz_model_states.Pose(i+1).Position.Z];
 end
  
%  uavs_pos(4,:) = [gz_model_states.Pose(N_targets+2).Position.X,...
%                   gz_model_states.Pose(N_targets+2).Position.Y,...
%                   gz_model_states.Pose(N_targets+2).Position.Z];
%               
%  uavs_pos(1,:) = [gz_model_states.Pose(N_targets+3).Position.X,...
%                   gz_model_states.Pose(N_targets+3).Position.Y,...
%                   gz_model_states.Pose(N_targets+3).Position.Z];  
%               
%  uavs_pos(3,:) = [gz_model_states.Pose(N_targets+4).Position.X,...
%                   gz_model_states.Pose(N_targets+4).Position.Y,...
%                   gz_model_states.Pose(N_targets+4).Position.Z];
%               
%  uavs_pos(2,:) = [gz_model_states.Pose(N_targets+5).Position.X,...
%                   gz_model_states.Pose(N_targets+5).Position.Y,...
%                   gz_model_states.Pose(N_targets+5).Position.Z];                
         
 

 %***after receive the pos from uavs and targets
 [target_cover, n_id_maxtra] = uav_tra_cover_fun();
 
 
 %%trajectory selection by four different algorithms
 % brute_force, resilient, greedy and random

 % 1st brute_force
 %  bf_traj_assign(target_cover);
 
 % 2nd resilient 
 %  resilient_traj_assign(target_cover, n_id_maxtra);  
 
 % 3rd greedy
 %  greedy_traj_assign(target_cover);
 
 % 4th random
   random_traj_assign(target_cover);
 end