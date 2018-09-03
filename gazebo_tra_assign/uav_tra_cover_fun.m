% 1. calculate the target set tracked by each trajectory of each robot
% 3. calculate number of targtes covered by maximum traj for each robot and
% the id of this traj
function [target_cover, n_id_maxtra] = uav_tra_cover_fun()

global N_uavs N_tars

global N_dir_uav

global uavs_pos tars_pos 

global track_length track_width sta_track_length


 target_cover = cell(N_uavs, N_dir_uav);
 
 n_target_cover = zeros(N_uavs, N_dir_uav);
 
 n_id_maxtra = zeros(N_uavs,2); % number of targtes covered by maximum tra for each robot and traj_id
 
 
 for i = 1 : N_uavs
     
     for j = 1 : N_dir_uav % each robot has N_dir_uav directions
         
         if j == 1 % up direction
            
             for k = 1 : N_tars % check all the targets
                 if tars_pos(k,2) >= uavs_pos(i,2) - sta_track_length/2 && ...
                    tars_pos(k,2) - (uavs_pos(i,2) - sta_track_length/2) <= track_length && ...
                    abs(tars_pos(k,1)- uavs_pos(i,1))<=track_width/2 % the targets are above the robot, just use ||x_r-x_t||
                     
                    target_cover{i,j}=[target_cover{i,j}, k]; % store the targets can be tracked if the distance is within tolerance
                 end
             end
             
         elseif j == 2 % down direction
             
             for k = 1 : N_tars % check all the targets
                 if tars_pos(k,2) <= uavs_pos(i,2) + sta_track_length/2 && ...
                    (uavs_pos(i,2)+sta_track_length/2) - tars_pos(k,2) <= track_length && ...
                    abs(tars_pos(k,1)- uavs_pos(i,1))<=track_width/2 % the targets are below the robot, just use ||x_r-x_t||
                     
                    target_cover{i,j}=[target_cover{i,j}, k]; % store the targets can be tracked if the distance is within tolerance
                 end
             end             
             
                
         elseif j == 3 % right direction

             for k = 1 : N_tars % check all the targets
                 if tars_pos(k,1) >= uavs_pos(i,1) - sta_track_length/2&& ...
                    tars_pos(k,1)- (uavs_pos(i,1) - sta_track_length/2) <= track_length && ...
                    abs(tars_pos(k,2)- uavs_pos(i,2))<=track_width/2 % the targets are right the robot, just use ||x_r-x_t||
                     
                    target_cover{i,j}=[target_cover{i,j}, k]; % store the targets can be tracked if the distance is within tolerance
                 end
             end                 
             
             
         else % left direction
             
             for k = 1 : N_tars % check all the targets
                 if tars_pos(k,1) <= uavs_pos(i,1) + sta_track_length/2&& ...
                    uavs_pos(i,1) + sta_track_length/2  - tars_pos(k,1) <= track_length && ...
                    abs(tars_pos(k,2) - uavs_pos(i,2))<=track_width/2 % the targets are left the robot, just use ||x_r-x_t||
                     
                    target_cover{i,j}=[target_cover{i,j}, k]; % store the targets can be tracked if the distance is within tolerance
                 end
             end                              
             
         end
         
         
     n_target_cover(i,j) = length(target_cover{i,j});     
     end
     
     n_id_maxtra(i,1) = max(n_target_cover(i,:)); % find the number of targets covered by max_traj
     n_id_maxtra(i,2) = find(n_target_cover(i,:)==n_id_maxtra(i,1),1); %the id of max_traj
     
 end
end 