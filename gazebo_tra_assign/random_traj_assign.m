% resilient target assign algorithm
% 1. calculate the number of targets tracked by the resilient algorithm and
% the number of targets after best removal
% 2. assign each uav an resilient trajectory
% 3. publish desired pos message for each uav(same as 2. assign trajectory)
function random_traj_assign(target_cover)      
    
    global N_uavs N_dir_uav
    
    traj_assign = zeros(N_uavs,1); % each uav has a assigned traj for publisher control
    % calculate the randomly choosing set
    s_random_index=cell(N_uavs,1);

    % for each robot choose one trajectory randomly
               for i = 1:N_uavs   
                   p = randperm(N_dir_uav);  
                   s_random_index{i}  = [i,p(1)];
                   traj_assign(i) = p(1);
               end       

    %calculate the targets tracked by this strategy.         
    [N_ran_cover] =tarcover_uav_traj_assign(target_cover, s_random_index);
    %best removal.
    [ran_remain_best_remo, best_remo_rate_ran]= uav_best_removal(target_cover, s_random_index, N_ran_cover); 
    
    %desired pos_publish, keep publishing desired for each uav. 
    %collect the targets tracked before removal, after removal and the
    %removal rate when publish to uav
    desired_pos_publisher(traj_assign, N_ran_cover, ran_remain_best_remo, best_remo_rate_ran); 
                               
end