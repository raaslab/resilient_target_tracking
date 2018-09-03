% greedy target assign algorithm
% 1. calculate the number of targets tracked by the resilient algorithm and
% the number of targets after best removal
% 2. assign each uav an resilient trajectory
% 3. publish desired pos message for each uav(same as 2. assign trajectory)
function greedy_traj_assign(target_cover)
            
        global N_uavs N_dir_uav

        global uav_id_set

        traj_assign = zeros(N_uavs,1); % each uav has a assigned traj for publisher control
        % calculate the non-resilient greedy set
        s_greedy=cell(N_uavs+1,1);
        s_greedy_index=cell(N_uavs,1);

        remain_set = uav_id_set ; 

        for k = 1:N_uavs % we need to greedily pick N_uavs rounds
                   resilience_thre=0;
                   % for each round, search all possible robots and their
                   % trajectories
                   for i = 1:length(remain_set)
                         for j = 1:N_dir_uav
                            marg_gain = length(union(target_cover{remain_set(i),j},s_greedy{k})) - length(s_greedy{k}); 
                            if marg_gain >= resilience_thre
                                resilience_thre = marg_gain;
                                s_greedy_index{k}  = [remain_set(i),j];
                                
                            end
                         end
                   end
                   
                  traj_assign(s_greedy_index{k}(1)) = s_greedy_index{k}(2);
                  remain_set = setdiff(remain_set, s_greedy_index{k}(1));
                  s_greedy{k+1} = union( s_greedy{k}, target_cover{s_greedy_index{k}(1),s_greedy_index{k}(2)});
        end   

            %calculate the targets tracked by this stratey 
            [N_greedy_cover] =tarcover_uav_traj_assign(target_cover, s_greedy_index); 

            %best removal,
            [gre_remain_best_remo, best_remo_rate_gre]= uav_best_removal(target_cover, s_greedy_index, N_greedy_cover);
            
             %desired pos_publish, keep publishing desired for each uav. 
             %collect the targets tracked before removal, after removal and the
             %removal rate when publish to uavs_greedy_index
            desired_pos_publisher(traj_assign, N_greedy_cover, gre_remain_best_remo, best_remo_rate_gre); 
                        
            
            
end