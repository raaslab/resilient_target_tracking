% resilient target assign algorithm
% 1. calculate the number of targets tracked by the resilient algorithm and
% the number of targets after best removal
% 2. assign each uav an resilient trajectory
% 3. publish desired pos message for each uav(same as 2. assign trajectory)
function resilient_traj_assign(target_cover, n_id_maxtra)

    global N_uavs N_dir_uav

    global N_fail_uavs N_resilience_uavs

    global uav_id_set
    
    traj_assign = zeros(N_uavs,1); % each uav has a assigned traj for publisher control
    uav_tra_assign_inx = cell(1, N_uavs);

    % sort out N_fail_uavs largest covering and its traj 
    % 1st step of resilient algorithm
    uav_fail_set=zeros(1,N_fail_uavs); 
    n_id_maxtra_store = n_id_maxtra;
    
    
    for nf = 1 : N_fail_uavs        
        uav_fail_set(nf) = find(n_id_maxtra_store(:,1)==max(n_id_maxtra_store(:,1)),1); % find the maximum one     
        n_id_maxtra_store(uav_fail_set(nf),1)=0; % set the maximum one to 0
        
        traj_assign(uav_fail_set(nf),1) = n_id_maxtra(uav_fail_set(nf), 2); % find the assign traj for uav,uav_fail_set(nf)  
        uav_tra_assign_inx{nf} = [uav_fail_set(nf), n_id_maxtra(uav_fail_set(nf), 2)];        
    end 
    


     % greedy resilience from the remining uavs
     
    uav_resilient_set=setdiff(uav_id_set, uav_fail_set);
    uav_greedy_set=cell(N_resilience_uavs+1,1);
    
    for k = 1:N_resilience_uavs
           
          resilience_thre=0;         
          for i = 1 : length(uav_resilient_set)
                for j = 1 : N_dir_uav
                   marg_gain = length(union(target_cover{uav_resilient_set(i),j},uav_greedy_set{k})) - length(uav_greedy_set{k}); 
                   if marg_gain >= resilience_thre
                       resilience_thre = marg_gain;
                       
                       traj_assign(uav_resilient_set(i),1)=j;
                       uav_tra_assign_inx{N_fail_uavs + k}  = [uav_resilient_set(i),j];
                   end
                end
          end
          
          uav_resilient_set = setdiff(uav_resilient_set, uav_tra_assign_inx{N_fail_uavs+k}(1));
          uav_greedy_set{k+1} = union(uav_greedy_set{k}, target_cover{uav_tra_assign_inx{N_fail_uavs+k}(1),...
                                                                       uav_tra_assign_inx{N_fail_uavs+k}(2)});
    end   
   
  %calculate the targets tracked by uav_traj_assignment strategy
   [n_assign_cover_resi] = tarcover_uav_traj_assign(target_cover, uav_tra_assign_inx); 
  
   %best removal.
   [remain_best_remo_resi, best_remo_rate_resi]= uav_best_removal(target_cover, ...
                                                                  uav_tra_assign_inx, n_assign_cover_resi);   
  %desired pos_publish, keep publishing desired for each uav. 
  %collect the targets tracked before removal, after removal and the
  %removal rate when publish to uav
   desired_pos_publisher(traj_assign, n_assign_cover_resi, remain_best_remo_resi, best_remo_rate_resi); 
end