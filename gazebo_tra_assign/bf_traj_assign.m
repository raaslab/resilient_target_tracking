% brute-force target assign algorithm
% 1. calculate the number of targets tracked by the resilient algorithm and
% the number of targets after best removal
% 2. assign each uav an resilient trajectory
% 3. publish desired pos message for each uav(same as 2. assign trajectory)

function bf_traj_assign(target_cover)
        
        global N_uavs N_dir_uav
        global N_fail_uavs 
        
        global opt_attacked_uavs
        
        opt_attacked_uavs = zeros(1, N_fail_uavs); 
        
        % we pick one trajectory for each robot, and enumerate all the
        % possible cases, 4^6. And index the four trajectories for robot 1 ,2 ...,
        % 6 as (1 2 3 4) (5 6 7 8) (9 10 11 12)...  and find its corresponding
        % (1,1) (1,2) (1,3), (1,4). (2,1) (2,2) (2,3) (2,4) .... 
        traj_assign = zeros(N_uavs,1); % each uav has a assigned traj for publisher control
        r_tra_index = zeros(N_uavs, N_dir_uav);

        for i =1 :N_uavs
            r_tra_index(i,:)=[(i-1)*N_dir_uav+1, (i-1)*N_dir_uav+2, (i-1)*N_dir_uav+3, (i-1)*N_dir_uav+4];
        end
        % find four trajectory index for each robot
        % find all the combinations by pick one trajectory from four trajecotries
        % of each robot
        all_combina = r_tra_index(1,:);
        for i = 2:N_uavs
            all_combina = combvec(all_combina, r_tra_index(i,:));
        end

        %inside all_combina, the number of column indicates all possible cases and
        %each column indicates one combination
        [row_allcomb, col_allcomb] = size(all_combina);

        % target_cover for all combinations
        tar_cover_allcomb = zeros(1,col_allcomb);

        % for each combination (column), choose N_failure best failures, it means
        % the remaining is the minimum (best removal)

        % define all possible cases for choosing N_failure failures
         %num_nrchoosefail= nchoosek(row_allcomn,N_failure);
        nchoose_kfail_index = cell(1,col_allcomb); 
        remain_after_kfail_index = cell(1,col_allcomb);
        num_remain_after_kfail = cell(1, col_allcomb); 
        % remaining target set
        %remain_after_kfail = cell(1,col_allcomb); 
        mini_remain_eachcomb = zeros(1, col_allcomb); 
        mini_index_eachcomb = zeros(1, col_allcomb); 
        
        for i = 1:col_allcomb
           %we also want to get the number of targets tracked before the best
           %removal % calculate the targets tracked by each column combination
           % r_tra_set for each combination
           r_tra_index_eachcomb = cell(1, row_allcomb);

           %before removal
           for s = 1: row_allcomb
               %we also want to get the number of targets tracked before the best
               %removal % calculate the targets tracked by each column combination

               r_inx_eachcomb = fix((all_combina(s,i)-1)/N_dir_uav)+1; 
               tra_inx_eachcomb = mod(all_combina(s,i)-1, N_dir_uav)+1;
               r_tra_index_eachcomb{s} = [r_inx_eachcomb, tra_inx_eachcomb]; 
               
           end
           [tar_cover_allcomb(i)] = tarcover_uav_traj_assign(target_cover, r_tra_index_eachcomb);           
           
           %calculate the all possible removal inside on combination
           nchoose_kfail_index{i} = nchoosek(all_combina(:,i),N_fail_uavs); 
           [row_nk, ~] = size(nchoose_kfail_index{i});
           for j = 1: row_nk
               remain_after_kfail_index{i}(j,:)= setdiff(all_combina(:,i)', nchoose_kfail_index{i}(j,:)); 
               % for each remaining, evaluate which one is minimum
               % calculate the number of tracking for the reamining
               [~, col_remian] = size( remain_after_kfail_index{i}(j,:)); 

               temp_remain_after_kfail = cell(1, col_remian+1);
                for k =1:col_remian % union remaining set by index
                     % corresping to whcih robot and which traj
                       r_remain_index =fix((remain_after_kfail_index{i}(j,k)-1)/N_dir_uav)+1; 
                       tra_remain_index =mod(remain_after_kfail_index{i}(j,k)-1, N_dir_uav)+1;
                       temp_remain_after_kfail{k+1} = union(target_cover{r_remain_index,tra_remain_index},...
                           temp_remain_after_kfail{k});
                end
                 num_remain_after_kfail{i}(j,:) = length(temp_remain_after_kfail{col_remian+1}); 
            end
           % calculate the number of targets tracked by each combination

           [mini_remain_eachcomb(i), mini_index_eachcomb(i)] = min(num_remain_after_kfail{i}); 
        end      
        
        bf_remain_best_romo = max(mini_remain_eachcomb); 
        maxi_index = find(bf_remain_best_romo== mini_remain_eachcomb,1);
       %[bf_remain_best_romo, maxi_index] = max(mini_remain_eachcomb); 
       
       % the uav removed are maxi_index  of nchoose_kfail_index and
       % mini_index_eachcomb(maxi_index)  item. 
        for i = 1 : N_fail_uavs
           opt_attacked_uavs(i) = fix((nchoose_kfail_index{maxi_index}(mini_index_eachcomb(maxi_index), i)-1)/N_dir_uav)+1; 
        end 

        % there are several equal ones... this has some problems, 
        % the best_removal_rate making sense in this scenario

        s_tra_cover_bf=cell(1,N_uavs+1); %the target covered by this strategy
        
        % for each robot, find the assigned trajectory and number of
        % targets tracked. 
        for i = 1:N_uavs
        robot_inx= fix((all_combina(i,maxi_index)-1)/N_dir_uav)+1;
        tra_inx =mod(all_combina(i,maxi_index)-1, N_dir_uav)+1; 
        traj_assign(robot_inx) = tra_inx; 
        s_tra_cover_bf{i+1} = union(s_tra_cover_bf{i}, target_cover{robot_inx,tra_inx});
        end
        
        N_bf_maxmin_cover = length(s_tra_cover_bf{N_uavs+1});
        best_remo_rate_bf =(N_bf_maxmin_cover-bf_remain_best_romo)/N_bf_maxmin_cover; 
        N_bf_max_cover = max(tar_cover_allcomb);
        
        desired_pos_publisher(traj_assign, N_bf_max_cover, bf_remain_best_romo, best_remo_rate_bf); 
        
end