% best removal algorithm
% 1, calculate the targets remained after best removal for uav_traj assign
% 2. calculate the removing rate after knowing the total tar_cover & remain
function [remain_best_remo, best_remo_rate]= uav_best_removal(tar_cover,r_tra_inx_origin, n_cover)

        global N_uavs N_dir_uav 
        
        global N_fail_uavs  
        
        global opt_attacked_uavs
        
        opt_attacked_uavs = zeros(1, N_fail_uavs); 

        r_tra_index = zeros(1,N_uavs); % give each robot_trajectory pair an index
        for i = 1:N_uavs
            r_tra_index(i) = (r_tra_inx_origin{i}(1)-1)*N_dir_uav + r_tra_inx_origin{i}(2); 
        end
        % choose N_failure out of r_tra_index
        nchoose_kfail_index = nchoosek(r_tra_index,N_fail_uavs); 
        [row_nchokfail, col_nchokfail] = size(nchoose_kfail_index); 
         remain_after_kfail_index = zeros(row_nchokfail, N_uavs-col_nchokfail); 
         remain_after_kfail = cell(1, row_nchokfail); 
         num_remain_after_kfail = zeros(1,row_nchokfail);

        for i = 1:row_nchokfail
            remain_after_kfail_index(i,:) = setdiff(r_tra_index, nchoose_kfail_index(i,:));
            [~, col_remian] = size( remain_after_kfail_index(i,:));

            temp_remain_after_kfail = cell(1, col_remian+1);
            for j = 1: col_remian
                % corresping to whcih robot and which traj
                       r_remain_index =fix((remain_after_kfail_index(i,j)-1)/N_dir_uav)+1; 
                       tra_remain_index =mod(remain_after_kfail_index(i,j)-1, N_dir_uav)+1;
                       temp_remain_after_kfail{j+1} = union(tar_cover{r_remain_index,tra_remain_index},...
                           temp_remain_after_kfail{j});
            end
             remain_after_kfail{i} = temp_remain_after_kfail{col_remian+1};
             num_remain_after_kfail(i) = length(remain_after_kfail{i}); 
        end
        %find the inx of the row that is chosen 
        [remain_best_remo, min_inx] = min(num_remain_after_kfail);
        %once we know the min_inx, we know the corresponding row and
        %finding the removed robots
        for i = 1 : N_fail_uavs
           opt_attacked_uavs(i) = fix((nchoose_kfail_index(min_inx, i)-1)/N_dir_uav)+1; 
        end 
        
        best_remo_rate = (n_cover-remain_best_remo)/n_cover;
        
end