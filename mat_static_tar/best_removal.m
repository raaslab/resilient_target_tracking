function [remain_best_remo, best_remo_rate]= best_removal(Nr,N_direction, N_failure, tar_cover,...
    r_tra_inx_origin,N_cover)

            r_tra_index = zeros(1,Nr); % give each robot_trajectory pair an index
            for i = 1:Nr
                r_tra_index(i) = (r_tra_inx_origin{i}(1)-1)*N_direction + r_tra_inx_origin{i}(2); 
            end
            % choose N_failure out of r_tra_index
            nchoose_kfail_index = nchoosek(r_tra_index,N_failure); 
            [raw_nchokfail, col_nchokfail] = size(nchoose_kfail_index); 
             remain_after_kfail_index = zeros(raw_nchokfail, Nr-col_nchokfail); 
             remain_after_kfail = cell(1, raw_nchokfail); 
             num_remain_after_kfail = zeros(1,raw_nchokfail);

            for i = 1:raw_nchokfail
                remain_after_kfail_index(i,:) = setdiff(r_tra_index, nchoose_kfail_index(i,:));
                [~, col_remian] = size( remain_after_kfail_index(i,:));

                temp_remain_after_kfail = cell(1, col_remian+1);
                for j = 1: col_remian
                    % corresping to whcih robot and which traj
                           r_remain_index =fix((remain_after_kfail_index(i,j)-1)/N_direction)+1; 
                           tra_remain_index =mod(remain_after_kfail_index(i,j)-1, N_direction)+1;
                           temp_remain_after_kfail{j+1} = union(tar_cover{r_remain_index,tra_remain_index},...
                               temp_remain_after_kfail{j});
                end
                 remain_after_kfail{i} = temp_remain_after_kfail{col_remian+1};
                 num_remain_after_kfail(i) = length(remain_after_kfail{i}); 
            end
            remain_best_remo = min(num_remain_after_kfail);
            best_remo_rate = (N_cover-remain_best_remo)/N_cover;
end