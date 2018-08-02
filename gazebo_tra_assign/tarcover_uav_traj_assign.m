%calculate the targets covered by assigned trajectory for each uav
function [n_tarcover] = tarcover_uav_traj_assign(target_cover, uav_tra_assign_inx)

           global N_uavs

           s_tarcover=cell(1, N_uavs+1); %the target set covered by this strategy
           for i = 1:N_uavs
           s_tarcover{i+1} = union(s_tarcover{i}, target_cover{uav_tra_assign_inx{i}(1),...
                                                               uav_tra_assign_inx{i}(2)});
           end
           n_tarcover = length(s_tarcover{N_uavs+1});         

end