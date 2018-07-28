function [tar_cover, N_tarcover,N_r_maxtra,tra_r_index] = robot_tra_cover_fun(Nr,Nt,N_direction,pt,pr, epsilon)
    
N_r_maxtra = zeros(Nr,1); % select the maximum coverage of it trajectories for each robot
tra_r_index = zeros(Nr,1); %  the max tra index
tar_cover = cell(Nr, N_direction); % The targets can be covered for a specific robot with a choosing trajecotry
N_tarcover=zeros(Nr, N_direction); % The number of targets covered for a specific robot with a choosing trajectory

    for i = 1: Nr
         for j = 1:N_direction


               if j ==1 % up_trajectory
                  for k = 1:Nt % check all the targets
                       if pt(2,k) >= pr(2,i) && abs(pr(1,i)-pt(1,k))<=epsilon % the targets are above the robot, just use ||x_r-x_t||
                           tar_cover{i,j}=[tar_cover{i,j}, k]; % store the targets can be tracked if the distance is within tolerance
                       end
                  end


               elseif j == 2 % down_trajectory 
                   for k = 1:Nt 
                       if pt(2,k) <= pr(2,i) && abs(pr(1,i)-pt(1,k))<=epsilon % targets are below, ||x_r-x_t||
                           tar_cover{i,j}=[ tar_cover{i,j}, k];
                       end
                   end



               elseif j == 3 % left_trajectory
                   for k = 1:Nt 
                       if pt(1,k) <= pr(1,i) && abs(pr(2,i)-pt(2,k))<=epsilon % targets are at left, ||y_r-y_t||
                           tar_cover{i,j}=[ tar_cover{i,j}, k];
                       end
                   end

               else % right-trajectory
                   for k = 1:Nt 
                       if pt(1,k) >= pr(1,i) && abs(pr(2,i)-pt(2,k))<=epsilon % targets are at right, ||y_r-y_t||
                           tar_cover{i,j}=[ tar_cover{i,j}, k];
                       end
                   end

               end

        N_tarcover(i,j) = length( tar_cover{i,j});
         end % j indicates the direction
         %pick the maximum trajecotry (Num_tarcover) and its corresponding targets for each robot 
         N_r_maxtra(i) = max(N_tarcover(i,:)); %maximum number of coveage targets for each robot
                                                                    %maximum directions 
         tra_r_index(i)= find(N_tarcover(i,:)==N_r_maxtra(i),1); 

         %r_max_tracover{i}= tar_cover{i, maxtra_index_j}; % store the maximum coverage for each robot

    end
    
end