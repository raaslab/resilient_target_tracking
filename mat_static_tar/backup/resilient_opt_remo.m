clc;
hold on; box on; 

Nt = 50; % the number of targets
Nr = 6; % the number of robots
r_set = zeros(1,Nr); % robot set
for i =1:Nr
    r_set(i)=i;
end

en_range=10; % the range of the environment.
N_failure = 2;
N_resilience = Nr- N_failure;

% generate the positions of targets
pt=rand(2,Nt)*en_range; 
sz = 100;
scatter(pt(1,:),pt(2,:), sz, 'r*')

% generate the positions of the robot
pr=rand(2,Nr)*en_range; 
sz = 100;
scatter(pr(1,:),pr(2,:), sz, 'bo', 'filled')

% for each robot, it has four trajectories, up, down, left, right, centered
% at robot's c urrent position. The trajectory is enough long to come across the
% edge of the environment

%trajectory_targets();
N_direction = 4; % each robot has four directions, up, down, left, right
%tar_cover = cell(Nr, N_direction); % The targets can be covered for a specific robot with a choosing trajecotry
%N_tarcover=zeros(Nr, N_direction); % The number of targets covered for a specific robot with a choosing trajectory
epsilon = 0.15* en_range; % convering distance -2<-->+2 
%N_r_maxtra = zeros(Nr,1); % select the maximum coverage trajectory for each robot
%r_max_tracover = cell(Nr, 1); % store the targets from maximum trajectory coverage for each robot

[tar_cover, N_tarcover,N_r_maxtra,tra_r_index]=robot_tra_cover_fun(Nr,Nt,N_direction,pt,pr, epsilon);
     
     %sort out the N_failure big failures_ specifc which robot fails
      r_fail_set=zeros(1,N_failure);
      N_r_temp_maxtra = N_r_maxtra; 
      r_tra_resilience_inx = cell(1,Nr);
      for nf = 1: N_failure
            r_fail_set(nf) = find(N_r_temp_maxtra==max(N_r_temp_maxtra),1);
            N_r_temp_maxtra(r_fail_set(nf))=0;
             r_tra_resilience_inx{nf} = [r_fail_set(nf), tra_r_index(r_fail_set(nf))]; % select the first N-failure robot-tra 
      end
      
     % greedy resilience from the remining robots
     % note that we calculate the curvature here.
       r_resilient_set=setdiff(r_set, r_fail_set);
       s_greedy=cell(1,N_resilience+1);
       %curvf = zeros(N_resilience,1);
       
     for k = 1:N_resilience
           resilience_thre=0;
         
           for i = 1:length(r_resilient_set)
                 for j = 1:N_direction
                    marg_gain = length(union(tar_cover{r_resilient_set(i),j},s_greedy{k})) - length(s_greedy{k}); 
                    if marg_gain >= resilience_thre
                        resilience_thre = marg_gain;
                        r_tra_resilience_inx{N_failure+k}  = [r_resilient_set(i),j];
                    end
                 end
           end    
           %curv(k)= resilience_thre/(length(tar_cover{r_tra_resilience_inx{N_failure+k}(1),r_tra_resilience_inx{N_failure+k}(2)}));
           r_resilient_set = setdiff(r_resilient_set, r_tra_resilience_inx{N_failure+k}(1));
           s_greedy{k+1} = union( s_greedy{k}, tar_cover{r_tra_resilience_inx{N_failure+k}(1),r_tra_resilience_inx{N_failure+k}(2)});
     end
     
          %curvf = 1 - min( curv);  % just among the resilience robots          
          %before removal, calculate the targets tracked by this strategy
          
          %calculate the targets tracked by this stratey 
           [N_resilience_cover] =select_tra_cover(Nr, tar_cover, r_tra_resilience_inx);     
                   
           %best removal,
           [resi_remain_best_remo, best_remo_rate_resi]= best_removal(Nr, N_direction, N_failure, tar_cover, ...
               r_tra_resilience_inx,N_resilience_cover);
            
          %greedy removal
          
          %random removal, 
            
          
          
          
        