clc;
hold on; box on; 

Nt = 50; % the number of targets
Nr = 6; % the number of robots
r_set = zeros(1,Nr); % robot set
for i =1:Nr
    r_set(i)=i;
end

en_range=10; %the range of the environment.
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
% at robot's current position. The trajectory is enough long to come across the
% edge of the environment

%trajectory_targets();
N_direction = 4; % each robot has four directions, up, down, left, right
%tar_cover = cell(Nr, N_direction); % The targets can be covered for a specific robot with a choosing trajecotry
%N_tarcover=zeros(Nr, N_direction); % The number of targets covered for a specific robot with a choosing trajectory
epsilon = 0.15* en_range; % convering distance -2<-->+2 
%N_r_maxtra = zeros(Nr,1); % select the maximum coverage trajectory for each robot

[tar_cover, N_tarcover,N_r_maxtra]=robot_tra_cover_fun(Nr,Nt,N_direction,pt,pr, epsilon);

% calculate the non-resilient greedy set
s_greedy=cell(Nr+1,1);
s_greedy{1}=[];
s_greedy_index=cell(Nr,1);

remain_set = r_set ; 
 
for k = 1:Nr % we need to greedily pick Nr rounds
           resilience_thre=0;
           % for each round, search all possible robots and their
           % trajectories
           for i = 1:length(remain_set)
                 for j = 1:N_direction
                    marg_gain = length(union(tar_cover{remain_set(i),j},s_greedy{k})) - length(s_greedy{k}); 
                    if marg_gain >= resilience_thre
                        resilience_thre = marg_gain;
                        s_greedy_index{k}  = [remain_set(i),j];
                    end
                 end
           end       
          remain_set = setdiff(remain_set, s_greedy_index{k}(1));
          s_greedy{k+1} = union( s_greedy{k}, tar_cover{s_greedy_index{k}(1),s_greedy_index{k}(2)});
end   
 
% once we know the greedy set, use the best removal! 
% transform the index to 1 2 3 4. 5 6 7 8...

%calculate the targets tracked by this stratey 
[N_greedy_cover] =select_tra_cover(Nr, tar_cover, s_greedy_index); 

%best removal,
[gre_remain_best_remo, best_remo_rate_gre]= best_removal(Nr, N_direction, N_failure, tar_cover, ...
               s_greedy_index,N_greedy_cover);


