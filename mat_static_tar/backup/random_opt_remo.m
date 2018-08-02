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

% calculate the randomly choosing set

s_random_index=cell(Nr,1);

% for each robot choose one trajectory randomly
     for i = 1:Nr   
          p = randperm(N_direction);  
          s_random_index{i}  = [i,p(1)];
     end       
 %calculate the targets tracked by this strategy.         
  [N_ran_cover] =select_tra_cover(Nr,tar_cover, s_random_index);
  %best removal.
  [ran_remain_best_remo, best_remo_rate_ran]= best_removal(Nr, N_direction, N_failure, tar_cover, ...
               s_random_index,N_ran_cover);
