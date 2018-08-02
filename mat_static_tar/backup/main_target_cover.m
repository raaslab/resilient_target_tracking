clc; 

Nt = 50; % the number of targets
Nr = 6; % the number of robots
en_range=30; % the range of the environment.

N_failure = 2;
N_resilience = Nr- N_failure;

N_direction = 4; % each robot has four directions, up, down, left, right
epsilon = 0.15* en_range; % convering distance -2<-->+2 

N_trails= 10;

%curvf = zeros(N_trails,1);
resi = zeros(N_trails,1);
resi_remain_bestremo= zeros(N_trails,1);
bestremo_rate_resi = zeros(N_trails, 1);

bf = zeros(N_trails,1); 
bf_remain_bestremo = zeros(N_trails,1);
bestremo_rate_bf = zeros(N_trails,1);

gre = zeros(N_trails,1);
gre_remain_bestremo = zeros(N_trails,1);
bestremo_rate_gre = zeros(N_trails,1);

ran  = zeros(N_trails,1);
ran_remain_bestremo = zeros(N_trails,1);
bestremo_rate_ran = zeros(N_trails,1);

r_set = zeros(1,Nr); % robot set
for i =1:Nr
    r_set(i)=i;
end


for i = 1: N_trails 
% generate the positions of targets
pt=rand(2,Nt)*en_range; 
%sz = 100;
%scatter(pt(1,:),pt(2,:), sz, 'r*')

% generate the positions of the robot
pr=rand(2,Nr)*en_range; 
%sz = 100;
%scatter(pr(1,:),pr(2,:), sz, 'bo', 'filled')

% for each robot, it has four trajectories, up, down, left, right, centered
% at robot's current position. The trajectory is enough long to come across the
% edge of the environment

%trajectory_targets();

[tar_cover, N_tarcover,N_r_maxtra,tra_r_index]=robot_tra_cover_fun(Nr,Nt,N_direction,pt,pr, epsilon);

% our algorithm, resilient_target_tracking
[resi(i),resi_remain_bestremo(i),bestremo_rate_resi(i)] = resilient_tracking_fun(Nr,N_direction,...
    N_failure, N_resilience, r_set,tar_cover,N_r_maxtra, tra_r_index); 
% brute_force
[bf(i),bf_remain_bestremo(i),bestremo_rate_bf(i)] = brute_force_fun(Nr,N_direction,...
    N_failure,tar_cover); 
% non-resilience greedy optimal removal 
[gre(i),gre_remain_bestremo(i),bestremo_rate_gre(i)] = greedy_optremoval_fun(Nr,N_direction,...
    N_failure,tar_cover,r_set); 
% random optimal removal
[ran(i),ran_remain_bestremo(i),bestremo_rate_ran(i)] = random_optremoval_fun(Nr,N_direction,...
    N_failure,tar_cover); 
end
% figure(1);
% cla; hold on;box on;
% title('greedy converage w.r.t. curvature','fontsize',12)
% xlabel('curvature','fontsize',11)
% ylabel('coverage number','fontsize',11)
% plot(curvf, resilient_cover);
figure(1);
cla; hold on;box on;
title('comparison of the numebr of targets tracked in four cases without attacking','fontsize',12)
xlabel('trail','fontsize',11)
ylabel('coverage number','fontsize',11)
h1 = plot(bf);
h2 = plot(resi);
h3 = plot(gre);
h4 = plot(ran); 
legend( [h1 h2 h3 h4], 'brute-force', 'resilient', 'greedy','random');

figure(2);
cla; hold on;box on;
title('comparison of the numebr of targets tracked in four cases after attacking','fontsize',12)
xlabel('trail','fontsize',11)
ylabel('coverage number','fontsize',11)

h1 = plot(bf_remain_bestremo);
h2 = plot(resi_remain_bestremo);
h3 = plot(gre_remain_bestremo);
h4 = plot(ran_remain_bestremo);
legend( [h1 h2 h3 h4], 'brute-force', 'resilient', 'greedy','random');

figure(3);
cla; hold on;box on;
title('comparison of the attacking rate in four cases','fontsize',12)
xlabel('trail','fontsize',11)
ylabel('coverage number','fontsize',11)

h1 = plot(bestremo_rate_bf);
h2 = plot(bestremo_rate_resi);
h3 = plot(bestremo_rate_gre);
h4 = plot(bestremo_rate_ran);
legend( [h1 h2 h3 h4], 'brute-force', 'resilient', 'greedy','random');