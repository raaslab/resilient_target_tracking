%Timer: constantly receive the pos of all the models in the gazebo and pick the
%trajectory for each uav
clc;
%subscribe the gazebo pose topic to get the pos_info of each uav and each
%ground target
global T cnt

global N_uavs N_tars N_fail_uavs N_resilience_uavs

global uavs_pos 

global uavs_ini_pos %each uav has a initial pos which is used to calcualte the local pos diff

global N_dir_uav %1, up; 2, down; 3, right; 4 left. It has four directions

global track_length track_width fly_length sta_track_length

global uav_id_set

global gz_model_states_sub

global desired_pos_pub_uavs desired_pub_msg work_status_pub_uavs work_status_uavs

global uav1_state_sub uav2_state_sub uav3_state_sub uav4_state_sub uav5_state desired_pos_track

global err_offset
    
global tars_pos_true tars_pos_hat  tars_sigma  tars_pos_last  tars_t_last  tars_v

global store_tar_before_remove
global store_tar_after_remove
global store_remove_rate

%global desired_pos_global


N_uavs = 4; % number of uavs 
N_tars = 30; % number of targets

N_fail_uavs = 2; % number of failed uavs
N_resilience_uavs = N_uavs - N_fail_uavs; %number of resilient uavs

tars_pos_true = zeros(N_tars, 3); % x, y, z
tars_pos_hat = zeros(N_tars, 3);  %may have some problem!!!!!!
tars_pos_last = zeros(N_tars, 3);  %may have some problem!!!!!!
tars_t_last = zeros(N_tars, 1);
tars_v = zeros(N_tars, 3);
tars_sigma = zeros(2*N_tars, 2); 
for i =  1 : N_tars
    tars_sigma(2*i-1 :2*i, : )  = eye(2); 
end


err_offset = 1.2;

store_tar_before_remove = [];
store_tar_after_remove =[];
store_remove_rate =[];

%uavs_ini_pos = [0, -5, 2; 3, -2, 3; 5, 0, 4; -2, 5, 5; -5, 0, 6];
%uavs_ini_pos = [0, -5, 2; 5, 0, 3; -2, 5, 4; -5, 0, 5];
%uavs_ini_pos = [-3, -5, 2; -2, -5 3; -1,-5, 4]; 
uavs_ini_pos = [0, -5, 0; 5, 0, 0; -2, 5, 0; -5, 0, 0];

uavs_pos = zeros(N_uavs,3); %x, y ,z

desired_pos_track = [0, -5, 2; 5, 0, 3; -2, 5, 4; -5, 0, 5];

N_dir_uav = 4; % each uav has four trajectories it can choose

%assume the uav has a 3x2 rectangle when it is static
track_width  = 3; % tracking the target within the width along that direction
sta_track_length  = 3; 
fly_length = 3; 
track_length = fly_length + sta_track_length; % track the target within the length in that direction, static cover plus moving 

uav_id_set = zeros(1,N_uavs);



for i =1:N_uavs
    uav_id_set(i)=i;
end


%publish desired pos for each uav 
desired_pos_pub_uav1 = rospublisher('/uav1/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav2 = rospublisher('/uav2/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav3 = rospublisher('/uav3/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav4 = rospublisher('/uav4/desired_pos','geometry_msgs/Point');
%desired_pos_pub_uav5 = rospublisher('/uav5/desired_pos','geometry_msgs/Point');

desired_pos_pub_uavs = [desired_pos_pub_uav1, desired_pos_pub_uav2, ...
    desired_pos_pub_uav3, desired_pos_pub_uav4];

desired_pub_msg = rosmessage('geometry_msgs/Point'); 


%publish the working_status for each uav 
work_staus_pub_uav1 = rospublisher('/uav1/work_status', 'std_msgs/Int8');
work_staus_pub_uav2 = rospublisher('/uav2/work_status', 'std_msgs/Int8');
work_staus_pub_uav3 = rospublisher('/uav3/work_status', 'std_msgs/Int8');
work_staus_pub_uav4 = rospublisher('/uav4/work_status', 'std_msgs/Int8');

work_status_pub_uavs = [work_staus_pub_uav1, work_staus_pub_uav2, ...
    work_staus_pub_uav3, work_staus_pub_uav4]; 
work_status_uav_1  = rosmessage('std_msgs/Int8');
work_status_uav_2  = rosmessage('std_msgs/Int8');
work_status_uav_3  = rosmessage('std_msgs/Int8');
work_status_uav_4  = rosmessage('std_msgs/Int8');
work_status_uavs = [work_status_uav_1, work_status_uav_2, work_status_uav_3, work_status_uav_4]; 


%subscribe all the model states in the gazebo environment
gz_model_states_sub = rossubscriber('/gazebo/model_states');

%global markerPub marker
%markerPub = rospublisher('/visualization_marker','visualization_msgs/Marker');
%marker = rosmessage(markerPub);

%%%receive ros_gazebo service
% gazebo = ExampleHelperGazeboCommunicator();
% 
% uav1_state = ExampleHelperGazeboSpawnedModel('iris_1',gazebo);
% uav2_state = ExampleHelperGazeboSpawnedModel('iris_2',gazebo);
% uav3_state = ExampleHelperGazeboSpawnedModel('iris_3',gazebo);
% uav4_state = ExampleHelperGazeboSpawnedModel('iris_4',gazebo);
%uav5_state = ExampleHelperGazeboSpawnedModel('iris_5',gazebo);

%get the uav pos from mavros

uav1_state_sub = rossubscriber('/uav1/mavros/local_position/pose');
uav2_state_sub = rossubscriber('/uav2/mavros/local_position/pose');
uav3_state_sub = rossubscriber('/uav3/mavros/local_position/pose');
uav4_state_sub = rossubscriber('/uav4/mavros/local_position/pose');

% start the timer
T = 0.1; % timer period
cnt  = 0; 

t = timer('TimerFcn',{@traj_assign_fun},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);
