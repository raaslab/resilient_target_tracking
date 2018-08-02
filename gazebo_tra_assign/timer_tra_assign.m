%Timer: constantly receive the pos of all the models in the gazebo and pick the
%trajectory for each uav
clc;
%subscribe the gazebo pose topic to get the pos_info of each uav and each
%ground target
global N_uavs N_targets N_fail_uavs N_resilience_uavs

global uavs_pos targets_pos

global uavs_ini_pos %each uav has a initial pos which is used to calcualte the local pos diff

global N_dir_uav %1, up; 2, down; 3, right; 4 left. It has four directions

global track_length track_width fly_length

global uav_id_set

global gz_model_states_sub

global desired_pos_pub_uavs desired_pub_msg

global uav1_state_sub uav2_state_sub uav3_state_sub uav4_state_sub uav5_state desired_pos_track

global err_offset
    


global store_tar_before_remove
global store_tar_after_remove
global store_remove_rate

%global desired_pos_global


N_uavs = 4; % number of uavs 
N_targets = 30; % number of targets

N_fail_uavs = 2; % number of failed uavs
N_resilience_uavs = N_uavs - N_fail_uavs; %number of resilient uavs

targets_pos = zeros(N_targets, 3); % x, y, z

err_offset = 2;

store_tar_before_remove = [];
store_tar_after_remove =[];
store_remove_rate =[];

%uavs_ini_pos = [0, -5, 2; 3, -2, 3; 5, 0, 4; -2, 5, 5; -5, 0, 6];
%uavs_ini_pos = [0, -5, 2; 5, 0, 3; -2, 5, 4; -5, 0, 5];
%uavs_ini_pos = [-3, -5, 2; -2, -5 3; -1,-5, 4]; 
uavs_ini_pos = [0, -5, 0; 5, 0, 0; -2, 5, 0; -5, 0, 0];

uavs_pos = zeros(N_uavs,3); %x, y ,z

desired_pos_track = [0, -5, 2; 5, 0, 3; -2, 5, 4; -5, 0, 5];

N_dir_uav = 4; % each uav hasro four trajectories it can choose

track_length = 5; % track the target within the length in that direction
track_width  = 2; % trackithe target within the width along that direction
fly_length = 3; 

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


%subscribe all the model states in the gazebo environment
gz_model_states_sub = rossubscriber('/gazebo/model_states');



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

t = timer('TimerFcn',{@traj_assign_fun},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);
