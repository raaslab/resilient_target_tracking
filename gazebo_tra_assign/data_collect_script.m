
%restore all the data. 
load('bf_50_aug24_18.mat')
store_remo_rate = zeros(4,50);
store_no_remo = zeros(4,50);
store_with_remo = zeros(4,50);
store_remo_rate(1,1:50) = store_remove_rate(1:50);
store_no_remo(1,1:50) = store_tar_before_remove(1:50);
store_with_remo(1,1:50) = store_tar_after_remove(1:50); 
%store_with_remo(1,1) = 10;
load('resilient_50_aug26_18_2.mat');
store_remo_rate(2,1:50) = store_remove_rate(1:50);
store_no_remo(2,1:50) = store_tar_before_remove(1:50);
store_with_remo(2,1:50) = store_tar_after_remove(1:50);
load('greedy_50_aug24_18.mat');
store_remo_rate(3,1:50) = store_remove_rate(1:50);
store_no_remo(3,1:50) = store_tar_before_remove(1:50);
store_with_remo(3,1:50) = store_tar_after_remove(1:50);
load('random_50_aug24_18.mat');
store_remo_rate(4,1:50) = store_remove_rate(1:50);
store_no_remo(4,1:50) = store_tar_before_remove(1:50);
store_with_remo(4,1:50) = store_tar_after_remove(1:50);
save('all_data_store_aug26.mat');