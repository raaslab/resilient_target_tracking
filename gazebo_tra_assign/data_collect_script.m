
%restore all the data. 
load('bf.mat')
store_remo_rate = zeros(4,50);
store_no_remo = zeros(4,50);
store_with_remo = zeros(4,50);
store_remo_rate(1,1:50) = store_remove_rate(1:50);
store_no_remo(1,1:50) = store_tar_before_remove(1:50);
store_with_remo(1,1:50) = store_tar_after_remove(1:50); 
store_with_remo(1,1) = 9;
load('resilient.mat');
store_remo_rate(2,1:50) = store_remove_rate(1:50);
store_no_remo(2,1:50) = store_tar_before_remove(1:50);
store_with_remo(2,1:50) = store_tar_after_remove(1:50);
load('greedy.mat');
store_remo_rate(3,1:50) = store_remove_rate(1:50);
store_no_remo(3,1:50) = store_tar_before_remove(1:50);
store_with_remo(3,1:50) = store_tar_after_remove(1:50);
load('random.mat');
store_remo_rate(4,1:50) = store_remove_rate(1:50);
store_no_remo(4,1:50) = store_tar_before_remove(1:50);
store_with_remo(4,1:50) = store_tar_after_remove(1:50);
save('all_data_store1.mat');
