function [N_ran_cover, ran_remain_best_remo, best_remo_rate_ran] = random_opt_remo_fun(Nr,N_direction,...
    N_failure,tar_cover)

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
end