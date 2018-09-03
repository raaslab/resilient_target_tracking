function [N_resilience_cover, resi_remain_gre_remo, gre_remo_rate_resi] = resilient_gre_remo_fun(Nr,N_direction,...
    N_failure, N_resilience, r_set,tar_cover,N_r_maxtra,tra_r_max_inx)

     
     %sort out the N_failure big failures_ specifc which robot fails
      r_fail_set=zeros(1,N_failure);
      N_r_temp_maxtra = N_r_maxtra;
      r_tra_resilience_inx = cell(1,Nr);
      %curvf = zeros(1,Nr);
      
      for nf = 1: N_failure % sorting big two
            r_fail_set(nf) = find(N_r_temp_maxtra==max(N_r_temp_maxtra),1);
            N_r_temp_maxtra(r_fail_set(nf))=0;
            r_tra_resilience_inx{nf} = [r_fail_set(nf), tra_r_max_inx(r_fail_set(nf))];
      end
      
     % greedy resilience from the remining robots
     % note that we calculate the curvature here.
       r_resilient_set=setdiff(r_set, r_fail_set);
       r_greedy_resi=cell(N_resilience+1,1);
             
     for k = 1:N_resilience
           resilience_thre=0;
         
           for i = 1:length(r_resilient_set)
                 for j = 1:N_direction
                    marg_gain = length(union(tar_cover{r_resilient_set(i),j},r_greedy_resi{k})) - length(r_greedy_resi{k}); 
                    if marg_gain >= resilience_thre
                        resilience_thre = marg_gain;
                        r_tra_resilience_inx{N_failure+k}  = [r_resilient_set(i),j];
                    end
                 end
           end    
           %curvf(k)= resilience_thre/(length(tar_cover{r_tra_resilience_inx{N_failure+k}(1),r_tra_resilience_inx{N_failure+k}(2)}));
           r_resilient_set = setdiff(r_resilient_set, r_tra_resilience_inx{N_failure+k}(1));
           r_greedy_resi{k+1} = union( r_greedy_resi{k}, tar_cover{r_tra_resilience_inx{N_failure+k}(1),r_tra_resilience_inx{N_failure+k}(2)});
     end
         
          %resilient_cover = length(r_greedy{N_resilience+1});
          %curvf = 1 - min(curvf); %it is defined
            
          %calculate the targets tracked by this strategy.
          % [N_resilience_cover] =select_tra_cover(Nr, tar_cover, r_tra_resilience_inx); 
           %best removal.
%            [resi_remain_best_remo, best_remo_rate_resi]= best_removal(Nr, N_direction, N_failure, tar_cover, ...
%                r_tra_resilience_inx,N_resilience_cover);
           %greedy removal.
           
           %random removal. 
           [N_resilience_cover, resi_remain_gre_remo, gre_remo_rate_resi]= greedy_removal(Nr, N_failure, tar_cover, ...
               r_tra_resilience_inx);
           
          
          
% define the epsilon as the creteria of the range that a target can be tracked
% by the trajectory. i.e., if the distance between the target and the
% trajectory is less than epsilon
end