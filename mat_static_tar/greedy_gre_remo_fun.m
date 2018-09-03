function [N_greedy_cover, gre_remain_gre_remo, gre_remo_rate_gre] = greedy_gre_remo_fun(Nr,N_direction,...
    N_failure,tar_cover,r_set)

        % calculate the non-resilient greedy set
        s_greedy=cell(Nr+1,1);
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

            %calculate the targets tracked by this stratey 
            %[N_greedy_cover] =select_tra_cover(Nr, tar_cover, s_greedy_index); 

            %best removal,
%             [gre_remain_best_remo, best_remo_rate_gre]= best_removal(Nr, N_direction, N_failure, tar_cover, ...
%                            s_greedy_index,N_greedy_cover);

            %random removal            
            [N_greedy_cover, gre_remain_gre_remo, gre_remo_rate_gre]= greedy_removal(Nr, N_failure, tar_cover, ...
               s_greedy_index);
end