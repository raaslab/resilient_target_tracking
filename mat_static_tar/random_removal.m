%random removal
function [N_origin_cover, N_ran_remo_remain, ran_remo_rate]= random_removal(Nr, tar_cover, ...
               r_tra_inx, r_ran_remo)
           
    s_tra_cover=cell(1,Nr+1); %the target covered by this strategy
    s_tra_cover_ran_remain=cell(1,Nr+1); %the target remaining by random revm strategy
    for i = 1:Nr
      
        s_tra_cover{i+1} = union(s_tra_cover{i}, tar_cover{r_tra_inx{i}(1),...
                                                               r_tra_inx{i}(2)});
        if ismember(r_tra_inx{i}(1), r_ran_remo) > 0
           s_tra_cover_ran_remain{i+1} = s_tra_cover_ran_remain{i}; 
        else
           s_tra_cover_ran_remain{i+1} = union(s_tra_cover_ran_remain{i}, tar_cover{r_tra_inx{i}(1),...
                                                                                        r_tra_inx{i}(2)});
        end
    end

    N_origin_cover = length(s_tra_cover{Nr+1}); 
    N_ran_remo_remain = length(s_tra_cover_ran_remain{Nr+1});    
    
    ran_remo_rate = (N_origin_cover - N_ran_remo_remain)/N_origin_cover; 
    
                     
end