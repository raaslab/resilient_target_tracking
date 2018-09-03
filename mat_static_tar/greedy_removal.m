% greedy removal
function [N_origin_cover, N_gre_remo_remain, gre_remo_rate]= greedy_removal(Nr, N_failure, tar_cover, ...
               r_tra_inx)
           
    
    %r_tra_inx gives the robot and its assigned trajectory. 
    %we need to select N_failure rounds for the greedy attack
    
    %keep a robot_trajectory copy for greedy attack use.
    gre_r_tra_inx = r_tra_inx;    
    %define gre_attack
    gre_attack_set = cell(1,N_failure + 1);
    %define robot attacked. 
    gre_r_attack = zeros(1,N_failure); 
   
    for r = 1 : N_failure % greedy needs N_failure rounds
        %define marginal_gain at each round
        marginal_gain = zeros(1,Nr-r+1);
        %search for all the possible cases. 
        for i = 1 : Nr-r+1
            marginal_gain(i) =  length(union(tar_cover{gre_r_tra_inx{i}(1), gre_r_tra_inx{i}(2)}, ...
                gre_attack_set{r})) - length(gre_attack_set{r});             
        end
        %find the maximum marginal gain
        [~, max_inx] = max(marginal_gain); 
        gre_attack_set{r+1} = tar_cover{gre_r_tra_inx{max_inx}(1), gre_r_tra_inx{max_inx}(2)};
        gre_r_attack(r) = gre_r_tra_inx{max_inx}(1); 
        gre_r_tra_inx(max_inx) = []; 
                        
    end
    
    s_tra_cover=cell(1,Nr+1); %the target covered by this strategy
    s_tra_cover_gre_remain=cell(1,Nr+1); %the target remaining by greedy revm strategy
    
    for i = 1:Nr
      
        s_tra_cover{i+1} = union(s_tra_cover{i}, tar_cover{r_tra_inx{i}(1),...
                                                               r_tra_inx{i}(2)});
        if ismember(r_tra_inx{i}(1), gre_r_attack) > 0
           s_tra_cover_gre_remain{i+1} = s_tra_cover_gre_remain{i}; 
        else
           s_tra_cover_gre_remain{i+1} = union(s_tra_cover_gre_remain{i}, tar_cover{r_tra_inx{i}(1),...
                                                                                        r_tra_inx{i}(2)});
        end
    end
    
  

    N_origin_cover = length(s_tra_cover{Nr+1}); 
    N_gre_remo_remain = length(s_tra_cover_gre_remain{Nr+1});    
    
    gre_remo_rate = (N_origin_cover - N_gre_remo_remain)/N_origin_cover; 
    
                  
end