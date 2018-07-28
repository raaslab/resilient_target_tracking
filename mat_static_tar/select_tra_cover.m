function [N_s_tra_cover] =select_tra_cover(Nr,tar_cover, r_tra_inx)
           s_tra_cover=cell(1,Nr+1); %the target covered by this strategy
           for i = 1:Nr
           s_tra_cover{i+1} = union(s_tra_cover{i}, tar_cover{r_tra_inx{i}(1),r_tra_inx{i}(2)});
           end
           N_s_tra_cover = length(s_tra_cover{Nr+1});           
end