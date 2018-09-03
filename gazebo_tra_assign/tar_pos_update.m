%update the pos of target at the next step, 

% we give each target a last seen time t_last and last seen pos,
% tar_pos_last to update its estimate velocity

function [tar_pos_hat1, tar_sigma1, tar_pos_last1, t_last1, tar_v1] =  ...
    tar_pos_update(tar_pos_true, tar_pos_hat, tar_sigma,  tar_pos_last, t_last, t_current, tar_v)
            
           % use true pos to check
           %if the target is not inside the field of view of the robot, just
           %use prediction
           global T
                       
          flag_in_fov = check_inside_fov(tar_pos_true);   
            
          if flag_in_fov == 0  
            tar_v1 = tar_v;   
            tar_pos_hat1 = tar_pos_hat + tar_v1 * T; 
            t_last1 = t_last; 
            tar_pos_last1 = tar_pos_last; 
            tar_sigma1 = tar_sigma; 
           
          else  
            %if the target is inside the field of view of the robot, use
            %update the tar_v; the velocity model
            t_last1 = t_current;         
            %update the pos by KF;         
            [tar_pos_hat1, tar_sigma1, tar_pos_last1, tar_v1]  = ...
                KF_tar_update(tar_pos_true, tar_pos_hat, tar_sigma, tar_pos_last, t_last, t_current);             
          end               
end