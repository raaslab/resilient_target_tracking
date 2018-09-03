% update the position of the target by Kalman filter, assume the targets
% has a linear dynamic model
function [tar_pos_hat1, tar_sigma1, tar_pos_last1, tar_v1]  = KF_tar_update(tar_pos_true, tar_pos_hat, tar_sigma, tar_pos_last, t_last, t_current)
              
             % rng(t);
             %measurement noise 
             global T
              measure_noise =  normrnd(0, 0.5) * ones(1,2);
              
              % covariance prediction noise 
              Q = 0.2*eye(2);
              
              % covariance measurement noise
              R = 0.2^2 * eye(2);        
              
              %create the measurement model 
              tar_pos_measure = tar_pos_true + measure_noise; 
                            
              tar_pos_last1 = tar_pos_measure; 

              tar_v1 = (tar_pos_measure - tar_pos_last)/(t_current - t_last);   
              
              % prediction model with new tar_v1; 
              tar_hat_pro = tar_pos_hat +  tar_v1 * T; 
              sigma_hat_pro= tar_sigma + Q;
              
              S = sigma_hat_pro + R;
              K = sigma_hat_pro * inv(S); 
              
              % update model
              tar_pos_hat1  = tar_hat_pro + (tar_pos_measure - tar_hat_pro) * K;     
              tar_sigma1 = (eye(2) - K) * sigma_hat_pro;  
end