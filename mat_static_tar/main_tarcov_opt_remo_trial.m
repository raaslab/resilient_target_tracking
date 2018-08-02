% main_target_cover for comparing 30 trials for different number of targets
  clear all;
  min_num_tars  = 30;
  max_num_tars = 60;
  N_trials = 30;
  
  
  Nr = 6; % the number of robots
  en_range=10; % the range of the environment.
  
  N_failure = 3;
  N_resilience = Nr- N_failure;
  
  N_direction = 4; % each robot has four directions, up, down, left, right
  epsilon = 0.15* en_range; % convering distance -2<-->+2 
  
  
  r_set = zeros(1,Nr); % robot set
for i =1:Nr
    r_set(i)=i;
end
  
  for Nt = min_num_tars : max_num_tars
    for i = 1  : N_trials
        cla;
        pt=rand(2,Nt)*en_range;
        pr=rand(2,Nr)*en_range;
        
        [tar_cover, N_tarcover,N_r_maxtra,tra_r_index]=...
            robot_tra_cover_fun(Nr,Nt,N_direction,pt,pr, epsilon);
        

        
        %our algorithm, resilient_target_tracking
        [resi(i,Nt),resi_remain_bestremo(i,Nt),bestremo_rate_resi(i,Nt)] = resilient_opt_remo_fun(Nr,N_direction,...
                                           N_failure, N_resilience, r_set,tar_cover,N_r_maxtra, tra_r_index);       
        % brute_force
        [bf(i,Nt),bf_remain_bestremo(i,Nt),bestremo_rate_bf(i,Nt)] = bf_opt_remo_fun(Nr,N_direction,...
            N_failure,tar_cover); 
        % non-resilience greedy optimal removal 
        [gre(i,Nt),gre_remain_bestremo(i,Nt),bestremo_rate_gre(i,Nt)] = greedy_opt_remo_fun(Nr,N_direction,...
            N_failure,tar_cover,r_set); 
        % random optimal removal
        [ran(i,Nt),ran_remain_bestremo(i,Nt),bestremo_rate_ran(i,Nt)] = random_opt_remo_fun(Nr,N_direction,...
            N_failure,tar_cover); 
        
    end
  end
  
  

          figure; hold on;grid on;
          
          shadedErrorBar(min_num_tars:max_num_tars,bf(:,min_num_tars:max_num_tars),...
              {@mean,@std},'lineprops','-r','patchSaturation',0.33)

          shadedErrorBar(min_num_tars:max_num_tars,resi(:,min_num_tars:max_num_tars),...
              {@mean,@std},'lineprops',':b','patchSaturation',0.33);

          shadedErrorBar(min_num_tars:max_num_tars,gre(:,min_num_tars:max_num_tars),...
              {@mean,@std}, 'lineprops', '-.g','patchSaturation',0.33)
          
          shadedErrorBar(min_num_tars:max_num_tars,ran(:,min_num_tars:max_num_tars),...
              {@mean,@std}, 'lineprops', '--m','patchSaturation',0.33)
          
          title('comparison of the numebr of targets tracked  before removal','fontsize',12)
          legend('brute-force','resilient','greedy','random');
          xlabel('number of targets','fontsize',11)
          ylabel('coverage number','fontsize',11)

          
          figure; hold on;grid on;
          
          shadedErrorBar(min_num_tars:max_num_tars,bf_remain_bestremo(:,min_num_tars:max_num_tars),...
              {@mean,@std},'lineprops','-r','patchSaturation',0.33)

          shadedErrorBar(min_num_tars:max_num_tars,resi_remain_bestremo(:,min_num_tars:max_num_tars),...
              {@mean,@std},'lineprops',':b','patchSaturation',0.33);

          shadedErrorBar(min_num_tars:max_num_tars,gre_remain_bestremo(:,min_num_tars:max_num_tars),...
              {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
          
          shadedErrorBar(min_num_tars:max_num_tars,ran_remain_bestremo(:,min_num_tars:max_num_tars),...
              {@mean,@std}, 'lineprops', '--m','patchSaturation',0.33)
          
          title('comparison of the numebr of targets tracked after removal','fontsize',12)
          legend('brute-force','resilient','greedy','random');
          xlabel('number of targets','fontsize',11)
          ylabel('coverage number','fontsize',11)
         
          figure; hold on;grid on;
          
          shadedErrorBar(min_num_tars:max_num_tars,bestremo_rate_bf(:,min_num_tars:max_num_tars),...
              {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)

          shadedErrorBar(min_num_tars:max_num_tars,bestremo_rate_resi(:,min_num_tars:max_num_tars),...
              {@mean,@std},'lineprops',':b','patchSaturation',0.33);

          shadedErrorBar(min_num_tars:max_num_tars,bestremo_rate_gre(:,min_num_tars:max_num_tars),...
              {@mean,@std}, 'lineprops', '-.g','patchSaturation',0.33)
          
          shadedErrorBar(min_num_tars:max_num_tars,bestremo_rate_ran(:,min_num_tars:max_num_tars),...
              {@mean,@std}, 'lineprops', '--m','patchSaturation',0.33)
          
          title('comparison of removal rate','fontsize',12)
          legend('brute-force','resilient','greedy','random');
          xlabel('number of targets','fontsize',11)
          ylabel('coverage number','fontsize',11)          
          
%   for Nt = min_num_tars : max_num_tars
%       
%         %average_resilience
%         avg_resi(Nt) = mean(resi(:,Nt));
%         avg_resi_remain_bestremo(Nt) = mean(resi_remain_bestremo(:,Nt));
%         avg_bestremo_rate_resi(Nt) = mean(bestremo_rate_resi(:,Nt));
%         
%         %std_resilience
%         std_resi(Nt) = std(resi(:,Nt));
%         std_resi_remain_bestremo(Nt) = std(resi_remain_bestremo(:,Nt));
%         std_bestremo_rate_resi(Nt) = std(bestremo_rate_resi(:,Nt));
%         
%         
%         %average_bf
%         avg_bf(Nt) = mean(bf(:,Nt));
%         avg_bf_remain_bestremo(Nt) = mean(bf_remain_bestremo(:,Nt));
%         avg_bestremo_rate_bf(Nt) = mean(bestremo_rate_bf(:,Nt));
%         
%         %std_bf
%         std_bf(Nt) = std(bf(:,Nt));
%         std_bf_remain_bestremo(Nt) = std(bf_remain_bestremo(:,Nt));
%         std_bestremo_rate_bf(Nt) = std(bestremo_rate_bf(:,Nt));
%         
%         
%         
%         %average_greedy
%         avg_gre(Nt) = mean(gre(:,Nt));
%         avg_gre_remain_bestremo(Nt) = mean(gre_remain_bestremo(:,Nt));
%         avg_bestremo_rate_gre(Nt) = mean(bestremo_rate_gre(:,Nt));
%         
%         %std_greedy
%         std_gre(Nt) = std(gre(:,Nt));
%         std_gre_remain_bestremo(Nt) = std(gre_remain_bestremo(:,Nt));
%         std_bestremo_rate_gre(Nt) = std(bestremo_rate_gre(:,Nt));
%         
%         
%  
%        
%         %average_random
%         avg_ran(Nt) = mean(ran(:,Nt));
%         avg_ran_remain_bestremo(Nt) = mean(ran_remain_bestremo(:,Nt));
%         avg_bestremo_rate_ran(Nt) = mean(bestremo_rate_ran(:,Nt));
%         
%         %std_random
%         std_ran(Nt) = std(ran(:,Nt));
%         std_ran_remain_bestremo(Nt) = std(ran_remain_bestremo(:,Nt));
%         std_bestremo_rate_ran(Nt) = std(bestremo_rate_ran(:,Nt));        
%         
%               
% 
%   end
%   
%             

%         figure; hold on;          
%         errorbar(min_num_tars:max_num_tars,avg_bf(min_num_tars:max_num_tars),...
%             std_bf(min_num_tars:max_num_tars),'b');
%         errorbar(min_num_tars:max_num_tars,avg_resi(min_num_tars:max_num_tars),...
%             std_resi(min_num_tars:max_num_tars), 'r');
%         errorbar(min_num_tars:max_num_tars,avg_gre(min_num_tars:max_num_tars),...
%             std_gre(min_num_tars:max_num_tars),'m');
%         errorbar(min_num_tars:max_num_tars,avg_ran(min_num_tars:max_num_tars),...
%             std_ran(min_num_tars:max_num_tars),'k');
         
%         title('comparison of the numebr of targets tracked  without removal','fontsize',12)
%         legend('brute-force','resilient','greedy','random');
%         xlabel('number of targets','fontsize',11)
%         ylabel('coverage number','fontsize',11)
% 
%         
%         
%         figure; hold on;
%         errorbar(min_num_tars:max_num_tars,avg_bf_remain_bestremo(min_num_tars:max_num_tars),...
%             std_bf_remain_bestremo(min_num_tars:max_num_tars),'b');
%         errorbar(min_num_tars:max_num_tars,avg_resi_remain_bestremo(min_num_tars:max_num_tars),...
%             std_resi_remain_bestremo(min_num_tars:max_num_tars), 'r');
%         errorbar(min_num_tars:max_num_tars,avg_gre_remain_bestremo(min_num_tars:max_num_tars),...
%             std_gre_remain_bestremo(min_num_tars:max_num_tars),'m');
%         errorbar(min_num_tars:max_num_tars,avg_ran_remain_bestremo(min_num_tars:max_num_tars),...
%             std_ran_remain_bestremo(min_num_tars:max_num_tars),'k');
% 
%         
%         title('comparison of the numebr of targets tracked after removal','fontsize',12)
%         legend('brute-force', 'resilient', 'greedy','random');
%         xlabel('number of targets','fontsize',11)
%         ylabel('coverage number','fontsize',11)   
%   
%   
%         
%         figure; hold on;
%         errorbar(min_num_tars:max_num_tars,avg_bestremo_rate_bf(min_num_tars:max_num_tars),...
%             std_bestremo_rate_bf(min_num_tars:max_num_tars),'b');
%         errorbar(min_num_tars:max_num_tars,avg_bestremo_rate_resi(min_num_tars:max_num_tars),...
%             std_bestremo_rate_resi(min_num_tars:max_num_tars), 'r');
%         errorbar(min_num_tars:max_num_tars,avg_bestremo_rate_gre(min_num_tars:max_num_tars),...
%             std_bestremo_rate_gre(min_num_tars:max_num_tars),'m');
%         errorbar(min_num_tars:max_num_tars,avg_bestremo_rate_ran(min_num_tars:max_num_tars),...
%             std_bestremo_rate_ran(min_num_tars:max_num_tars),'k');
% 
%         
%         title('comparison of removal rate','fontsize',12)
%         legend('brute-force', 'resilient', 'greedy','random');
%         xlabel('number of targets','fontsize',11)
%         ylabel('coverage number','fontsize',11)   
        
  
  
  