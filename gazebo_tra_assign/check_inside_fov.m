function flag_in_fov = check_inside_fov(tar_pos_true)
            
               global N_uavs
               global uavs_pos
               
               global track_width  sta_track_length fly_length % tracking the target within the width along that direction
               
               flag_in_fov = 0; 
               
               for i = 1 : N_uavs
                   
                      if abs(uavs_pos(i,1) - tar_pos_true(1))<= track_width/2 && abs(uavs_pos(i,2) - tar_pos_true(2))<= sta_track_length/2 + fly_length
                          flag_in_fov = 1; 
                      elseif abs(uavs_pos(i,2) - tar_pos_true(2))<= track_width/2 && abs(uavs_pos(i,1) - tar_pos_true(1))<= sta_track_length/2 + fly_length
                          flag_in_fov = 1;     
                          
                      end
                   
               end
                
                          
end