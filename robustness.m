function [noise_data7, error7] = robustness(rotor_data, control_input, time, x0)

   
    noise_data7.state_noise_amp = 0.003;
    noise_data7.output_noise_amp = 0.02; 
    
   
    diverged = false; 
    count = 0; 

  
    while ~diverged
       
        [~, ~, error7] = simulation_quadrotor(rotor_data, control_input, noise_data7, time, x0);
        
       
        if error7.rmse_states(1,1) > 5
            count = count + 1;
        else
            count = 0;  
        end
        
        if error7.rmse_states(1,2) > 0.2
            count = count + 1;
        else
            count = 0; 
        end
        
        if error7.rmse_states(1,3) > 5
            count = count + 1;
        else
            count = 0; 
        end

        if error7.rmse_states(1,4) > 0.2
            count = count + 1;
        else
            count = 0; 
        end

        if count >= 4
            diverged = true;  
        else
           
            noise_data7.state_noise_amp = noise_data7.state_noise_amp + 0.003;
            noise_data7.output_noise_amp = noise_data7.output_noise_amp + 0.02;
        end


        disp(error7.rmse_states)
    end

    % Mostrar en qué nivel de ruido se detectó la divergencia
    disp('Divergence detected at:');
    disp(noise_data7.state_noise_amp);
    disp(noise_data7.output_noise_amp);
end
