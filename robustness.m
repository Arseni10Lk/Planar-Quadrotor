function [noise_data_all, error_all] = robustness(rotor_data, control_input, time, x0)

    % Case 1: Times 0.5
    noise_data1.state_noise_amp = 0.0015;
    noise_data1.output_noise_amp = 0.01;
    [~, ~, error1] = simulation_quadrotor(rotor_data, control_input, noise_data1, time, x0);

    % Case 2: Times 1
    noise_data2.state_noise_amp = 0.003;
    noise_data2.output_noise_amp = 0.02;
    [~, ~, error2] = simulation_quadrotor(rotor_data, control_input, noise_data2, time, x0);

    % Case 3: Times 5
    noise_data3.state_noise_amp = 0.015;
    noise_data3.output_noise_amp = 0.1;
    [~, ~, error3] = simulation_quadrotor(rotor_data, control_input, noise_data3, time, x0);

    % Case 4: Times 10
    noise_data4.state_noise_amp = 0.03;
    noise_data4.output_noise_amp = 0.2;
    [~, ~, error4] = simulation_quadrotor(rotor_data, control_input, noise_data4, time, x0);

    % Case 5: No process / high measurement
    noise_data5.state_noise_amp = 0.00;
    noise_data5.output_noise_amp = 0.2;
    [~, ~, error5] = simulation_quadrotor(rotor_data, control_input, noise_data5, time, x0);

    % Case 6: High process / no measurement
    noise_data6.state_noise_amp = 0.03;
    noise_data6.output_noise_amp = 0.0;
    [~, ~, error6] = simulation_quadrotor(rotor_data, control_input, noise_data6, time, x0);

    % Case 7: Divergence detection
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
    end

    disp('Divergence detected at:');
    disp(noise_data7.state_noise_amp);
    disp(noise_data7.output_noise_amp);


    % Collect all results
    noise_data_all = struct( 'noise_data1', noise_data1, ...
                              'noise_data2', noise_data2, ...
                              'noise_data3', noise_data3, ...
                              'noise_data4', noise_data4, ...
                              'noise_data5', noise_data5, ...
                              'noise_data6', noise_data6, ...
                              'noise_data7', noise_data7 );

    error_all = struct( 'error1', error1, ...
                        'error2', error2, ...
                        'error3', error3, ...
                        'error4', error4, ...
                        'error5', error5, ...
                        'error6', error6, ...
                        'error7', error7 );
end
