clear;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% CONSTANTS, COEFFICIENTS, ETC %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SCRIPT CONTROL PARAMETERS
user_selected_file = true;


%IMU Output parameters
R2D = 180/pi;  %radians to degrees conversion factor (gyroscope output is in degrees/sec);
fs = 100;  %IMU sample rate, configurable
dt = 1/fs;
% for fixed point conversions
int16_max = 32767; % IMU output is int 16
scaling_max_gyro = 500; %gyro range (deg/s), configurable
scaling_max_accel = 4;  %accel range (G's), configurable

%FILTERING PARAMETERS

%complementary filter
accelerometer_weight = 0.1;  %for complementary filter

%RLS
lambda = 0.99; %forgetting factor on RLS filter
delta = 100; %Parameter used for filter initialization
solve_dc = false;

%LMS
mu = 0.01;  %convergence factor  - LMS

%Both RLS AND LMS
M = 50;  %number of samples in RLS/LMS FIR filters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  GET IMU DATA FROM FILE  %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(user_selected_file == true)
    [file_nm path_nm] = uigetfile({'*.txt;*.log','.txt, .log'},'Select IMU output txt file');
    
    if(~iscell(file_nm))
        file_nm = {file_nm};
    end
    
    for(i=1:length(file_nm))
        full_file_name{i} = fullfile(path_nm, file_nm{i});
    end
    
else
    full_file_name = {'imu_data.log'};
end

for(i=1:length(full_file_name))
   imu_raw_data{i} = csvread(full_file_name{i}); 
end


% imu data file format
time_index = 1;
accel_index = 2:4;
gyro_index = 5:7;
% convert imu data to data structure
% allow for multiple files to be selected (will batch process and plot);
for(i=1:length(imu_raw_data))
    imu_data{i}.time = imu_raw_data{i}(:,time_index)*10e-5;
    imu_data{i}.gyro = imu_raw_data{i}(:,gyro_index)*scaling_max_gyro/int16_max;
    imu_data{i}.accel = imu_raw_data{i}(:,accel_index)*scaling_max_accel/int16_max;
end


for(i=1:length(imu_data))
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %pull the needed info out of the imu data
    gyro_data = imu_data{i}.gyro;
    accel_data = imu_data{i}.accel;
    number_of_samples = size(gyro_data,1);
    
    %time array
    t = (1:number_of_samples) * dt;
    
    %initialize attitude - identity matrix
    initial_attitude = eye(3);
    %initialize attitude for all 3 estimators running in parallel
    current_attitude_compl    = initial_attitude;
    current_attitude_LMS      = initial_attitude;
    current_attitude_RLS      = initial_attitude;
    
    %set weights for complementary filter
    accel_weight = accelerometer_weight;
    gyro_weight = 1-accel_weight;
    
    % initialize empty RLS and LMS filter states
    %RLS
    RLS_state_X = update_RLS([], [], M, lambda);
    RLS_state_Y = update_RLS([], [], M, lambda);
    RLS_state_Z = update_RLS([], [], M, lambda);
    %LMS
    LMS_state_X = update_LMS([], [], M, mu, 'init_dc');
    LMS_state_Y = update_LMS([], [], M, mu, 'init_dc');
    LMS_state_Z = update_LMS([], [], M, mu, 'init_dc');
    
    if(solve_dc)
       RLS_options = 'find_bias'; 
    else
        RLS_options = [];
    end
    %preallocate arrays for speed

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%% ORIENTATION ESTIMATOR %%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for(j=1:number_of_samples)
        
        %gyro/accel measurement for this sample
        % gyro needs to be in radians (per sample) and accel is in "G"
        % Both need to be a column vector.
        gyro_meas = -1 * gyro_data(j,:)' *dt/R2D;
        accel_meas = accel_data(j,:)';
        
        
        %complementary filter
        new_attitude_compl = update_orientation_complementary(...
            current_attitude_compl, gyro_meas, accel_meas, accel_weight);
        
        %estimate 'desired' accelerometer measurements (based on new attitude)
        accel_est = new_attitude_compl' * [0;0;1];
        
        % filter input
        new_meas_x.x = accel_meas(1);
        new_meas_y.x = accel_meas(2);
        new_meas_z.x = accel_meas(3);
        % filter "Desired" output
        new_meas_x.d = accel_est(1);
        new_meas_y.d = accel_est(2);
        new_meas_z.d = accel_est(3);
        
        %Update the RLS state
        
        RLS_state_X = update_RLS(RLS_state_X, new_meas_x, M, lambda, RLS_options);
        RLS_state_Y = update_RLS(RLS_state_Y, new_meas_y, M, lambda, RLS_options);
        RLS_state_Z = update_RLS(RLS_state_Z, new_meas_z, M, lambda, RLS_options);
        RLS_out_accel = [RLS_state_X.y; RLS_state_Y.y; RLS_state_Z.y];
        %update the LMS state
        LMS_state_X = update_LMS(LMS_state_X, new_meas_x, M, mu);
        LMS_state_Y = update_LMS(LMS_state_Y, new_meas_y, M, mu);
        LMS_state_Z = update_LMS(LMS_state_Z, new_meas_z, M, mu);
        LMS_out_accel = [LMS_state_X.y; LMS_state_Y.y; LMS_state_Z.y];
        
        
        current_attitude_LMS = update_orientation_complementary(...
            current_attitude_LMS, gyro_meas, LMS_out_accel, accel_weight);
        
        current_attitude_RLS = update_orientation_complementary(...
            current_attitude_RLS, gyro_meas, RLS_out_accel, accel_weight);
        
        % Finally setup for next sample 
        current_attitude_compl = new_attitude_compl;
        
        
        %save off arrays for plotting later
        
        %compute euler angles from the DCM's
        
        accel_meas_array(j,:) = accel_meas';
        accel_est_array(j,:) = accel_est';
        RLS_output_accel_array(j,:) = RLS_out_accel';
        LMS_output_accel_array(j,:) = LMS_out_accel';
        
        current_attitude_DCM_array(j,:,:)= current_attitude_compl;
        current_attitude_LMS_array(j,:,:)= current_attitude_LMS;
        current_attitude_RLS_array(j,:,:)= current_attitude_RLS;
        
        
    end
    
    
    
    figure;
    axis_strings = {'x','y','z'};
    for(j=1:3);
        subplot(3,1,j);
        plot([accel_meas_array(:,j),...
            accel_est_array(:,j)],...
            'linewidth',2);
        
        hold on;
        plot([LMS_output_accel_array(:,j),...
            RLS_output_accel_array(:,j)], '--','linewidth',2);
        if(j==1)
            legend('measured accel','estimate (from attitude)','LMS output','RLS output');
        end
        if(j==3)
            xlabel('Sample');
        end
        
        ylabel([axis_strings{j} ' axis accel (g)']);
        
        axes_handles(j) = gca;
    end
    
    linkaxes(axes_handles, 'x');
    
    
    if(solve_dc)
        dc_gain_x = sum(RLS_state_X.w(1:end-1));
        dc_gain_y = sum(RLS_state_Y.w(1:end-1));
        dc_gain_z = sum(RLS_state_Z.w(1:end-1));
        
        accel_bias_x = RLS_state_X.w(end);
        accel_bias_y = RLS_state_Y.w(end);
        accel_bias_z = RLS_state_Z.w(end);
        
        
    else
        dc_gain_x = sum(RLS_state_X.w);
        dc_gain_y = sum(RLS_state_Y.w);
        dc_gain_z = sum(RLS_state_Z.w);
        
        accel_bias_x = 0;
        accel_bias_y = 0;
        accel_bias_z = 0;
    end
    fprintf(1,'*****RLS output********\n');
    fprintf(1,'X axis.  DC Gain: %.3f, Accel Bias term: %.3f \n', dc_gain_x, accel_bias_x);
    fprintf(1,'Y axis.  DC Gain: %.3f, Accel Bias term: %.3f \n', dc_gain_y, accel_bias_y);
    fprintf(1,'Z axis.  DC Gain: %.3f, Accel Bias term: %.3f \n', dc_gain_z, accel_bias_z);
    fprintf(1,'*****LMS output *******\n');
    
    dc_gain_x = sum(LMS_state_X.w);
    dc_gain_y = sum(LMS_state_Y.w);
    dc_gain_z = sum(LMS_state_Z.w);
    
    fprintf(1,'X axis.  DC Gain: %.3f \n', dc_gain_x);
    fprintf(1,'Y axis.  DC Gain: %.3f\n', dc_gain_y);
    fprintf(1,'Z axis.  DC Gain: %.3f\n', dc_gain_z);    
    
    
    if(solve_dc)
        [f_x, w] = freqz(RLS_state_X.w(1:end-1));
        [f_y, w] = freqz(RLS_state_Y.w(1:end-1));
        [f_z, w] = freqz(RLS_state_Z.w(1:end-1));
    else
        [f_x, w] = freqz(RLS_state_X.w);
        [f_y, w] = freqz(RLS_state_Y.w);
        [f_z, w] = freqz(RLS_state_Z.w);
    end
        [f_x_LMS, w] = freqz(LMS_state_X.w);
        [f_y_LMS, w] = freqz(LMS_state_Y.w);
        [f_z_LMS, w] = freqz(LMS_state_Z.w);
    
    figure;
    % look at the final frequency response of the LMS filter
    
%     subplot(3,1,1);
%     semilogy(w/pi,abs(f_x));
%     title('Final RLS filter frequency response (x)');
%     subplot(3,1,2);
%     semilogy(w/pi,abs(f_y));
%     title('Final RLS filter frequency response (y)');
%     subplot(3,1,3);
%     semilogy(w/pi,abs(f_z));
%     title('Final RLS filter frequency response (z)');
%     xlabel('Frequency (radians/pi)');
    
    
    figure; %try plotting all 3 on one axis
    subplot(2,1,1);
    freq_LMS = abs([f_x_LMS, f_y_LMS, f_z_LMS]);
    semilogy(w/pi,freq_LMS,'linewidth',2);
    legend('x','y','z');
    title('LMS Filter final frequency response');
    ax(2) = gca;
    subplot(2,1,2);
    freq_RLS = abs([f_x, f_y, f_z]);
    semilogy(w/pi,freq_RLS,'linewidth',2)
    ax(1) = gca;
    title('RLS Filter final frequency response','linewidth',2);
    
 
    
    % plot euler angles on attitude estimate.  
    
    
    euler_roll_LMS = atan2(current_attitude_LMS_array(:,2,3), current_attitude_LMS_array(:,3,3));
    euler_pitch_LMS  = -asin(current_attitude_LMS_array(:,1,3));
    euler_yaw_LMS= atan2(current_attitude_LMS_array(:,1,2), current_attitude_LMS_array(:,1,1));

    
    figure; plot(t,[euler_roll_LMS, euler_pitch_LMS, euler_yaw_LMS],'linewidth',2);
    title('LMS attitude estimate');
    
    
    euler_roll_RLS = atan2(current_attitude_RLS_array(:,2,3), current_attitude_RLS_array(:,3,3));
    euler_yaw_RLS= atan2(current_attitude_RLS_array(:,1,2), current_attitude_RLS_array(:,1,1));
    euler_pitch_RLS  = -asin(current_attitude_RLS_array(:,1,3));
    
    figure; plot(t,[euler_roll_RLS, euler_pitch_RLS, euler_yaw_RLS],'linewidth',2);
    title('RLS attitude estimate');
    legend('roll','pitch','yaw');
    
    euler_roll_Compl = atan2(current_attitude_DCM_array(:,2,3), current_attitude_DCM_array(:,3,3));
    euler_yaw_Compl= atan2(current_attitude_DCM_array(:,1,2), current_attitude_DCM_array(:,1,1));
    euler_pitch_Compl  = -asin(current_attitude_DCM_array(:,1,3));
    
    figure; plot(t,[euler_roll_Compl, euler_pitch_Compl, euler_yaw_Compl],'linewidth',2);
    title('Complementary filter attitude estimate');    
    
    

    
    
end