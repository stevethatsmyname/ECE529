clear all;
close all;
accel_scale_factor = 1.03637;  %it would be nice to have the filter solve for these.  But for now we can hard code them.
gyro_scale_factor = 1.0000;
R2D = 180/pi;  %radians to degrees conversion factor (gyroscope output is in degrees/sec);

fs = 100;  %100hz sample rate
dt = 1/fs;
fc = 5; %cutoff frequency (in continues time
fc_d = fc/fs; %cutoff frequency (digital).
int16_max = 32767; % imu conversion ratio
scaling_max_gyro = 500; %gyro range (deg/s);
scaling_max_accel = 4;  %accel range (G's);

use_smoothed_gyro_data = true; %for gyro only calculations use filtered input
check_accel_validity = false;  % check to see if accelerometer orientation is valid before using to update filter.
accelerometer_weight = 0.03;  %for complementary filter



use_filter = 4;
%  1 : 5Hz 2nd order butterworth LPF
%  2 : 5Hz 4th order butterworth LPF
%  3 : 2Hz 4th order butterworth LPF
%  4 : none (use adaptive filter in recursive algorithm);

imu_raw_data{1} = csvread('imu_data.log');
%imu_raw_data{1} = csvread('raw_imu_rotation_to_initial_orientation.txt');
%imu_raw_data{1} = csvread('raw_imu_tapping.txt');
%imu_raw_data{2} = csvread('raw_imu_stationary.txt');
%imu_raw_data{3} = csvread('raw_imu_rotation_to_initial_orientation.txt');
%figure; plot(M)

time_index = 1;
accel_index = 2:4;
gyro_index = 5:7;
%format data into accelerometer and gyroscope data.
for(i=1:length(imu_raw_data))
    imu_data{i}.time = imu_raw_data{i}(:,time_index)*10e-5;
    imu_data{i}.gyro = imu_raw_data{i}(:,gyro_index)*scaling_max_gyro/int16_max;
    imu_data{i}.accel = imu_raw_data{i}(:,accel_index)*scaling_max_accel/int16_max;
end

%plot gyro and accel data

for(i=1:length(imu_raw_data)) %for each file
    this_imu_data = imu_data{i};
    number_of_samples = size(this_imu_data.gyro,1);
    n = 1:number_of_samples;
    t = n*dt;
    
    figure;
    subplot(2,2,1);
    plot(t,this_imu_data.gyro);
    
    title('Raw Gyro Data');
    legend('x','y','z');
    subplot(2,2,2);
    plot(t,this_imu_data.accel);
    title('Raw Accel Data');
    [gyro_psd_matrix, f] = matrix_psd(this_imu_data.gyro,[],[],[],fs);
    [accel_psd_matrix, f] = matrix_psd(this_imu_data.accel, [], [], [], fs);
    subplot(2,2,3);
    loglog(f, gyro_psd_matrix);
    subplot(2,2,4);
    loglog(f, accel_psd_matrix);
    
    
    
    
    switch(use_filter)
        case 1
            % 5Hz 2nd order butterworth
            gyro_filter_a = [1         -1.77863177782459         0.800802646665708];
            gyro_filter_b = [0.00554271721028068        0.0110854344205614       0.00554271721028068];
            
        case 2
            
            % IIR lowpass filter - 5Hz cutoff 4th order butterworth
            gyro_filter_b = [3.12389769170918e-05      0.000124955907668367      0.000187433861502551,  0.000124955907668367      3.12389769170918e-05];
            gyro_filter_a = [1, -3.5897339, 4.85127588, -2.92405265, 0.663010484];
        case 3
            %2Hz 4th order butterworth
            
            gyro_filter_b =  1.0e-05 *[ 0.089848614637233   0.359394458548934   0.539091687823401   0.359394458548934   0.089848614637233];
            gyro_filter_a =  [1.000000000000000  -3.835825540647347   5.520819136622225  -3.533535219463012   0.848555999266476 ];
            
            
        case 4
            gyro_filter_b = [1];
            gyro_filter_a = [1];
            
    end
    
    
    
    
    accel_filter_a = gyro_filter_a;
    accel_filter_b = gyro_filter_b;
    
    
    
    % IIR lowpass filter - 5Hz 3rd order butterworth.
    
    
    gyro_smoothed = zeros(size(this_imu_data.gyro));
    accel_smoothed = zeros(size(this_imu_data.accel));
    for(j=1:3)
        gyro_smoothed(:,j) = filter(gyro_filter_b, gyro_filter_a, this_imu_data.gyro(:,j));
        accel_smoothed(:,j) = filter(accel_filter_b, accel_filter_a, this_imu_data.accel(:,j));
    end
    
    % find the magnitude of the accel vector.
    accel_mag = (this_imu_data.accel(:,1).^2 + this_imu_data.accel(:,2).^2 +this_imu_data.accel(:,3).^2).^0.5;
    accel_mag_smooth = (accel_smoothed(:,1).^2 + accel_smoothed(:,2).^2 + accel_smoothed(:,3).^2).^0.5;
    
    accel_mag_rate = diff(accel_mag_smooth)/dt;
    
    
    figure; plot([accel_mag accel_mag_smooth]); title('Accel magnitude (smoothed vs not)');
    
    roll_measurement_valid = false(number_of_samples,1);
    pitch_measurement_valid = false(number_of_samples,1);
    
    roll_measurement = nan(number_of_samples,1);
    pitch_measurement = nan(number_of_samples,1);
    
    roll_state = nan(size(accel_mag_smooth));
    
    at_rest_threshold1 = 0.2; %acceleration rate (lol i know right?) less than this.
    % g's per second
    at_rest_threshold2 = 0.05; %accel has to be this close to "1g" to be considered at rest.
    
    
    
    for(j=1:(length(accel_mag_smooth)-1));
        if( (accel_mag_rate(j) <= at_rest_threshold1) && abs(accel_mag_smooth(j)-1) < at_rest_threshold2) %are we at rest?
            roll_measurement_valid(j) = true;
            pitch_measurement_valid(j) = true;
        else
            roll_measurement_valid(j) = false;
            pitch_measurement_valid(j) = false;
        end
        %this is wrong - recheck math when my brain works
        ax = accel_smoothed(j,1);
        ay = accel_smoothed(j,2);
        az = accel_smoothed(j,3);
        roll_measurement(j) = atan2(ay,az);
        
        % align the z axis of aircraft to z axis of
        ayz = (ay.^2 + az.^2)*0.5;
        pitch_measurement(j) = -asin(ax);
        
    end
    
    %attitude measurement using gyroscope output
    initial_attitude = eye(3);
    current_attitude_dcm = zeros(number_of_samples,3,3);
    current_attitude_euler = zeros(number_of_samples,3);  %we will be computing attitude as a DCM then converting to euler angles (which is easier to visualize)
    temp_attitude = initial_attitude;
    for(j=1:number_of_samples)
        
        if(use_smoothed_gyro_data)
            gx = gyro_smoothed(j,1)*dt/R2D;
            gy = gyro_smoothed(j,2)*dt/R2D;
            gz = gyro_smoothed(j,3)*dt/R2D;
            
        else
            
            gx = imu_data{i}.gyro(j,1)*dt/R2D; %input IMU data, convert from deg/s to radians/sample
            gy = imu_data{i}.gyro(j,2)*dt/R2D;
            gz = imu_data{i}.gyro(j,3)*dt/R2D;
        end
        
        imu_dcm_x = [...
            1,        0,        0;
            0,      cos(gx),    -sin(gx);
            0,      sin(gx),    cos(gx)]';
        
        imu_dcm_z = [...
            cos(gz),  -sin(gz), 0;
            sin(gz), cos(gz), 0;
            0,        0,       1]';  %the rotation for this frame based on IMU input
        
        imu_dcm_y = [...
            cos(gy),  0,     sin(gy);
            0,        1,       0;
            -sin(gy), 0,     cos(gy)]';
        
        %temp attitude should be "DCM_BI" or "body from inertial" coordinate
        %frame
        temp_attitude = temp_attitude * imu_dcm_x * imu_dcm_y * imu_dcm_z;
        
        % dcm(a,b) : a: missile axis, b: inertial axis.
        
        current_attitude_dcm(j,:,:) = temp_attitude;
        current_attitude_euler(j,1) = [atan2(temp_attitude(2,3),temp_attitude(3,3))]; %roll
        current_attitude_euler(j,2) = -[asin(temp_attitude(1,3))];                     %pitch
        current_attitude_euler(j,3) = [atan2(temp_attitude(1,2), temp_attitude(1,1))];%yaw
        
        % tilt vector update equation.
        
        
    end
    
    
    figure;
    subplot(2,1,1);
    plot([roll_measurement pitch_measurement]); title('Attitude measurements from accelerometer');
    ylabel('attitude(radians)'); xlabel('time (s');
    legend('roll','pitch');
    
    subplot(2,1,2);
    plot(current_attitude_euler);
    title('Attitude measurements from gyro');
    xlabel('time'); ylabel('Attitude (radians)');
    legend('roll','pitch','yaw');
    
    
    
    
    synthetic_gravity_body = zeros(3,number_of_samples);
    
    for(j=1:number_of_samples)
        % generate a 'synthetic' gravity vector.
        g0 = [0; 0; 1];
        synthetic_gravity_body(:,j) = squeeze(current_attitude_dcm(j,:,:))*g0;
        
        
    end
    
    figure; plot(synthetic_gravity_body');
    hold on; plot(this_imu_data.accel);
    legend('Gx est(orientation)','Gy est (orientation)','Gz est (orientation)','Ax','Ay','Az');
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %the full algorithm
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    gyro_data = gyro_smoothed;
    accel_data = accel_smoothed;
    
    initial_attitude = eye(3);
    temp_attitude    = initial_attitude;
    % Define I(x) pointing North, J(y) Pointing West, and K(z) pointing Up.
    %
    accel_weight_if_valid = accelerometer_weight;
    gyro_weight_if_valid = 1-accel_weight_if_valid; % hard code these for now, but they should be dynamically set.
    
    wa_ = zeros(3,number_of_samples);
    wg_ = zeros(3,number_of_samples);
    
    %initialize RLS filter
    M = 50;  %number of samples in RLS filter
    delta = 100; %don't remember what delta was
    lambda = 0.98;
    previous_state_x = new_rls(M, delta);
    previous_state_y = new_rls(M, delta);
    previous_state_z = new_rls(M, delta);
    
    
    %get the first update
    new_meas.x = 1; %test input
    new_meas.d = 0.5; %test input
    
    rls_states_x(number_of_samples) = update_rls(previous_state_x, new_meas, M, lambda ); %put something in the last element so that matlab will preallocate the state matrix.
    rls_states_y = rls_states_x;
    rls_states_z = rls_states_y;
    
    for(j=1:number_of_samples)

        prev_attitude = temp_attitude;
        
        if(roll_measurement_valid(j) && check_accel_validity == true)
            accel_weight = accel_weight_if_valid;
            gyro_weight = gyro_weight_if_valid;
        else
            accel_weight = 0;
            gyro_weight = 1;
            accel_weight = accel_weight_if_valid;
            gyro_weight = gyro_weight_if_valid;
        end
        
        gx = gyro_data(j,1);
        gy = gyro_data(j,2);
        gz = gyro_data(j,3);
        %angular velocity as measured by the gyroscope channel.
        
        %adopt the framework of using some type of filter on the Gyro
        %channels, prior to the angular rate estimate (w)
        gx_filt = gx;
        gy_filt = gy;
        gz_filt = gz;
        
        wg = -[gx_filt; gy_filt; gz_filt]/R2D; % gyro sign is flipped
        del_theta_gyro =  wg * dt;
        I_g =  prev_attitude(1,:) +cross(del_theta_gyro, prev_attitude(1,:));
        I_g = I_g / norm(I_g);
        J_g =  prev_attitude(2,:) +cross(del_theta_gyro, prev_attitude(2,:));
        J_g = J_g / norm(J_g);
        
        err_IJ = dot(I_g, J_g)/2;
        
        I_g = I_g - err_IJ * J_g;
        J_g = J_g - err_IJ * I_g;
        K_g = cross(I_g , J_g);
        
        dcm_gyro = [I_g; J_g; K_g];
        
        accel_est = dcm_gyro' * [0;0;1];
        
        
        
        
        %Now, using accelerometer output, come up with an angular rate
        %estimate
        ax = accel_data(j,1);
        ay = accel_data(j,2);
        az = accel_data(j,3);
        
        new_meas_x.x = ax;
        new_meas_x.d = accel_est(1);
        new_meas_y.x = ay;
        new_meas_y.d = accel_est(2);
        new_meas_z.x = az;
        new_meas_z.d = accel_est(3);
        
        new_state_x = update_rls( previous_state_x, new_meas_x ,M, lambda);
        new_state_y = update_rls( previous_state_y, new_meas_y ,M, lambda);
        new_state_z = update_rls( previous_state_z, new_meas_z ,M, lambda);
              
        rls_states_x(j) = new_state_x;
        rls_states_y(j) = new_state_y;
        rls_states_z(j) = new_state_z;
        
        
        previous_state_x = new_state_x;
        previous_state_y = new_state_y;
        previous_state_z = new_state_z;
        
        
        ax_filt = ax;
        ay_filt = ay; 
        az_filt = az;

%         ax_filt = new_state_x.y;
%         ay_filt = new_state_y.y;
%         az_filt = new_state_z.y;
        
        K_b = [ax_filt, ay_filt, az_filt]/norm([ax_filt; ay_filt; az_filt]);
        % use the "Ib" from previous frame.  Then solve for Jb as the cross
        % product of the two vectors
        
        I_g =  prev_attitude(1,:) +cross(del_theta_gyro, prev_attitude(1,:));
        I_g = I_g/norm(I_g);
        
        I_b  = I_g;
        I_b_prev = prev_attitude(1,:);
        J_b = cross(K_b,I_b);
        K_b_prev = prev_attitude(3,:);
        
        v_a = (K_b - K_b_prev)/dt;   %linear velocity of the K unit vector in body frame
        wa = cross(K_b, v_a)';       %angular velocity based on change in K unit vector
        
        v_gi = (I_b - I_b_prev)/dt;
        wgi = cross(I_b, v_gi)';
        wa = wa + wgi;
        
        % compute the total del theta.
        del_theta_tot = (accel_weight * wa  + gyro_weight * wg)*dt;
        
        %update dcm .
        dcm_update_I =  prev_attitude(1,:) +cross(del_theta_tot, prev_attitude(1,:));
        dcm_update_I = dcm_update_I / norm(dcm_update_I); %normalize
        
        dcm_update_J = prev_attitude(2,:) +cross(del_theta_tot, prev_attitude(2,:));
        dcm_update_J = dcm_update_J / norm(dcm_update_J);
        
        %fix I and J to be perpendicular
        err_IJ = dot(dcm_update_I, dcm_update_J)/2;
        
        dcm_update_I = dcm_update_I - err_IJ * dcm_update_J;
        dcm_update_J = dcm_update_J - err_IJ * dcm_update_I;
        dcm_update_K = cross(dcm_update_I , dcm_update_J);
        
        dcm_update = [...
            dcm_update_I;
            dcm_update_J;
            dcm_update_K];
        
        temp_attitude = dcm_update;
        
        %now save off some of the more important paramters for plotting
        %later
        wa_(:,j) = wa;
        wg_(:,j) = wg;
        
        current_attitude = temp_attitude'; %this is DCM_IB vs DCM_BI
        current_attitude_dcm(j,:,:) = current_attitude;
        current_attitude_euler_comp(j,1) = [atan2(current_attitude(2,3),current_attitude(3,3))]; %roll
        current_attitude_euler_comp(j,2) = -[asin(current_attitude(1,3))];                     %pitch
        current_attitude_euler_comp(j,3) = [atan2(current_attitude(1,2), current_attitude(1,1))];%yaw
        
        
        
        
        
        
    end
    
    ax_in = [rls_states_x.x_bar]';
    ax_out = [rls_states_x.y]'
    ax_d = [rls_states_x.d]';
    
    figure; 
    subplot(3,1,1);
    plot([ax_in(:,1), ax_out, ax_d]);
    legend('in','out','desired');
    subplot(3,1,2);
    subplot(3,1,3);
    
    
    wa_filter_b =  1.0e-05 *[ 0.089848614637233   0.359394458548934   0.539091687823401   0.359394458548934   0.089848614637233];
    wa_filter_a =  [1.000000000000000  -3.835825540647347   5.520819136622225  -3.533535219463012   0.848555999266476 ];
    wa_ = wa_';
    for(j=1:3)
        wa_smoothed(:,j) = filter(wa_filter_b, wa_filter_a, wa_(:,j));
    end
    
    
    figure; subplot(2,1,1);
    plot(wa_);
    legend('x','y','z');
    
    hold on;
    title('Angular rate est (accel)');
    
    ax(1) = gca;
    subplot(2,1,2);
    plot(wg_');
    title('Angular rate est (gyro)');
    ax(2) = gca;
    linkaxes(ax);
    
    
    figure;
    subplot(3,1,1);
    plot(current_attitude_euler);
    title('Gyroscope only euler angles'); grid on;
    ax(1) = gca;
    subplot(3,1,2);
    plot([roll_measurement pitch_measurement]); title('Attitude measurements from accelerometer');
    ylabel('attitude(radians)');
    legend('roll','pitch'); grid on;
    ax(2) = gca;
    subplot(3,1,3);
    plot(current_attitude_euler_comp);
    title('Complementary filter euler angles'); grid on;
    ax(3) = gca;
    
    linkaxes(ax);
    clear ax;
    current_attitude_euler_accel = [roll_measurement, pitch_measurement, nan(size(roll_measurement))];
    figure;
    title_strings = {'roll','pitch','yaw'};
    for(j=1:3)
        subplot(3,1,j);
        plot([current_attitude_euler(:,j),  current_attitude_euler_accel(:,j), current_attitude_euler_comp(:,j)]);
        title(title_strings{j});
        xlabel('sample number');
        ylabel('Euler angle (radians)');
        if(j==1)
            legend('gyro only','accelerometer only','complimentary filter');
        end
        ax(j) = gca;
    end
    linkaxes(ax,'x');
    
    
    figure;
    subplot(3,1,1)
    plot([accel_data(:,1), [rls_states_x.y]'])
    title('Accel Data, raw vs (RLS) filtered');
    legend('raw','RLS');
    xlabel('Samples');
    ylabel('X Accel output (g)');
    subplot(3,1,2);
    plot([accel_data(:,2), [rls_states_y.y]'])
    ylabel('Y Accel output (g)');
    subplot(3,1,3);
    plot([accel_data(:,3), [rls_states_z.y]'])
    ylabel('Z Accel output (g)');
    
end
