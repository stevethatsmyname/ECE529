clear all;
close all;
accel_scale_factor = 1.03637;  %it would be nice to have the filter solve for these.  But for now we can hard code them.  
gyro_scale_factor = 1.0000;
R2D = 180/pi;  %radians to degrees conversion factor (gyroscope output is in degrees/sec);
 
fs = 100;  %100hz sample rate
dt = 1/fs;  
fc = 5; %cutoff frequency (in continues time
fc_d = fc/fs; %cutoff frequency (digital).

imu_raw_data{1} = csvread('raw_imu_rotation_to_initial_orientation.txt');
%imu_raw_data{1} = csvread('raw_imu_tapping.txt');
%imu_raw_data{2} = csvread('raw_imu_stationary.txt');
%imu_raw_data{3} = csvread('raw_imu_rotation_to_initial_orientation.txt');
%figure; plot(M)


%format data into accelerometer and gyroscope data.
for(i=1:length(imu_raw_data))
    imu_data{i}.gyro = imu_raw_data{i}(:,4:6);
    imu_data{i}.accel = imu_raw_data{i}(:,1:3)*accel_scale_factor;
end

%plot gyro and accel data

for(i=1:length(imu_raw_data))
    this_imu_data = imu_data{i};
    number_of_samples = size(this_imu_data.gyro,1);
    n = 1:number_of_samples;
    t = n*dt;
    
    figure;
    subplot(2,2,1);
    plot(t,this_imu_data.gyro);
    title('Raw IMU Data');
    subplot(2,2,2);
    plot(t,this_imu_data.accel);
    
    [gyro_psd_matrix, f] = matrix_psd(this_imu_data.gyro,[],[],[],fs);
    [accel_psd_matrix, f] = matrix_psd(this_imu_data.accel, [], [], [], fs);
    subplot(2,2,3);
    loglog(f, gyro_psd_matrix);
    subplot(2,2,4);
    loglog(f, accel_psd_matrix);
    

    % IIR lowpass filter
    gyro_filter_b = [3.12389769170918e-05      0.000124955907668367      0.000187433861502551,  0.000124955907668367      3.12389769170918e-05]
    gyro_filter_a = [1, -3.5897339, 4.85127588, -2.92405265, 0.663010484];
    
    accel_filter_a = gyro_filter_a;
    accel_filter_b = gyro_filter_b;

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
        
        gx = imu_data{i}.gyro(j,1)*dt/R2D; %input IMU data, convert from deg/s to radians/sample
        gy = imu_data{i}.gyro(j,2)*dt/R2D;
        gz = imu_data{i}.gyro(j,3)*dt/R2D;
        
        imu_dcm_x = [...
            1,        0,        0;
            0,      cos(gx),    -sin(gx);
            0,      sin(gx),    cos(gx)];
        
        imu_dcm_z = [...
            cos(gz),  -sin(gz), 0;
            sin(gz), cos(gz), 0;
            0,        0,       1];  %the rotation for this frame based on IMU input
        
        imu_dcm_y = [...
            cos(gy),  0,     sin(gy);
            0,        1,       0;
            -sin(gy), 0,     cos(gy)];
        
        %temp attitude should be "DCM_BI" or "body from inertial" coordinate
        %frame
        temp_attitude = temp_attitude * imu_dcm_x * imu_dcm_y * imu_dcm_z;
        
        % dcm(a,b) : a: missile axis, b: inertial axis.
        
        current_attitude_dcm(j,:,:) = temp_attitude;
        current_attitude_euler(j,1) = [atan2(temp_attitude(2,3),temp_attitude(3,3))]; %roll
        current_attitude_euler(j,2) = [asin(temp_attitude(1,3))];                     %pitch 
        current_attitude_euler(j,3) = [atan2(temp_attitude(1,2), temp_attitude(1,1))];%yaw
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

end
