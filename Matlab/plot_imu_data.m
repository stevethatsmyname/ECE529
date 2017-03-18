fs = 100;  %100hz sample rate
dt = 1/fs;  


imu_raw_data{1} = csvread('raw_imu_tapping.txt');
imu_raw_data{2} = csvread('raw_imu_stationary.txt');
imu_raw_data{3} = csvread('raw_imu_rotation_to_initial_orientation.txt');
%figure; plot(M)


%format data into accelerometer and gyroscope data.
for(i=1:3)
    imu_data{i}.gyro = imu_raw_data{i}(:,4:6);
    imu_data{i}.accel = imu_raw_data{i}(:,1:3);
end

%plot gyro and accel data

for(i=1:3)
    this_imu_data = imu_data{i};
    number_of_samples = size(this_imu_data.gyro,1);
    n = 1:number_of_samples;
    t = n*dt;
    
    figure;
    subplot(2,2,1);
    plot(t,this_imu_data.gyro);
    subplot(2,2,2);
    plot(t,this_imu_data.accel);
    
    [gyro_psd_matrix, f] = matrix_psd(this_imu_data.gyro,[],[],[],fs);
    [accel_psd_matrix, f] = matrix_psd(this_imu_data.accel, [], [], [], fs);
    subplot(2,2,3);
    loglog(f, gyro_psd_matrix);
    subplot(2,2,4);
    loglog(f, accel_psd_matrix);
    

    
    
    
    
    

end
