% Complementary filter


    
    %the full algorithm
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
    
    for(j=50:number_of_samples)
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
        
        %Now, using accelerometer output, come up with an angular rate
        %estimate
        ax = accel_data(j,1);
        ay = accel_data(j,2);
        az = accel_data(j,3);
        
        %allow the use of a filter on accelerometer in this framework
        %(without having to change a bunch of code later on)
        ax_filt = ax;
        ay_filt = ay;
        az_filt = az;
        
        K_b = [ax_filt, ay_filt, az_filt]/norm([ax_filt; ay_filt; az_filt]);
        % use the "Ib" from previous frame.  Then solve for Jb as the cross
        % product of the two vectors
        del_theta_gyro =  wg * dt;
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
    
    
    