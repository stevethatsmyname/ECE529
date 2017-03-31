function [ new_orientation ] = update_orientation_complementary( prev_attitude, gyro_xyz, accel_xyz, gain)
% new_orientation = update_orientation_complementary( previous_orientation, gyro_xyz, accel_xyz, gain )
%Outputs a 3x3 DCM with the new orientation.  
%Inputs:
% 3x3 DCM with the prev orientation
% gyro xyz:  gyro output (Del theta) in radians per sample
% accel xyz: accel output in G
% gain: the gain of the accelerometer complementary piece (typical value would be 0.01 to 0.04 ish). 

accel_weight = gain;
gyro_weight = 1-gain; %they have to add to 1. 

fake_dt = 1; %a dt is needed for the acceleration velocity calculations to make sense.  

ax = accel_xyz(1);
ay = accel_xyz(2);
az = accel_xyz(3);

gx = gyro_xyz(1);
gy = gyro_xyz(2);
gz = gyro_xyz(3);

K_b_prev = prev_attitude(3,:); % save off the previous orientation.
I_b_prev = prev_attitude(1,:);
%  The accelerometer output forms an estimate for the 3rd row of the DCM
%  Because the "Inertial" or "World" z-axis aligns with the gravity vector
K_b = [ax, ay, az]; 
K_b = K_b/norm(K_b); %renormalize it to be a unit vector
   

% The gyro output allows us to estimate the new 1st row based on the
% previous 1st row
del_theta_gyro = -1*gyro_xyz; 
I_b =  prev_attitude(1,:) +cross(del_theta_gyro, prev_attitude(1,:)); 
I_b = I_b / norm(I_b); %renormalize it to be a unit vector;

%apparent linear velocity of the "K" unit vector
v_K = (K_b - K_b_prev) / fake_dt; 
% which implies an angular velocity perpendicular to it.
w_K = cross(K_b_prev, v_K)';

% apparent linear velocity of the "I" unit vector
% which implies an angular velocity perpendicular to it.
v_I = (I_b - I_b_prev)  /fake_dt;
w_I = cross(I_b, v_I)';

% we get a total "accelerometer" angular velocity estimate by adding the two together.
wa =  w_K + w_I;

wg = gyro_xyz / fake_dt; 
del_theta_compl = (accel_weight * wa  + gyro_weight * wg) * fake_dt;

%update dcm
dcm_update_I =  prev_attitude(1,:) +cross(del_theta_compl, prev_attitude(1,:));
dcm_update_I = dcm_update_I / norm(dcm_update_I); %renormalize
        
dcm_update_J = prev_attitude(2,:) +cross(del_theta_compl, prev_attitude(2,:));
dcm_update_J = dcm_update_J / norm(dcm_update_J); %renormalize

%Fix I and J to be perpendicular
err_IJ = dot(dcm_update_I, dcm_update_J)/2;
dcm_update_I = dcm_update_I - err_IJ * dcm_update_J;
dcm_update_J = dcm_update_J - err_IJ * dcm_update_I;
dcm_update_K = cross(dcm_update_I , dcm_update_J);

new_orientation = [...
    dcm_update_I;
    dcm_update_J;
    dcm_update_K];




end

