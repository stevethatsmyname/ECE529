clear; 



mu = 0.0005;  %convergence factor

num_sample = 3000;  %initial number of samples
N= 20;   %filter size
dt = 0.01;  %sample period

n = (0:num_sample-1)';
t = n*dt;  

dc_d = 5;  % DC content in desired signal
w_d = pi/20; %desired frequency
d_n = sin(w_d * n)+dc_d; % sinusoid
phase_lag = 0*pi/180;
d_n_lag = sin(w_d * n-phase_lag); % sinusoid with phase lag

w_noise = pi/4;  
phase_noise = 0.3421; 
gain_noise = 1;
gain_d = 1;
gain_random_noise = 0.02;
 % create an x(n) with an undesired sine wave and some noise. 
u_n = gain_d * d_n_lag + gain_noise*sin( w_noise * n + phase_noise) + gain_random_noise*randn(size(n));
u_n = 0.5*u_n; % add a scaling error and bias to input. 

new_meas.x = 0;
new_meas.d = 0;
%function [ new_state ] = update_lms( state, new_meas, N, mu)
init_state = update_lms([], new_meas, N, mu);
current_state = init_state;

all_states(num_sample) = init_state;
y = zeros(size(d_n));
for(n=1:num_sample)
    

    new_meas.x = u_n(n);
    new_meas.d = d_n(n);
    new_state = update_lms(current_state, new_meas, N, mu);
    
    all_states(n) = new_state;
    y(n) = new_state.y;
    
    current_state = new_state;
    
end

figure; 

subplot(2,1,1);
plot([u_n, d_n, y]);
legend('input','desired','output');
xlim([50, num_sample]);
ax(1) = gca;

subplot(2,1,2);

all_err = [all_states.err];
all_err_sqr = all_err.^2;
plot(all_err_sqr);
ax(2) = gca;
title('Sqare of error');
xlim([50, num_sample]);
linkaxes(ax,'x');

%initialize states to zero (w, and P)

%initialize the state
% init_state.P     = delta*eye(M+1);  %P     : covariance matrix
% init_state.x_bar = zeros(M+1,1);    %x_bar : (M+1)by(1) measurement matrix
% init_state.w     = zeros(M+1,1);    %w     : filter coefficients matrix
coeffs = current_state.w;

fprintf(1,'DC Gain: %.3f \n',sum(coeffs(1:(end-1))));
fprintf(1,'Constant coeff : %.3f \n', coeffs(end));