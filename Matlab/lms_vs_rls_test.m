clear;
close all;


mu = 0.002;  %convergence factor  - LMS
lambda = 0.99; %forgetting factor - RLS
num_sample = 3000;  %initial number of samples
N= 50;   %filter size
dt = 0.01;  %sample period



desired_biases = 1:10;

for(j=1:length(desired_biases))
    bias_desired = desired_biases(j);
    
    clear all_states;
    clear init_state;
    clear all_states_RLS
    clear init_state_rls;
    
    
    all_n = (0:num_sample-1)';
    t = all_n*dt;
    
    bias_input = 0;
    gain_input = 1;
    gain_desired = 5;
    dc_d = 0;  % DC content in desired signal
    
    
    w_d = pi/80; %desired frequency
    d_n = sin(w_d * all_n)+dc_d + bias_desired; % sinusoid
    phase_lag = 0*pi/180;
    d_n_lag = sin(w_d * all_n -phase_lag); % sinusoid with phase lag
    
    
    
    w_noise = pi/4;  % noise frequency
    phase_noise = 0.3421;
    gain_noise = 1;
    gain_d = 1;
    gain_random_noise = 0.02;
    % create an x(n) with an undesired sine wave and some noise.
    u_n = dc_d + gain_d * d_n_lag + gain_noise*sin( w_noise * all_n + phase_noise) + gain_random_noise*randn(size(all_n));
    u_n = gain_input*u_n + bias_input; % add a scaling error and bias to input.
    
    new_meas.x = 0;
    new_meas.d = 0;
    %function [ new_state ] = update_lms( state, new_meas, N, mu)
    init_state = update_lms([], new_meas, N, mu);
    
    
    init_state_rls = update_rls([], new_meas, N, lambda);
    all_states_RLS(num_sample) = init_state_rls;
    current_state_RLS = init_state_rls;
    
    
    current_state = init_state;
    
    all_states(num_sample) = init_state;
    
    
    y = zeros(size(d_n));
    y_rls = y;
    for(n=1:num_sample)
        
        
        new_meas.x = u_n(n);
        new_meas.d = d_n(n);
        %LMS
        new_state = update_lms(current_state, new_meas, N, mu);
        all_states(n) = new_state;
        y(n) = new_state.y;
        current_state = new_state;
        
        %RLS
        new_state_RLS = update_rls(current_state_RLS, new_meas, N, lambda);
        all_states_RLS(n) = new_state_RLS;
        y_rls(n) = new_state_RLS.y;
        current_state_RLS = new_state_RLS;
        
        
    end
    
    figure;
    
    subplot(2,1,1);
    plot([u_n, d_n, y, y_rls]);
    legend('input','desired','output', 'output (RLS)');
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
    
    hold on;
    
    rls_err = [all_states_RLS.alpha];
    rls_err_sqr = rls_err.^2;
    plot(rls_err_sqr);
    
    %initialize states to zero (w, and P)
    
    %initialize the state
    % init_state.P     = delta*eye(M+1);  %P     : covariance matrix
    % init_state.x_bar = zeros(M+1,1);    %x_bar : (M+1)by(1) measurement matrix
    % init_state.w     = zeros(M+1,1);    %w     : filter coefficients matrix
    coeffs = current_state.w;
    
    
    fprintf(1,' ********  LMS Filter  ********\n');
    fprintf(1,'DC Gain: %.3f \n',sum(coeffs(1:(end-1))));
    fprintf(1,'Constant coeff : %.3f \n', coeffs(end));
    
    rls_coeffs = current_state_RLS.w;
    dc_gain = sum(rls_coeffs(1:end-1));
    
    fprintf(1,' ********  RLS Filter  *********\n');
    fprintf(1,'DC Gain: %.3f \n',dc_gain);
    fprintf(1,'Constant coeff : %.3f \n', rls_coeffs(end));
    
end