% Recursive Least Squares Estimation Tester Script

%definition of terms
%lambda = forgetting factor (0 to 1)
% M : filter size [Wikipedia: p]
% w(n) : filter coefficients
% d(n) : desired response
% u(n) : filter input  [wikipedia : x(n)]
% y(n) : filter output 

% intermediate filter variables
% u_bar(n) : inputs [n-M] to [n] (M+1 x 1)
% k_bar(n) : [intermediate variable, i think]
% alpha(n) : [intermediate variable, 1x1]

% w_bar(n) : all filter coefficients for this timestep
% P(n) :  Phi Inverse
 
%not computed (but appears in proof)
% Phi(n) (not computed) : least squares criterion
% Psi(n) (not computed) 
clear all; 

lambda = 0.99;  %forgetting factor lambda

num_sample = 500;  %initial number of samples
M= 20;   %filter size

w_n = zeros(M,1); %initialize the filter

dt = 0.01;  %sample period

n = (0:num_sample-1)';
t = n*dt;  

dc_d = 3; 
w_d = pi/20; %desired frequency
d_n = sin(w_d * n)+dc_d; % sinusoid
phase_lag = 0*pi/180;
d_n_lag = sin(w_d * n-phase_lag); % sinusoid

w_noise = pi/4;  
phase_noise = 0.3421; 
gain_noise = 0.1;
gain_d = 1;
gain_random_noise = 0.02;

u_n = dc_d + gain_d * d_n_lag + gain_noise*sin( w_noise * n + phase_noise) + gain_random_noise*randn(size(n));



%initialize states to zero (w, and P)

delta = 10; 

%initialize the state
init_state.P     = delta*eye(M+1);  %P     : covariance matrix
init_state.x_bar = zeros(M+1,1);    %x_bar : (M+1)by(1) measurement matrix
init_state.w     = zeros(M+1,1);    %w     : filter coefficients matrix

%get the first update
new_meas.x = 1; %test input
new_meas.d = 0.5; %test input

rls_states(num_sample) = update_rls(init_state, new_meas, M, lambda ); %put something in the last element so that matlab will preallocate the state matrix.

%update the RLS for each sample. 
for(i=1:num_sample)
    new_meas.x = u_n(i);
    new_meas.d = d_n(i);
    if(i==1)
        rls_states(i) = update_rls(init_state, new_meas, M, lambda );
    else
        rls_states(i) = update_rls(rls_states(i-1), new_meas, M, lambda );
    end    
end

%make some plots.

y = [rls_states.y]';
d = [rls_states.d]';

figure;
subplot(3,1,1);
plot([y, d, u_n]);
grid on;
xlabel('sample (n)');
ylabel('filter output (unitless)');
legend('output','desired','input');

coefs = [rls_states.w];
subplot(3,1,2);
plot(coefs');
grid on;


freq_response_n = 100;
%find the frequency response of the filter.
figure;
b_matrix = coefs(:,freq_response_n);
a_matrix = zeros(size(b_matrix));
a_matrix(1) = 1;
 freqz(b_matrix,a_matrix,100);
title(sprintf('N = %d\n',freq_response_n));
 
 
 freq_response_n = 300;
%find the frequency response of the filter.
figure;
b_matrix = coefs(:,freq_response_n);
a_matrix = zeros(size(b_matrix));
a_matrix(1) = 1;

 freqz(b_matrix,a_matrix,100);
title(sprintf('N = %d\n',freq_response_n));
 





