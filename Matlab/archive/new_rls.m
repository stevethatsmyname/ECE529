function [ new_state ] = new_rls( M, delta)
%initialize states to zero (w, and P)


%initialize the state
new_state.P     = delta*eye(M+1);  %P     : covariance matrix
new_state.x_bar = zeros(M+1,1);    %x_bar : (M+1)by(1) measurement matrix
new_state.w     = zeros(M+1,1);    %w     : filter coefficients matrix



end

