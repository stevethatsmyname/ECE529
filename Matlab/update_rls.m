function [ new_state ] = update_rls( previous_state, new_meas ,M, lambda)
%Update RLS operates on two structures:
% previous state
% * P(n-1) (M+1 by M+1 matrix)
% * x_bar(n-1) previous input matrix (M+1 by 1 column vector)
% * w(n-1)   previous filter coefficient matrix (M+1 by 1 column vector)
% new measurement
% * x(n) : current measurement
% * d(n) : current desired output
%and two parameters
% M : size of filter
% lambda: forgetting factor (0 to 1)



if(isempty(previous_state))
    %first frame , initialize state
    delta = 10;
    new_state.x_bar = zeros(M+1,1);
    new_state.alpha = 0;
    new_state.g     =  zeros(M+1,1);
    new_state.P     =  delta*eye(M+1);  %P     : covariance matrix
    new_state.w     =  zeros(M+1,1);    %w     : filter coefficients matrix
    new_state.y     = 0;
    new_state.d     = 0;
    
    
else
    
    new_state.x_bar = zeros(M+1,1);
    new_state.alpha = 0;
    new_state.g     =  zeros(M+1,1);
    new_state.P     =  eye(M+1);  %P     : covariance matrix
    new_state.w     =  zeros(M+1,1);    %w     : filter coefficients matrix
    new_state.y     = 0;
    new_state.d     = 0;
    
    %first, check dimensions to make sure they match.
    size_P      = size(previous_state.P);
    size_x_bar  = size(previous_state.x_bar);
    size_w      = size(previous_state.w);
    if( (size_P(1) ~= M+1) || (size_P(2) ~= M+1))
        error('P Size Incorrect');
    end
    
    if( (size_x_bar(1) ~= M+1) || (size_x_bar(2) ~= 1))
        error('x_bar Size Incorrect');
    end
    
    if( (size_w(1) ~= M+1) || (size_w(2) ~= 1))
        error('w Size Incorrect');
    end
    %update state
    new_state.d = new_meas.d;
    new_state.x_bar = [new_meas.x; previous_state.x_bar(1:M)];
    new_state.x_bar(end) = 1;
    
    %update alpha
    % alpha = d(n) + (x(n)') * (w(n-1))
    new_state.alpha = new_meas.d - (new_state.x_bar') * (previous_state.w);
    %alpha does not need to be output but is useful to monitor filter
    %performance
    
    new_state.g = (previous_state.P * new_state.x_bar) * ...
        (lambda + new_state.x_bar' * previous_state.P * new_state.x_bar)^-1;
    
    new_state.P = (1/lambda) * previous_state.P - ...
        (1/lambda) * new_state.g * (new_state.x_bar') * (previous_state.P);
    
    new_state.w = previous_state.w + new_state.alpha * new_state.g;
    
    
    %compute output.
    
    new_state.y = (new_state.x_bar') * new_state.w;
    
    
end

end

