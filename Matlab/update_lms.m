function [ new_state ] = update_LMS( state, new_meas, N, mu, varargin)
%LMS update function : update_lms(state, new_meas, N, update_type)
%   state: previous state of the filter (input [] for first sample)
%   new_meas: structure with elements "x" and "d" (x : input, d: desired
%   output)
%   N : number of samples in the FIR filter
%   mu : is some sort of tuning parameter for the filter.  
if(nargin > 4)
   find_bias = strcmpi(varargin,'init_dc'); %include a coefficient to find the DC bias ( system identification)
   if( ~isempty(find(find_bias)))
       init_dc = true;
   else
       init_dc = false;
   end
   
else
    init_dc = false;
end




if(isempty(state))
    %initialize filter
    new_state.x = zeros(N,1);  %input array (x / u )
    new_state.w = zeros(N,1);  %filter coefficients (w)
    if(init_dc)
        new_state.w(1) = 1;  %initialize to a "DC" filter 
    end
    new_state.err = 0;
    new_state.y = 0;
    
    
else
    
    new_state.x = zeros(N,1);  %input array (x / u )
    new_state.w = zeros(N,1);  %filter coefficients (w)
    new_state.err = 0;
    new_state.y = 0;
     
    x_bar = [new_meas.x; state.x(1:end-1) ];  %'input' array
    w_bar = state.w;
    
    new_state.y = x_bar' * w_bar;
    err = new_meas.d - new_state.y;
    
    w_n_plus_1 = w_bar + mu *(x_bar * err);
    
    
    new_state.w = w_n_plus_1;
    new_state.x = x_bar;
    new_state.err = err;

end



end
