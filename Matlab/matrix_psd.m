function [ output_matrix , output_freq] = matrix_psd( input_matrix,  window, noverlap, f, fs)
%MATRIX_PSD input a large matrix into pwelch
%   input each column into pwelch and output a matrix with the output

num_col = size(input_matrix,2);



for(i = 1:num_col)
    [temp_p, temp_f] = pwelch(input_matrix(:,i), window, noverlap, f, fs);
    output_matrix(1:length(temp_p),i) = temp_p;
end
output_freq = temp_f;


end

