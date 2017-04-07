%M = 7 or 8 (filter length odd or even)

omega_1 = 0.3*pi;
omega_2 = 0.6*pi;


n_inf_odd = -2000:2000; % really, really long n (centered at origin)
n_inf_odd_center = 2001; 
n_odd_inds = (-3:1:3) + n_inf_odd_center

n_inf_even = -1999.5:1:1999.5; % really, really long n even_length
n_inf_even_center = 2000.5; %index of center
n_even_inds = (-3.5:1:3.5) + n_inf_even_center
%filter 1 : Type 1 (odd and symmetric)

h_ideal_type_1 = (1./(pi * n_inf_odd)) .* (sin(omega_2 * n_inf_odd) - sin(omega_1 * n_inf_odd));
h_ideal_type_1(n_inf_odd_center) = omega_2/pi - omega_1/pi;
figure; freqz(h_ideal_type_1)
figure; freqz(h_ideal_type_1 .* hamming(length(h_ideal_type_1))');

h_ideal_type_2 = (1./(pi * n_inf_even)) .* (sin(omega_2 * n_inf_even) - sin(omega_1 * n_inf_even));
figure; freqz(h_ideal_type_2)

h_ideal_type_3 = (1./(pi* n_inf_odd)) .* (cos(omega_2 * n_inf_odd) - cos(omega_1 * n_inf_odd));
h_ideal_type_3(n_inf_odd_center) = 0;

h_ideal_type_4 = (1./(pi* n_inf_even)) .* (cos(omega_2 * n_inf_even) - cos(omega_1 * n_inf_even));
figure; freqz(h_ideal_type_4)

M_odd = 6; M_even = 7;

h_type1_rect = h_ideal_type_1(n_odd_inds);
h_type1_hamming = h_type1_rect.*hamming(M_odd+1)';

h_type2_rect = h_ideal_type_2(n_even_inds);
h_type2_hamming = h_type2_rect.*hamming(M_even+1)';

h_type3_rect = h_ideal_type_3(n_odd_inds);
h_type3_hamming = h_type3_rect.*hamming(M_odd+1)';

h_type4_rect = h_ideal_type_4(n_even_inds);
h_type4_hamming = h_type4_rect.*hamming(M_even+1)';


hamming_mean_odd = mean(hamming(M_odd+1));
hamming_mean_even = mean(hamming(M_even+1));
h_type1_hamming = h_type1_hamming/hamming_mean_odd;
h_type2_hamming = h_type2_hamming/hamming_mean_even;
h_type3_hamming = h_type3_hamming/hamming_mean_odd;
h_type4_hamming = h_type4_hamming/hamming_mean_even;

% plots.
[H_rect(:,1),F] = freqz(h_type1_rect);
[H_rect(:,2),F] = freqz(h_type2_rect);
[H_rect(:,3),F] = freqz(h_type3_rect);
[H_rect(:,4),F] = freqz(h_type4_rect);

[H_hamm(:,1),F] = freqz(h_type1_hamming);
[H_hamm(:,2),F] = freqz(h_type2_hamming);
[H_hamm(:,3),F] = freqz(h_type3_hamming);
[H_hamm(:,4),F] = freqz(h_type4_hamming);



figure; semilogy(F/pi,abs(H_hamm), 'linewidth',2); 
title('Hamming window on filter lengths 7 and 8');
legend('type 1','type 2','type 3','type 4');
xlabel('Digital Frequency / pi');
ylabel('Magnitude of H');
grid on;
figure; semilogy(F/pi,abs(H_rect)); 
title('Rectangular window on filter lengths 7 and 8');
legend('type 1','type 2','type 3','type 4');
xlabel('Digital Frequency / pi');
ylabel('Magnitude of H');
grid on;
