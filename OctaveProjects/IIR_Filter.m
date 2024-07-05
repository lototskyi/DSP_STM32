% Define Filter cut-off frequency (6KHz)
Cutoff_Freq = 6000;

% Nyquist frequency
Nyq_Freq = fs/2;

cutoff_norm = Cutoff_Freq/Nyq_Freq;

% IIR filter order
order = 8;

% Create low-pass IIR filter
[b,a] = butter(order, cutoff_norm, 'low');

% Get Second Order Coeffs from a and b polynomials
[sos,g] = tf2sos(b,a);

% Prepare Coeffs to be used in CMSIS library, a0 is always 1, remove from sos (4th element)
IIR_Coeff = sos(:, [1 2 3 5 6]);

% CMSIS expects a1 and a2 as negative values
IIR_Coeff(:, 4) = -IIR_Coeff(:, 4);
IIR_Coeff(:, 5) = -IIR_Coeff(:, 5);

% Filter the Input signal with IIR Filter
Filtered_Signal = filter(b, a, Input_signal);

figure(1);
plot(Filtered_Signal);

% Frequency domain
Nfft = 2^14;
CLKfreq = fft(Filtered_Signal, Nfft);

fFFT = fs/2*linspace(0, 1, (Nfft/2)+1);

figure(2);
plot(fFFT, (10*log10(abs(CLKfreq(1:(Nfft/2)+1)))), 'r-', 'LineWidth', 2);

% Write Coeffs into CSV files
csvwrite('CoeffsIIR.csv', IIR_Coeff);
