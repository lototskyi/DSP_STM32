% Define filter cut-off frequency (6KHz)
CutOff_Freq = 6000;

% Nyquist frequency
Nyq_Freq = fs/2;
cutoff_norm = CutOff_Freq/Nyq_Freq;

% FIR filter order
order = 28;

% Create low-pass FIR filter
FIR_Coeff = fir1(order, cutoff_norm);

% Filter the Input signal with the FIR filter
Filtered_signal = filter(FIR_Coeff, 1, Input_signal);

% Write CSV Coeff file
csvwrite('CoeffFIR.csv', FIR_Coeff);
