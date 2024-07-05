clear all

% Array for signal sampling
SampArr = [1:1024];

% Sampling frequency
fs = 48000;

% 1000Hz signal
fclk1 = 1000;
clk1 = sin(2*pi*(fclk1/fs)*SampArr);

% 15000Hz signal
fclk2 = 15000;
clk2 = 0.5*sin(2*pi*(fclk2/fs)*SampArr);

Input_signal = clk1 + clk2;

% Write CSV Input Signal file in loadpath
csvwrite('Input_File.csv', Input_signal);

figure(1);
plot(Input_signal);

% Frequency Domain

Nfft = 2^14;
CLKfreq = fft(Input_signal, Nfft);

fFFT = fs/2*linspace(0, 1, (Nfft/2)+1);

figure(2);
plot(fFFT, (10*log10(abs(CLKfreq(1:(Nfft/2)+1)))), 'r-', 'LineWidth', 2);
