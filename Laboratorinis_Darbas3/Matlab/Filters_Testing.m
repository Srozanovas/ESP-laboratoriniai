%%Laboratory 3 FIR IIR filter in esp and matlab comparison 


clc; 
clear all; 

%% 1st step. Generate testing signal. Sampling rate 1000Hz, SquareWave
%%signal - 100Hz. 

%%Generate one square signal then copy to 3 seconds

for i = 1:5
    squareWave(i) = 0; 
end
for i = 6:10
    squareWave(i) = 3850;
end
signal = squareWave;
for i = 1:299
    signal = [signal squareWave];
end 

Fd = 1000; %Sample rate 
T = 3;     %Seconds
t = 0:1/Fd:T-1/Fd; %time vector



%% 2nd step Then generate two filters 9th order FIR and IIR filters.

fir_a = importdata("FIRcoefs.mat");
temp = importdata("IIRcoefs.mat");
G = temp.G; 
SOS = temp.SOS; 
[iir_b, iir_a] = sos2tf(SOS, G); 

clear temp G SOS


%% 3rd step input signal, filtered signal plots and frequency phase responses 

% Compute FFT
N = length(signal);
Y = fft(signal);
f = (0:N-1)*(Fd/N);         % Frequency vector

% Compute magnitude and phase
magnitude = abs(Y)/N;       % Normalize magnitude
phase = angle(Y);           % Phase in radians

% Only keep the first half of the spectrum (up to Nyquist frequency)
half_N = floor(N/2);
f_plot = f(1:half_N);
magnitude_plot = magnitude(1:half_N);
phase_plot = phase(1:half_N);

% Plot magnitude spectrum
figure(1);
subplot(3,1,1);
plot(t, signal);
xlim([0, 0.1])
title('Input signal (theoretical)');
xlabel('Time (s)');
ylabel('Magnitude');
grid on;

subplot(3,1,2);
plot(f_plot, magnitude_plot);
title('Magnitude Spectrum (theoretical)');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;

% Plot phase spectrum
subplot(3,1,3);
plot(f_plot, phase_plot);
title('Phase Spectrum (theoretical)');
xlabel('Frequency (Hz)');
ylabel('Phase (radians)');
grid on;


signal_fir_filtered = filter(fir_a, 1, signal); 


% Compute FFT
N = length(signal_fir_filtered);
Y = fft(signal_fir_filtered);
f = (0:N-1)*(Fd/N);         % Frequency vector

% Compute magnitude and phase
magnitude = abs(Y)/N;       % Normalize magnitude
phase = angle(Y);           % Phase in radians

% Only keep the first half of the spectrum (up to Nyquist frequency)
half_N = floor(N/2);
f_plot = f(1:half_N);
magnitude_plot = magnitude(1:half_N);
phase_plot = phase(1:half_N);

% Plot magnitude spectrum
figure(2);
subplot(3,1,1);
plot(t, signal_fir_filtered);
xlim([0, 0.1])
title('Input signal FIR filtered (theoretical)');
xlabel('Time (s)');
ylabel('Magnitude');
grid on;

subplot(3,1,2);
plot(f_plot, magnitude_plot);
title('Magnitude Spectrum of FIR filtered signal (theoretical)');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;

% Plot phase spectrum
subplot(3,1,3);
plot(f_plot, phase_plot);
title('Phase Spectrum of FIR filtered signal (theoretical)');
xlabel('Frequency (Hz)');
ylabel('Phase (radians)');
grid on;


signal_iir_filtered = filter(iir_b, iir_a, signal); 

% Compute FFT
N = length(signal_iir_filtered);
Y = fft(signal_iir_filtered);
f = (0:N-1)*(Fd/N);         % Frequency vector

% Compute magnitude and phase
magnitude = abs(Y)/N;       % Normalize magnitude
phase = angle(Y);           % Phase in radians

% Only keep the first half of the spectrum (up to Nyquist frequency)
half_N = floor(N/2);
f_plot = f(1:half_N);
magnitude_plot = magnitude(1:half_N);
phase_plot = phase(1:half_N);

% Plot magnitude spectrum
figure(1);
subplot(3,1,1);
plot(t, signal_iir_filtered);
xlim([0, 0.1])
title('Input signal of IIR filtered signal (theoretical)');
xlabel('Time (s)');
ylabel('Magnitude');
grid on;

subplot(3,1,2);
plot(f_plot, magnitude_plot);
title('Magnitude Spectrum of IIR filtered signal (theoretical)');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;

% Plot phase spectrum
subplot(3,1,3);
plot(f_plot, phase_plot);
title('Phase Spectrum of IIR filtered signal (theoretical)');
xlabel('Frequency (Hz)');
ylabel('Phase (radians)');
grid on;




%% 4th step. Calculate filters impulse response and zero pole diagrams for both filters
%Zero poles diagram 
figure(4)
subplot(2,1,1)
zplane(fir_a, 1);
title('Pole-Zero Plot of FIR filter');
grid on;
subplot(2,1,2)
zplane(iir_a, iir_b);
title('Pole-Zero Plot of IIR filter');
grid on;

%Impulse response 
figure(5)
subplot(2,1,1)
impz(fir_a, 1, 30);          % Plot impulse response
title('Impulse Response of the FIR Filter');
grid on;
subplot(2,1,2)
impz(iir_b, iir_a, 30);          % Plot impulse response
title('Impulse Response of the IIR Filter');
grid on;

%% 5th step. GET ESP ADC Sampled signal and Filtered signals.


signal_esp = importdata("..\ADC LOGAI\ADC.txt"); 
signal_fir_filtered_esp = importdata("..\ADC LOGAI\ADCFIIR.txt"); 
signal_iir_filtered_esp = importdata("..\ADC LOGAI\ADCIIR.txt"); 


figure(6)
subplot(3,1,1);
plot(t, signal);
hold on 
plot(t, signal_esp);
xlim([0, 0.1])
title('Input signal vs esp input signal');
xlabel('Time(s)');
ylabel('Magnitude');
legend("Theoretical", "ESP");
grid on;

subplot(3,1,2);
plot(t, signal_fir_filtered);
hold on 
plot(t, signal_fir_filtered_esp);
xlim([0, 0.1])
title('FIR filtered signal vs esp FIR filtered signal');
xlabel('Time(s)');
ylabel('Magnitude');
legend("Theoretical", "ESP");
grid on;

% Plot phase spectrum
subplot(3,1,3);
plot(t, signal_iir_filtered);
hold on 
plot(t, signal_iir_filtered_esp);
xlim([0, 0.1])
title('IIR filtered signal vs esp FIR filtered signal');
xlabel('Time(s)');
ylabel('Magnitude');
legend("Theoretical", "ESP");
grid on;












