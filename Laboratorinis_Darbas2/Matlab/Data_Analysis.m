clc; clear all; 
filepath = "ADC_SawtoothNuskaitymas.txt"
ADC = importdata(filepath);
ADC = transpose(ADC);
L = length (ADC);

%%Data importing

for i = 1:length(ADC)
    x = cell2mat(ADC(i));
    index = 17; 
    while(x(index) ~= ':')
        index = index + 1; 
    end 
    index = index + 2;
    index2 = 1; 
    while(x(index) ~= '<')
        temp(index2) = x(index); 
        index2 = index2 + 1; 
        index = index + 1; 
    end
    ADC_Vertes(i) = str2num(temp);
    clear temp index2 index
end

clear i x filepath ADC

%%Signal plottting

Fd = 100; %100 hz sampling frequency
F = 10; %10Hz sawtooth frequency 

t = 0:1/Fd:L/Fd - 1/Fd;
signal = ADC_Vertes.* 3.3 / 4095;

figure(1) 

subplot(2,1,1)
plot(t,signal);
title('Triangular Signal full')
xlabel('Time (s)')
ylabel('Voltage (V)')
grid on;
xlim([1 10])


subplot(2,1,2)
plot(t,signal);
title('Triangular Signal 1sec')
xlabel('Time (s)')
ylabel('Voltage (V)')
grid on;
xlim([1 2])



%%Calculating FFT 


% Compute FFT
fourier = fft(signal);

% Compute two-sided spectrum P2, then single-sided spectrum P1
P2 = abs(fourier/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

% Frequency domain f
f = Fd*(0:(L/2))/L;

% Phase Spectrum (unwrap to make it smooth)
phase = angle(fourier);                    % Full spectrum
phase_unwrapped = unwrap(phase);    
phase_single = phase_unwrapped(1:L/2+1);


% Plot
figure(2);
subplot(2,1,1)
plot(f, P1)
title('Single-Sided Amplitude Spectrum of Signal')
xlabel('Frequency (Hz)')
ylabel('|P1(f)|')
grid on;
subplot(2,1,2)
plot(f, phase_single)
title('Single-Sided Phase Spectrum')
xlabel('Frequency (Hz)')
ylabel('Phase (radians)')
grid on;







