function plot_kicker(dataFile)

% Channel 1 (row 0): current, 100A/V
% Channel 2 (row 1): voltage, 1V/V
% 200ksamples/s

% Find the beginning of discharge
data = dlmread(dataFile,' ');
for start=(1:size(data,2))
   if data(2,start) > 10
       break;
   end
end

% Number of post-trigger (useful) samples
num_samples = size(data,2) - start;

% Sample rate, fixed by scope configuration
sample_rate = 200e3;

% Current sense resistor
rsense = 0.010;

% Time of each sample
tt = linspace(0,num_samples-1,num_samples)./sample_rate;

% Time of last sample
tmax = (num_samples - 1) / sample_rate;

% Current out of the capacitor
i_rlc = data(1, start:end-1) / rsense;

% Voltage across the capacitor and its internal resistance
v = data(2, start:end-1);

% Maximum current
imax = max(i_rlc);

% Index of maximum current
imax_pos = find(i_rlc==imax,1);

% Find the parameters of the damped oscillation with least squares estimation.
% This depends only on the measured current, so it will not be affected by the unknown
% resistances in the capacitor and inductor.


f = fittype('i_rlc-(exp(-X * tt) * sin(X * tt) * X)');

subplot(3,1,1);
plot(tt, i_rlc, 'linewidth', 1, 'color', 'green'); hold on;
plot(tt, fit(tt,i_rlc,f,'StartPoint',[1 1]), 'linewidth', 1, 'color', 'red');

subplot(3,1,2);
plot(tt, v, 'linewidth', 1, 'color', 'green'); hold on;
% plot(tt, v, 'linewidth', 1, 'color', 'red');

subplot(3,1,3);
% plot(tt, i_rlc, 'linewidth', 1, 'color', 'red'); hold on;
% plot(tt, i_rlc, 'linewidth', 1, 'color', 'green'); hold on;
% plot(tt, i_rlc, 'linewidth', 1, 'color', 'blue');






end

