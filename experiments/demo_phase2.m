% DEMO_PHASE2  Windowed-sinc FIR lowpass design.
%
%   Designs lowpass FIR filters with different windows and shows the classic
%   FIR design trade-off (transition width vs stopband attenuation), then
%   demonstrates the filter cleaning broadband noise off a tonal signal.
%
%   Saves:
%     phase2_window_compare.png  -- impulse + magnitude response per window
%     phase2_filtering.png       -- a filter applied to a noisy signal
%
%   Run:  octave --no-gui experiments/demo_phase2.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');

fs       = 8000;
fc       = 1000;
numtaps  = 101;
windows  = {'rect', 'hann', 'hamming', 'blackman'};
colors   = {'k', 'g', 'b', 'r'};

% -------------------------------------------------------------------------
% 1. Compare windows: impulse response + magnitude response
% -------------------------------------------------------------------------
nfft = 4096;
fvec = (0:nfft-1).' * fs / nfft;
half = 1:floor(nfft/2);

figure('position', [100 100 950 650]);
subplot(2,1,1); hold on; grid on;
for k = 1:numel(windows)
  h = fir_lowpass(numtaps, fc, fs, windows{k});
  stem(0:numtaps-1, h, colors{k}, 'marker', 'none', 'linewidth', 1);
end
xlabel('tap index'); ylabel('coefficient');
title(sprintf('Lowpass FIR impulse responses (%d taps, fc=%d Hz)', numtaps, fc));
legend(windows, 'location', 'northeast');

subplot(2,1,2); hold on; grid on;
atten = struct();
for k = 1:numel(windows)
  h  = fir_lowpass(numtaps, fc, fs, windows{k});
  H  = 20*log10(abs(fft(h, nfft)) + eps);
  plot(fvec(half), H(half), colors{k}, 'linewidth', 1);
  % crude stopband attenuation: worst (max) level above 1.5*fc
  sb = fvec(half) > 1.5*fc;
  atten.(windows{k}) = max(H(half(sb)));
end
xlabel('frequency [Hz]'); ylabel('magnitude [dB]');
xlim([0 fs/2]); ylim([-120 5]);
line([fc fc], [-120 5], 'color', [.5 .5 .5], 'linestyle', '--');
title('Magnitude responses -- wider window taper => more stopband attenuation');
legend(windows, 'location', 'southwest');
print(gcf, fullfile(resdir, 'phase2_window_compare.png'), '-dpng', '-r110');

printf('Stopband attenuation (max level above %.0f Hz):\n', 1.5*fc);
for k = 1:numel(windows)
  printf('  %-9s : %6.1f dB\n', windows{k}, atten.(windows{k}));
end

% -------------------------------------------------------------------------
% 2. Apply the filter: strip broadband noise off a tonal signal
% -------------------------------------------------------------------------
[x, t] = gen_reference(fs, 0.25, 300, [1.0 0.4], 0.6);   % heavy noise
h = fir_lowpass(numtaps, 1000, fs, 'hamming');
y = filter(h, 1, x);

% compensate the known linear-phase group delay for visual alignment
gd = (numtaps-1)/2;
yc = [y(gd+1:end); zeros(gd,1)];

Nx  = numel(x);
fx  = (0:Nx-1).' * fs / Nx;
hx  = 1:floor(Nx/2);
Xf  = abs(fft(x))/Nx;  Yf = abs(fft(yc))/Nx;

figure('position', [100 100 950 650]);
subplot(2,1,1);
  ms = t*1000;
  plot(ms, x, 'color', [.7 .7 .7]); hold on; grid on;
  plot(ms, yc, 'b', 'linewidth', 1);
  xlim([0 30]); xlabel('time [ms]'); ylabel('amplitude');
  legend('noisy input', 'lowpass output', 'location', 'northeast');
  title('Windowed-sinc lowpass removing broadband noise (time domain)');
subplot(2,1,2);
  plot(fx(hx), 2*Xf(hx), 'color', [.7 .7 .7]); hold on; grid on;
  plot(fx(hx), 2*Yf(hx), 'b', 'linewidth', 1);
  xlim([0 fs/2]); xlabel('frequency [Hz]'); ylabel('magnitude');
  legend('noisy input', 'lowpass output', 'location', 'northeast');
  title('Spectrum: tones preserved, high-frequency noise attenuated');
print(gcf, fullfile(resdir, 'phase2_filtering.png'), '-dpng', '-r110');

printf('Saved: results/phase2_window_compare.png, results/phase2_filtering.png\n');
printf('PHASE 2 OK\n');
