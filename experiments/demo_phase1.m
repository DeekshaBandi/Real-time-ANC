% DEMO_PHASE1  Visualize the reference signal and the acoustic paths.
%
%   Generates a periodic reference noise signal and the primary/secondary
%   path models, then saves two figures to results/:
%     phase1_reference.png  -- reference signal: time + spectrum
%     phase1_paths.png      -- P(z) and S(z): impulse + frequency response
%
%   Run:  octave --no-gui experiments/demo_phase1.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');   % headless
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');

% -------------------------------------------------------------------------
% 1. Reference signal: 120 Hz fundamental + 2 harmonics + a little noise
% -------------------------------------------------------------------------
fs  = 8000;
dur = 1.0;
f0  = 120;
[x, t] = gen_reference(fs, dur, f0, [1.0 0.5 0.25], 0.02);

printf('Reference: fs=%d Hz, %.2f s, %d samples, f0=%d Hz\n', ...
       fs, dur, numel(x), f0);
printf('  rms = %.4f, peak = %.4f\n', rms(x), max(abs(x)));

% spectrum (single-sided magnitude)
N    = numel(x);
X    = abs(fft(x)) / N;
f    = (0:N-1).' * fs / N;
half = 1:floor(N/2);

figure('position', [100 100 900 600]);
subplot(2,1,1);
  ms = t*1000;
  plot(ms(ms<=50), x(ms<=50), 'b', 'linewidth', 1);
  grid on; xlabel('time [ms]'); ylabel('amplitude');
  title(sprintf('Reference signal (first 50 ms), f_0 = %d Hz + harmonics', f0));
subplot(2,1,2);
  plot(f(half), 2*X(half), 'b', 'linewidth', 1);
  grid on; xlim([0 800]); xlabel('frequency [Hz]'); ylabel('magnitude');
  title('Reference spectrum (tonal peaks at f_0 and harmonics)');
print(gcf, fullfile(resdir, 'phase1_reference.png'), '-dpng', '-r110');

% -------------------------------------------------------------------------
% 2. Acoustic paths P(z) and S(z)
% -------------------------------------------------------------------------
[P, S] = make_paths(fs);
printf('Primary path   P: %d taps\n', numel(P));
printf('Secondary path S: %d taps\n', numel(S));

nfft = 1024;
fP = (0:nfft-1).' * fs / nfft;
HP = 20*log10(abs(fft(P, nfft)) + eps);
HS = 20*log10(abs(fft(S, nfft)) + eps);
hp = 1:floor(nfft/2);

figure('position', [100 100 900 600]);
subplot(2,2,1);
  stem((0:numel(P)-1)/fs*1000, P, 'b', 'marker', 'none', 'linewidth', 1);
  grid on; xlabel('time [ms]'); ylabel('amp'); title('P(z) impulse response');
subplot(2,2,2);
  stem((0:numel(S)-1)/fs*1000, S, 'r', 'marker', 'none', 'linewidth', 1);
  grid on; xlabel('time [ms]'); ylabel('amp'); title('S(z) impulse response');
subplot(2,2,[3 4]);
  plot(fP(hp), HP(hp), 'b', 'linewidth', 1); hold on;
  plot(fP(hp), HS(hp), 'r', 'linewidth', 1);
  grid on; xlim([0 fs/2]); ylim([-60 10]);
  xlabel('frequency [Hz]'); ylabel('magnitude [dB]');
  legend('P(z) primary', 'S(z) secondary', 'location', 'southwest');
  title('Path frequency responses');
print(gcf, fullfile(resdir, 'phase1_paths.png'), '-dpng', '-r110');

printf('Saved: results/phase1_reference.png, results/phase1_paths.png\n');
printf('PHASE 1 OK\n');
