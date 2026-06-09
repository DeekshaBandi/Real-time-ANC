% DEMO_PHASE3  Secondary-path identification (acoustic transfer-function measurement).
%
%   Estimates Shat(z) of the true secondary path S(z) using a white-noise
%   probe and NLMS, then reports how well it matches and plots:
%     - learning curve (error power vs iteration)
%     - true vs estimated impulse response
%     - true vs estimated frequency response (magnitude + phase)
%
%   Saves:
%     phase3_identification.png
%
%   Run:  octave --no-gui experiments/demo_phase3.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
randn('state', 1); rand('state', 1);    % reproducible

fs = 8000;
[~, S] = make_paths(fs);

L  = numel(S);        % match the true length
mu = 0.5;             % NLMS step
N  = 40000;           % probe samples
ns = 1e-3;            % measurement noise

[Shat, learn, info] = secondary_path_id(S, L, mu, N, ns);

printf('Secondary-path identification:\n');
printf('  taps L              = %d\n', L);
printf('  probe samples N      = %d\n', N);
printf('  NLMS step mu         = %.2f\n', mu);
printf('  measurement noise sd = %g\n', ns);
printf('  relative coef error  = %.4f  (%.1f%%)\n', ...
       info.coef_err_norm, 100*info.coef_err_norm);
printf('  final MSE            = %.1f dB\n', info.final_mse_db);

% smoothed learning curve (moving average) in dB
win  = 200;
sm   = filter(ones(win,1)/win, 1, learn);
lcdb = 10*log10(sm + eps);

% frequency responses
nfft = 1024;
fv   = (0:nfft-1).' * fs / nfft;
hh   = 1:floor(nfft/2);
HS   = fft(S,    nfft);
HSh  = fft(Shat, nfft);

figure('position', [100 100 950 720]);
subplot(2,2,[1 2]);
  plot(lcdb, 'b', 'linewidth', 1); grid on;
  xlabel('iteration'); ylabel('error power [dB]');
  title(sprintf('Identification learning curve (final MSE %.1f dB)', info.final_mse_db));
subplot(2,2,3);
  stem(0:numel(S)-1, S, 'b', 'marker', 'none', 'linewidth', 1); hold on;
  plot(0:numel(Shat)-1, Shat, 'r--', 'linewidth', 1);
  grid on; xlabel('tap'); ylabel('amp');
  legend('true S', 'estimated Shat', 'location', 'northeast');
  title(sprintf('Impulse response (rel. coef error %.1f%%)', 100*info.coef_err_norm));
subplot(2,2,4);
  plot(fv(hh), 20*log10(abs(HS(hh))+eps), 'b', 'linewidth', 1); hold on;
  plot(fv(hh), 20*log10(abs(HSh(hh))+eps), 'r--', 'linewidth', 1);
  grid on; xlim([0 fs/2]); xlabel('frequency [Hz]'); ylabel('mag [dB]');
  legend('true S', 'estimated Shat', 'location', 'southwest');
  title('Frequency response match');
print(gcf, fullfile(resdir, 'phase3_identification.png'), '-dpng', '-r110');

printf('Saved: results/phase3_identification.png\n');
printf('PHASE 3 OK\n');
