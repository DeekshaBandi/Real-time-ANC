% SWEEP_STEPSIZE  Phase 5a: step-size (mu) study for FxLMS.
%
%   Shows the fundamental adaptive-filter trade-off: small mu => slow but
%   low-misadjustment convergence; large mu => fast but noisy; too large =>
%   divergence. The secondary-path delay inside the loop caps the stable mu
%   well below the plain-NLMS limit of 2.
%
%   Saves:
%     phase5_stepsize.png  -- learning curves + steady-state reduction vs mu
%
%   Run:  octave --no-gui experiments/sweep_stepsize.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
randn('state', 1); rand('state', 1);

fs = 8000; dur = 1.5;
[x, t] = gen_reference(fs, dur, 120, [1.0 0.5 0.25], 0.01);
[P, S] = make_paths(fs);
d = filter(P, 1, x);
Shat = secondary_path_id(S, numel(S), 0.5, 40000, 1e-3);
L = 256;

% --- learning curves for a few representative step sizes -----------------
mus  = [0.002 0.005 0.02 0.05 0.1];
cols = {'m', 'g', 'b', 'c', 'r'};
win  = 256; ma = ones(win,1)/win;

figure('position', [100 100 950 720]);
subplot(2,1,1); hold on; grid on;
labels = {};
for k = 1:numel(mus)
  [e, ~, learn, info] = fxlms(x, d, S, Shat, L, mus(k));
  lcdb = 10*log10(filter(ma, 1, learn) + eps);
  plot((1:numel(lcdb))/fs, lcdb, cols{k}, 'linewidth', 1);
  labels{end+1} = sprintf('mu=%.3f  (%.0f dB)', mus(k), info.reduction_db);
end
xlabel('time [s]'); ylabel('residual power [dB]'); ylim([-40 40]);
legend(labels, 'location', 'northeast');
title('FxLMS learning curves vs step size (small=slow/clean, large=fast/noisy/unstable)');

% --- steady-state reduction across a denser mu sweep ---------------------
mus2 = [0.001 0.002 0.005 0.01 0.02 0.03 0.05 0.07 0.1 0.15];
red  = zeros(size(mus2));
for k = 1:numel(mus2)
  [~, ~, ~, info] = fxlms(x, d, S, Shat, L, mus2(k));
  red(k) = max(info.reduction_db, -20);    % clamp divergence for the plot
  printf('mu=%6.3f -> %7.1f dB\n', mus2(k), info.reduction_db);
end
subplot(2,1,2);
  semilogx(mus2, red, 'bo-', 'linewidth', 1, 'markerfacecolor', 'b'); grid on;
  xlabel('step size mu (log scale)'); ylabel('steady-state reduction [dB]');
  title('Noise reduction vs step size: sweet spot, then divergence');
print(gcf, fullfile(resdir, 'phase5_stepsize.png'), '-dpng', '-r110');

printf('Saved: results/phase5_stepsize.png\n');
printf('PHASE 5a OK\n');
