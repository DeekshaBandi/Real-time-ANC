% COMPARE_LMS_FXLMS  Phase 6: why FxLMS exists + before/after spectrum.
%
%   Runs standard LMS (raw reference in the update) and FxLMS (filtered
%   reference) under IDENTICAL conditions. Standard LMS diverges because the
%   control signal passes through the secondary path S(z): ignoring S in the
%   gradient leaves a phase mismatch the loop cannot tolerate. Filtering the
%   reference by Shat(z) fixes the gradient -- and the noise collapses.
%
%   Also shows the frequency-domain payoff: the tonal peaks at the error mic
%   before vs after cancellation.
%
%   Saves:
%     phase6_lms_vs_fxlms.png  -- learning curves, LMS vs FxLMS
%     phase6_spectrum.png      -- error-mic PSD before vs after FxLMS
%
%   Run:  octave --no-gui experiments/compare_lms_fxlms.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
randn('state', 1); rand('state', 1);

fs = 8000; dur = 1.5; f0 = 120;
[x, t] = gen_reference(fs, dur, f0, [1.0 0.5 0.25], 0.01);
[P, S] = make_paths(fs);
d = filter(P, 1, x);
Shat = secondary_path_id(S, numel(S), 0.5, 40000, 1e-3);
L = 256; mu = 0.005;          % SAME step size for both algorithms

[eF, ~, lcF, iF] = fxlms(x,   d, S, Shat, L, mu);
[eL, ~, lcL, iL] = lms_anc(x,  d, S,      L, mu);

printf('Same conditions (mu=%.3f, L=%d):\n', mu, L);
printf('  FxLMS  reduction = %7.1f dB\n', iF.reduction_db);
printf('  LMS    reduction = %7.1f dB  (diverges)\n', iL.reduction_db);

win = 256; ma = ones(win,1)/win;
lcFdb = 10*log10(filter(ma,1,lcF) + eps);
lcLdb = 10*log10(filter(ma,1,lcL) + eps);

figure('position', [100 100 950 520]);
  plot((1:numel(lcLdb))/fs, lcLdb, 'r', 'linewidth', 1); hold on;
  plot((1:numel(lcFdb))/fs, lcFdb, 'b', 'linewidth', 1); grid on;
  ylim([-40 80]);
  xlabel('time [s]'); ylabel('residual power [dB]');
  legend('standard LMS (diverges)', 'FxLMS (converges)', 'location', 'east');
  title(sprintf('Why FxLMS: identical mu=%.3f -- LMS blows up, FxLMS gives %.0f dB', ...
        mu, iF.reduction_db));
print(gcf, fullfile(resdir, 'phase6_lms_vs_fxlms.png'), '-dpng', '-r110');

% --- before/after spectrum for FxLMS (steady state) ----------------------
ss   = round(0.6*numel(d)) : numel(d);     % converged portion
[fD, PD] = avg_psd(d(ss),  fs, 1024);
[fE, PE] = avg_psd(eF(ss), fs, 1024);

figure('position', [100 100 950 520]);
  plot(fD, 10*log10(PD+eps), 'color', [.6 .6 .6], 'linewidth', 1.2); hold on;
  plot(fE, 10*log10(PE+eps), 'b', 'linewidth', 1.2); grid on;
  xlim([0 800]); xlabel('frequency [Hz]'); ylabel('power spectral density [dB]');
  legend('ANC off (disturbance)', 'ANC on (FxLMS residual)', 'location', 'northeast');
  title('Error-mic spectrum: tonal peaks at 120/240/360 Hz knocked down');
print(gcf, fullfile(resdir, 'phase6_spectrum.png'), '-dpng', '-r110');

printf('Saved: results/phase6_lms_vs_fxlms.png, results/phase6_spectrum.png\n');
printf('PHASE 6 OK\n');
