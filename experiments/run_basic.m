% RUN_BASIC  End-to-end single-channel FxLMS active noise cancellation.
%
%   Ties the whole pipeline together:
%     reference -> primary path (disturbance d) -> FxLMS using the ESTIMATED
%     secondary path -> residual error e, and the headline dB reduction.
%
%   Saves:
%     phase4_fxlms.png   -- error waveform before/after + learning curve (dB)
%
%   Run:  octave --no-gui experiments/run_basic.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
randn('state', 1); rand('state', 1);

% --- build the world ------------------------------------------------------
fs  = 8000;
dur = 3.0;
f0  = 120;
[x, t] = gen_reference(fs, dur, f0, [1.0 0.5 0.25], 0.01);
[P, S] = make_paths(fs);
d = filter(P, 1, x);                    % disturbance at the error mic

% --- measure the secondary path (Phase 3) --------------------------------
Shat = secondary_path_id(S, numel(S), 0.5, 40000, 1e-3);

% --- run FxLMS ------------------------------------------------------------
L    = 256;          % control filter length
mu   = 0.02;         % normalized step (in the stable sweet spot; see Phase 5)
[e, w, learn, info] = fxlms(x, d, S, Shat, L, mu);

printf('FxLMS single-channel ANC:\n');
printf('  control taps L   = %d\n', L);
printf('  step size mu     = %g\n', mu);
printf('  disturbance rms  = %.4f\n', info.d_rms);
printf('  residual   rms   = %.4f\n', info.e_rms);
printf('  >>> NOISE REDUCTION = %.1f dB <<<\n', info.reduction_db);

% --- plots ----------------------------------------------------------------
win  = 256;
lcdb = 10*log10(filter(ones(win,1)/win, 1, learn) + eps);
ms   = t*1000;
seg  = (ms >= 2000) & (ms <= 2080);    % steady-state window (converged)

figure('position', [100 100 950 720]);
subplot(2,1,1);
  plot(ms(seg), d(seg), 'color', [.75 .75 .75], 'linewidth', 1); hold on;
  plot(ms(seg), e(seg), 'b', 'linewidth', 1);
  grid on; xlabel('time [ms]'); ylabel('amplitude');
  legend('disturbance d (ANC off)', 'residual e (ANC on)', 'location', 'northeast');
  title(sprintf('FxLMS steady state: error mic signal, %.1f dB reduction', info.reduction_db));
subplot(2,1,2);
  plot((1:numel(lcdb))/fs, lcdb, 'b', 'linewidth', 1); grid on;
  xlabel('time [s]'); ylabel('residual power [dB]');
  title('Learning curve (convergence of residual error)');
print(gcf, fullfile(resdir, 'phase4_fxlms.png'), '-dpng', '-r110');

printf('Saved: results/phase4_fxlms.png\n');
printf('PHASE 4 OK\n');
