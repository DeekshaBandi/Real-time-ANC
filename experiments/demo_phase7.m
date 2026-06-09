% DEMO_PHASE7  2x2 multichannel (MIMO) FxLMS active noise cancellation.
%
%   One reference, two error mics, two actuators, with cross-coupled secondary
%   paths. Each secondary path S_{jk} is identified separately, then the
%   coupled multichannel FxLMS drives both error mics down together.
%
%   Saves:
%     phase7_mimo.png  -- total learning curve, per-mic reduction, steady-state
%
%   Run:  octave --no-gui experiments/demo_phase7.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
randn('state', 1); rand('state', 1);

fs = 8000; dur = 2.0; f0 = 120;
J = 2; K = 2;
[x, t] = gen_reference(fs, dur, f0, [1.0 0.5 0.25], 0.01);
[Pj, Sjk] = make_mimo_paths(fs, J, K);

% disturbance at each error mic
D = zeros(numel(x), J);
for j = 1:J, D(:,j) = filter(Pj{j}, 1, x); end

% identify every secondary path S_{jk} (acoustic transfer-function measurement)
Shatjk = cell(J, K);
for j = 1:J
  for k = 1:K
    Shatjk{j,k} = secondary_path_id(Sjk{j,k}, numel(Sjk{j,k}), 0.5, 30000, 1e-3);
  end
end

L = 128; mu = 0.02;
[E, W, learn, info] = fxlms_mimo(x, D, Sjk, Shatjk, L, mu);

printf('2x2 MIMO FxLMS (J=%d mics, K=%d actuators):\n', J, K);
for j = 1:J
  printf('  error mic %d reduction = %5.1f dB\n', j, info.reduction_db(j));
end
printf('  >>> OVERALL REDUCTION = %.1f dB <<<\n', info.overall_db);

% --- plots ----------------------------------------------------------------
win  = 256; ma = ones(win,1)/win;
lcdb = 10*log10(filter(ma,1,learn) + eps);
ms   = t*1000; seg = (ms >= 1500) & (ms <= 1560);

figure('position', [100 100 980 760]);
subplot(2,2,[1 2]);
  plot((1:numel(lcdb))/fs, lcdb, 'b', 'linewidth', 1); grid on;
  xlabel('time [s]'); ylabel('total residual power [dB]');
  title(sprintf('2x2 MIMO FxLMS learning curve (overall %.1f dB)', info.overall_db));
subplot(2,2,3);
  bar(info.reduction_db); grid on;
  set(gca, 'xticklabel', {'mic 1','mic 2'});
  ylabel('reduction [dB]'); title('Per-microphone reduction');
subplot(2,2,4);
  plot(ms(seg), D(seg,1), 'color', [.75 .75 .75], 'linewidth', 1); hold on;
  plot(ms(seg), E(seg,1), 'b', 'linewidth', 1);
  grid on; xlabel('time [ms]'); ylabel('amp');
  legend('mic1 off','mic1 on','location','northeast');
  title('Mic 1 steady-state waveform');
print(gcf, fullfile(resdir, 'phase7_mimo.png'), '-dpng', '-r110');

printf('Saved: results/phase7_mimo.png\n');
printf('PHASE 7 OK\n');
