% ROBUSTNESS_PATHERROR  Phase 5b: sensitivity to secondary-path estimation error.
%
%   FxLMS only converges while the phase error between the true path S and the
%   estimate Shat stays under ~90 deg. This script degrades Shat two ways and
%   measures the resulting noise reduction:
%
%     (a) random coefficient error   Shat = S + beta * noise   (sweep beta)
%     (b) pure delay mismatch         Shat = S delayed by k samples (sweep k)
%
%   The delay sweep is the NVH-relevant one: a few samples of unmodeled delay
%   in the secondary path is enough to wreck cancellation, because at the
%   tonal frequency it shows up directly as phase error.
%
%   Saves:
%     phase5_robustness.png
%
%   Run:  octave --no-gui experiments/robustness_patherror.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
set(0, 'defaultfigurevisible', 'off');
resdir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
randn('state', 1); rand('state', 1);

fs = 8000; dur = 1.5; f0 = 120;
[x, t] = gen_reference(fs, dur, f0, [1.0 0.5 0.25], 0.01);
[P, S] = make_paths(fs);
d = filter(P, 1, x);
L = 256; mu = 0.02;

% --- (a) random coefficient error ----------------------------------------
betas = [0 0.05 0.1 0.2 0.4 0.8];
relerr = zeros(size(betas));  redA = zeros(size(betas));
base = randn(numel(S), 1);                 % fixed error shape
for k = 1:numel(betas)
  Shat = S + betas(k) * norm(S)/norm(base) * base;
  relerr(k) = norm(S - Shat)/norm(S) * 100;
  [~, ~, ~, info] = fxlms(x, d, S, Shat, L, mu);
  redA(k) = max(info.reduction_db, -20);
  printf('coef error %.0f%%  -> %6.1f dB\n', relerr(k), info.reduction_db);
end

% --- (b) pure delay mismatch ---------------------------------------------
ks  = -8:2:12;
redB = zeros(size(ks));
for j = 1:numel(ks)
  k = ks(j);
  if k >= 0, Shat = [zeros(k,1); S(1:end-k)];        % delay
  else       Shat = [S(1-k:end); zeros(-k,1)];       % advance
  end
  [~, ~, ~, info] = fxlms(x, d, S, Shat, L, mu);
  redB(j) = max(info.reduction_db, -20);
end
% A k-sample delay error gives phase error 360*f*k/fs deg at frequency f.
% Stability is governed by the HIGHEST tone present (3*f0 = 360 Hz), where
% the phase error is largest -- that is where the ~90 deg limit bites first.
ftop  = 3 * f0;
phdeg = 360 * ftop * ks / fs;

figure('position', [100 100 950 720]);
subplot(2,1,1);
  plot(relerr, redA, 'bo-', 'linewidth', 1, 'markerfacecolor', 'b'); grid on;
  xlabel('secondary-path coefficient error [%]');
  ylabel('reduction [dB]');
  title('(a) Robustness to random Shat error (mu=0.02)');
subplot(2,1,2);
  [ax, h1, h2] = plotyy(ks, redB, ks, phdeg); grid on;
  set(h1, 'marker', 'o', 'linewidth', 1, 'color', 'b');
  set(h2, 'linestyle', '--', 'color', [.6 .6 .6]);
  line(get(ax(2),'xlim'), [ 90  90], 'parent', ax(2), 'color', 'r', 'linestyle', ':');
  line(get(ax(2),'xlim'), [-90 -90], 'parent', ax(2), 'color', 'r', 'linestyle', ':');
  xlabel('secondary-path delay error [samples]');
  ylabel(ax(1), 'reduction [dB]'); ylabel(ax(2), 'phase error @ top harmonic 360 Hz [deg]');
  title('(b) Delay mismatch: cancellation collapses where phase error hits +/-90 deg');
print(gcf, fullfile(resdir, 'phase5_robustness.png'), '-dpng', '-r110');

printf('Saved: results/phase5_robustness.png\n');
printf('PHASE 5b OK\n');
