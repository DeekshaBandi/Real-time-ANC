function [Shat, learn, info] = secondary_path_id(S, L, mu, N, noise_std)
% SECONDARY_PATH_ID  Offline identification of the secondary path S(z).
%   [Shat, learn, info] = secondary_path_id(S, L, mu, N, noise_std)
%
%   Before FxLMS can run it needs an estimate of the secondary path (control
%   speaker -> error mic). The standard procedure -- and the real-world
%   "acoustic transfer function measurement" -- is an OFFLINE system ID:
%
%       1. Drive the control speaker with a known white-noise probe u(n).
%       2. Record the error-mic response d(n) = S(z)*u(n) (+ measurement noise).
%       3. Adapt an FIR estimate Shat(z) to reproduce d(n) from u(n).
%
%   Adaptation uses normalized LMS (NLMS), which converges robustly regardless
%   of the probe power:
%
%       e(n)    = d(n) - Shat' * u_vec(n)
%       Shat   += (mu / (eps + ||u_vec||^2)) * e(n) * u_vec(n)
%
%   Inputs:
%     S          true secondary path impulse response (column vector)
%     L          number of taps in the estimate Shat   (default numel(S))
%     mu         NLMS step size, 0 < mu < 2            (default 0.5)
%     N          number of probe samples               (default 30000)
%     noise_std  std-dev of additive measurement noise (default 0.001)
%
%   Outputs:
%     Shat   estimated secondary path (column vector, length L)
%     learn  per-sample squared error e(n)^2 (the learning curve)
%     info   struct: final coefficient-error norm, etc.
%
%   Example:
%     [~,S] = make_paths(8000);
%     Shat  = secondary_path_id(S, numel(S), 0.5, 30000, 1e-3);

  if nargin < 2 || isempty(L),         L = numel(S);   end
  if nargin < 3 || isempty(mu),        mu = 0.5;       end
  if nargin < 4 || isempty(N),         N = 30000;      end
  if nargin < 5 || isempty(noise_std), noise_std = 1e-3; end

  S = S(:);

  % 1. White-noise probe and 2. measured response through the TRUE path
  u = randn(N, 1);
  d = filter(S, 1, u) + noise_std * randn(N, 1);

  % 3. NLMS adaptation
  Shat  = zeros(L, 1);
  learn = zeros(N, 1);
  ub    = zeros(L, 1);          % tapped-delay regressor (most recent first)
  for n = 1:N
    ub    = [u(n); ub(1:end-1)];
    yhat  = Shat.' * ub;
    e     = d(n) - yhat;
    Shat  = Shat + (mu / (eps + ub.'*ub)) * e * ub;
    learn(n) = e^2;
  end

  % Coefficient-error norm vs the true path (zero-pad to compare)
  Lmax = max(numel(S), L);
  a = [S; zeros(Lmax-numel(S), 1)];
  b = [Shat; zeros(Lmax-L, 1)];
  info = struct();
  info.coef_err_norm = norm(a - b) / norm(a);     % relative
  info.final_mse_db  = 10*log10(mean(learn(end-min(N,2000)+1:end)) + eps);
  info.L = L; info.mu = mu; info.N = N; info.noise_std = noise_std;
end
