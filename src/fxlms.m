function [e, w, learn, info] = fxlms(x, d, S, Shat, L, mu, leak)
% FXLMS  Single-channel Filtered-x LMS active noise controller.
%   [e, w, learn, info] = fxlms(x, d, S, Shat, L, mu, leak)
%
%   Runs the FxLMS adaptive control loop:
%
%       y(n)   = w(n)' * x_vec(n)              % control (anti-noise) output
%       y'(n)  = S(z) * y(n)                   % propagates through TRUE path
%       e(n)   = d(n) - y'(n)                  % residual at the error mic
%       x'(n)  = Shat(z) * x(n)                % FILTERED reference
%       w(n+1) = (1-mu*leak) w(n) + (mu/||x'_vec||^2) e(n) x'_vec(n)
%
%   The update is power-normalized (FxNLMS): dividing by the filtered-
%   reference energy makes the stable step-size range a clean 0 < mu < 2,
%   independent of signal/path scaling. Set the step accordingly.
%
%   The "filtered-x" trick is the whole point: because the control output
%   passes through the secondary path S before reaching the error mic, the
%   correct LMS gradient requires the reference to be filtered by an estimate
%   Shat of that path. Plain LMS (updating with the raw reference) uses the
%   wrong gradient and can diverge -- see lms_anc.m / Phase 6.
%
%   Inputs:
%     x      reference signal seen by the controller (column)
%     d      desired/disturbance signal at the error mic, = P(z)*x (column)
%     S      TRUE secondary path impulse response (physical plant)
%     Shat   ESTIMATED secondary path (from secondary_path_id); used only to
%            filter the reference. Pass S here to study the ideal case.
%     L      control-filter (adaptive FIR) length
%     mu     LMS step size
%     leak   optional leakage coefficient (default 0); >0 adds robustness
%
%   Outputs:
%     e      residual error signal (column) -- the noise that remains
%     w      final control-filter coefficients (L x 1)
%     learn  per-sample squared error e(n)^2 (learning curve)
%     info   struct: reduction_db, d_rms, e_rms
%
%   Example:
%     [e,w,lc,info] = fxlms(x, d, S, Shat, 256, 5e-3);

  if nargin < 7 || isempty(leak), leak = 0; end
  x = x(:); d = d(:); S = S(:); Shat = Shat(:);
  N  = numel(x);
  Ns = numel(S);

  % Filtered reference x'(n) = Shat(z) * x(n) -- can be precomputed (x known)
  xf = filter(Shat, 1, x);

  w     = zeros(L, 1);
  e     = zeros(N, 1);
  learn = zeros(N, 1);
  xb    = zeros(L, 1);     % reference window (for control output)
  xfb   = zeros(L, 1);     % filtered-reference window (for the update)
  ybuf  = zeros(Ns, 1);    % past control outputs (for propagation through S)

  for n = 1:N
    xb  = [x(n);  xb(1:end-1)];
    y   = w.' * xb;                       % anti-noise sample
    ybuf = [y; ybuf(1:end-1)];
    yp  = S.' * ybuf;                     % anti-noise at mic via TRUE path
    e(n) = d(n) - yp;                     % residual
    xfb = [xf(n); xfb(1:end-1)];
    nrm = eps + xfb.'*xfb;                 % filtered-reference energy
    w   = (1 - mu*leak)*w + (mu/nrm) * e(n) * xfb;
    learn(n) = e(n)^2;
  end

  % Headline metric: steady-state noise reduction (last 25% of the record)
  ss = max(1, round(0.75*N)) : N;
  d_rms = sqrt(mean(d(ss).^2));
  e_rms = sqrt(mean(e(ss).^2));
  info = struct();
  info.reduction_db = 20*log10(d_rms / (e_rms + eps));
  info.d_rms = d_rms;
  info.e_rms = e_rms;
  info.L = L; info.mu = mu; info.leak = leak;
end
