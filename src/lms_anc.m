function [e, w, learn, info] = lms_anc(x, d, S, L, mu, leak)
% LMS_ANC  Standard (non-filtered) LMS applied to ANC -- the WRONG way.
%   [e, w, learn, info] = lms_anc(x, d, S, L, mu, leak)
%
%   Identical to fxlms.m EXCEPT the coefficient update uses the raw reference
%   x_vec(n) instead of the secondary-path-filtered reference x'_vec(n):
%
%       w(n+1) = (1-mu*leak) w(n) + (mu/||x_vec||^2) e(n) x_vec(n)  % no filtered-x
%
%   This is included purely as a baseline for Phase 6: it shows WHY FxLMS
%   exists. Because the control output still physically passes through S(z)
%   before the error mic, ignoring S in the update means the gradient is
%   mismatched; the algorithm converges slowly, poorly, or diverges -- the
%   instability grows once S adds more than ~90 deg of phase.
%
%   Inputs / outputs mirror fxlms.m (no Shat: that's the whole point).
%
%   Example:
%     [e,w,lc,info] = lms_anc(x, d, S, 256, 5e-3);

  if nargin < 6 || isempty(leak), leak = 0; end
  x = x(:); d = d(:); S = S(:);
  N  = numel(x);
  Ns = numel(S);

  w     = zeros(L, 1);
  e     = zeros(N, 1);
  learn = zeros(N, 1);
  xb    = zeros(L, 1);
  ybuf  = zeros(Ns, 1);

  for n = 1:N
    xb  = [x(n); xb(1:end-1)];
    y   = w.' * xb;
    ybuf = [y; ybuf(1:end-1)];
    yp  = S.' * ybuf;
    e(n) = d(n) - yp;
    nrm = eps + xb.'*xb;                        % raw-reference energy
    w   = (1 - mu*leak)*w + (mu/nrm) * e(n) * xb;  % raw reference, not filtered
    learn(n) = e(n)^2;
  end

  ss = max(1, round(0.75*N)) : N;
  d_rms = sqrt(mean(d(ss).^2));
  e_rms = sqrt(mean(e(ss).^2));
  info = struct();
  info.reduction_db = 20*log10(d_rms / (e_rms + eps));
  info.d_rms = d_rms; info.e_rms = e_rms;
  info.L = L; info.mu = mu; info.leak = leak;
end
