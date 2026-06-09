function [E, W, learn, info] = fxlms_mimo(x, D, Sjk, Shatjk, L, mu)
% FXLMS_MIMO  Multichannel (MIMO) Filtered-x LMS active noise controller.
%   [E, W, learn, info] = fxlms_mimo(x, D, Sjk, Shatjk, L, mu)
%
%   One reference signal, J error microphones, K control actuators. Each
%   actuator output reaches every error mic through its own secondary path,
%   so the K adaptive filters are coupled and must be updated jointly.
%
%       y_k(n)   = w_k(n)' * x_vec(n)                      % actuator k output
%       e_j(n)   = d_j(n) - sum_k  S_{jk}(z) * y_k(n)      % residual at mic j
%       x'_{jk}  = Shat_{jk}(z) * x(n)                     % filtered reference
%       w_k     += (mu / norm_k) * sum_j e_j(n) * x'_{jk,vec}(n)
%
%   The update for actuator k accumulates the filtered-reference-weighted error
%   over ALL error mics -- this cross term is the essence of multichannel FxLMS
%   (Elliott's algorithm). It reduces to the single-channel fxlms.m for J=K=1.
%
%   Inputs:
%     x       reference signal (N x 1)
%     D       disturbance at each error mic (N x J), d_j = P_j(z)*x
%     Sjk     J x K cell of TRUE secondary-path impulse responses
%     Shatjk  J x K cell of ESTIMATED secondary paths (filter the reference)
%     L       control-filter length (per actuator)
%     mu      normalized step size (0 < mu < 2)
%
%   Outputs:
%     E       residual error per mic (N x J)
%     W       final control filters (L x K)
%     learn   total residual power across mics, per sample (N x 1)
%     info    struct: per-mic and overall reduction in dB
%
%   Example:
%     [Pj,Sjk] = make_mimo_paths(8000,2,2);
%     ... build x, D, Shatjk ...
%     [E,W,lc,info] = fxlms_mimo(x, D, Sjk, Shatjk, 128, 0.02);

  x = x(:); N = numel(x);
  [J, K] = size(Sjk);

  % Precompute filtered references x'_{jk} = Shat_{jk} * x
  XF = cell(J, K);
  for j = 1:J, for k = 1:K, XF{j,k} = filter(Shatjk{j,k}, 1, x); end, end

  % Max secondary-path length (for the actuator output history buffers)
  maxS = 0;
  for j = 1:J, for k = 1:K, maxS = max(maxS, numel(Sjk{j,k})); end, end

  W     = zeros(L, K);
  E     = zeros(N, J);
  learn = zeros(N, 1);
  xb    = zeros(L, 1);              % shared reference window
  ybuf  = zeros(maxS, K);          % past outputs of each actuator
  xfb   = cell(J, K);              % filtered-reference windows
  for j = 1:J, for k = 1:K, xfb{j,k} = zeros(L,1); end, end

  for n = 1:N
    xb = [x(n); xb(1:end-1)];

    % actuator outputs and their propagation to each error mic
    y = zeros(K, 1);
    for k = 1:K
      y(k) = W(:,k).' * xb;
      ybuf(:,k) = [y(k); ybuf(1:end-1, k)];
    end
    e = zeros(J, 1);
    for j = 1:J
      yp = 0;
      for k = 1:K
        s = Sjk{j,k};
        yp = yp + s.' * ybuf(1:numel(s), k);
      end
      e(j) = D(n,j) - yp;
    end
    E(n,:) = e.';
    learn(n) = sum(e.^2);

    % update each control filter using errors at ALL mics
    for k = 1:K
      grad = zeros(L, 1); nrm = eps;
      for j = 1:J
        xfb{j,k} = [XF{j,k}(n); xfb{j,k}(1:end-1)];
        grad = grad + e(j) * xfb{j,k};
        nrm  = nrm + xfb{j,k}.' * xfb{j,k};
      end
      W(:,k) = W(:,k) + (mu / nrm) * grad;
    end
  end

  % per-mic and overall steady-state reduction
  ss = max(1, round(0.75*N)) : N;
  info = struct(); info.J = J; info.K = K; info.L = L; info.mu = mu;
  info.reduction_db = zeros(J,1);
  for j = 1:J
    info.reduction_db(j) = 20*log10( sqrt(mean(D(ss,j).^2)) / ...
                                     (sqrt(mean(E(ss,j).^2)) + eps) );
  end
  info.overall_db = 10*log10( sum(sum(D(ss,:).^2)) / (sum(sum(E(ss,:).^2)) + eps) );
end
