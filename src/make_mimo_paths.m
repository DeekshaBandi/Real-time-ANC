function [Pj, Sjk, info] = make_mimo_paths(fs, J, K)
% MAKE_MIMO_PATHS  Acoustic paths for a multichannel (MIMO) ANC system.
%   [Pj, Sjk, info] = make_mimo_paths(fs, J, K)
%
%   Builds the paths for a system with one noise source / reference, J error
%   microphones and K control actuators (loudspeakers):
%
%     Pj{j}    primary path:   noise source -> error mic j
%     Sjk{j,k} secondary path: actuator k   -> error mic j
%
%   The secondary-path matrix has strong DIRECT (j==k) paths and weaker,
%   more-delayed CROSS paths (j~=k). The cross terms are what make the problem
%   genuinely MIMO: each actuator leaks into every error mic, so the K control
%   filters cannot be tuned independently.
%
%   Inputs:
%     fs   sample rate [Hz]      (default 8000)
%     J    number of error mics  (default 2)
%     K    number of actuators   (default 2)
%
%   Outputs:
%     Pj   J x 1 cell of primary-path impulse responses
%     Sjk  J x K cell of secondary-path impulse responses
%     info struct with sizes
%
%   Example:
%     [Pj, Sjk] = make_mimo_paths(8000, 2, 2);

  if nargin < 1 || isempty(fs), fs = 8000; end
  if nargin < 2 || isempty(J),  J  = 2;    end
  if nargin < 3 || isempty(K),  K  = 2;    end

  % Primary paths: one per error mic, slightly different delay/decay
  Pj = cell(J, 1);
  for j = 1:J
    Pj{j} = path_ir(fs, 0.004 + 0.0010*(j-1), 0.010, 0.040, 0.40, 0.012);
  end

  % Secondary paths: direct (j==k) strong & prompt; cross (j~=k) weaker & later
  Sjk = cell(J, K);
  for j = 1:J
    for k = 1:K
      if j == k
        h = path_ir(fs, 0.0015, 0.004, 0.020, 0.30, 0.005);
        g = 1.0;
      else
        h = path_ir(fs, 0.0030, 0.005, 0.020, 0.30, 0.006);
        g = 0.5;                      % cross-coupling is weaker
      end
      Sjk{j,k} = g * h;
    end
  end

  info = struct('fs', fs, 'J', J, 'K', K);
end

% ---------------------------------------------------------------------------
function h = path_ir(fs, delay_s, tau_s, len_s, refl_gain, refl_delay_s)
% Same compact path model used in make_paths.m: delayed, decaying, gently
% oscillatory impulse response plus one reflection, normalized to unit peak.
  N = round(len_s*fs); n = (0:N-1).';
  d = round(delay_s*fs); tau = tau_s*fs;
  env = exp(-(n-d)/tau); env(n<d) = 0;
  h   = env .* sin(2*pi*(n-d)/(0.6*tau));
  dr  = round(refl_delay_s*fs);
  if dr < N, h(dr+1:end) = h(dr+1:end) + refl_gain*h(1:end-dr); end
  h = h / max(abs(h));
end
