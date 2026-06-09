function [P, S, info] = make_paths(fs)
% MAKE_PATHS  Build FIR impulse responses for the two acoustic paths in ANC.
%   [P, S, info] = make_paths(fs)
%
%   Returns compact FIR stand-ins for the two acoustic transfer functions in
%   a single-channel ANC system:
%
%     P(z) -- PRIMARY path:   noise source  -> error microphone
%     S(z) -- SECONDARY path: control speaker -> error microphone
%
%   Each is modeled as a pure bulk delay followed by an exponentially-decaying
%   impulse response with a single reflection -- a small, repeatable stand-in
%   for a real measured duct/cabin response. The secondary path is given a
%   shorter delay and faster decay (the control speaker sits closer to the
%   error mic than the noise source does).
%
%   These are the "ground truth" plant. In Phase 3 the controller does NOT get
%   to see S directly: it must *estimate* it (acoustic transfer-function
%   measurement) before FxLMS can run.
%
%   Inputs:
%     fs    sample rate [Hz]   (default 8000)
%
%   Outputs:
%     P     column vector -- primary path impulse response
%     S     column vector -- secondary path impulse response
%     info  struct with the design parameters (for plotting / reporting)
%
%   Example:
%     [P,S] = make_paths(8000);

  if nargin < 1 || isempty(fs), fs = 8000; end

  % --- Primary path: longer delay, longer tail, a reflection -------------
  P = path_ir(fs, ...
              0.004, ...   % bulk delay [s]
              0.010, ...   % decay time constant [s]
              0.040, ...   % total length [s]
              0.45, 0.012);% reflection: gain, extra delay [s]

  % --- Secondary path: shorter delay, faster decay -----------------------
  S = path_ir(fs, ...
              0.0015, ...  % bulk delay [s]
              0.004, ...   % decay time constant [s]
              0.020, ...   % total length [s]
              0.30, 0.005);% reflection: gain, extra delay [s]

  info = struct('fs', fs, 'lenP', numel(P), 'lenS', numel(S));
end

% ---------------------------------------------------------------------------
function h = path_ir(fs, delay_s, tau_s, len_s, refl_gain, refl_delay_s)
% Build one path impulse response: delayed, exponentially-decaying response
% modulated by a mid-band oscillation, plus one delayed reflection.
  N  = round(len_s * fs);
  n  = (0:N-1).';
  d  = round(delay_s * fs);
  tau = tau_s * fs;

  % Direct arrival: decaying, gently oscillatory (bandpass-ish character)
  env  = exp(-(n - d) / tau);
  env(n < d) = 0;
  osc  = sin(2*pi*(n - d) / (0.6*tau));   % slow modulation, bandlimited look
  h    = env .* osc;

  % One reflection: delayed, attenuated copy of the direct arrival
  dr = round(refl_delay_s * fs);
  if dr < N
    h(dr+1:end) = h(dr+1:end) + refl_gain * h(1:end-dr);
  end

  % Normalize to unit peak so path gains are comparable
  h = h / max(abs(h));
end
