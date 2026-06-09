function [x, t] = gen_reference(fs, dur, f0, harm_amps, noise_std)
% GEN_REFERENCE  Generate a periodic reference noise signal.
%   [x, t] = gen_reference(fs, dur, f0, harm_amps, noise_std)
%
%   Models the narrowband periodic noise that ANC targets -- e.g. engine
%   firing order or fan blade-pass hum: a fundamental tone at f0 plus a set
%   of harmonics, with optional additive broadband (measurement) noise.
%
%   In a real system this is what a *reference sensor* (tach pulse, ref mic,
%   accelerometer) picks up -- a signal correlated with the unwanted noise.
%
%   Inputs:
%     fs         sample rate [Hz]                         (default 8000)
%     dur        signal duration [s]                      (default 1.0)
%     f0         fundamental frequency [Hz]               (default 120)
%     harm_amps  row vector of harmonic amplitudes;
%                harm_amps(k) scales the k-th harmonic (k*f0).
%                harm_amps(1) is the fundamental.          (default [1])
%     noise_std  std-dev of additive white Gaussian noise
%                (0 => pure tones)                         (default 0)
%
%   Outputs:
%     x   column vector  -- the reference signal
%     t   column vector  -- time stamps [s]
%
%   Example:
%     [x,t] = gen_reference(8000, 1.0, 120, [1 0.5 0.25], 0.01);

  if nargin < 1 || isempty(fs),        fs = 8000;            end
  if nargin < 2 || isempty(dur),       dur = 1.0;            end
  if nargin < 3 || isempty(f0),        f0 = 120;             end
  if nargin < 4 || isempty(harm_amps), harm_amps = 1;        end
  if nargin < 5 || isempty(noise_std), noise_std = 0;        end

  N = round(dur * fs);
  t = (0:N-1).' / fs;

  x = zeros(N, 1);
  for k = 1:numel(harm_amps)
    x = x + harm_amps(k) * sin(2*pi*(k*f0)*t);
  end

  if noise_std > 0
    x = x + noise_std * randn(N, 1);
  end
end
