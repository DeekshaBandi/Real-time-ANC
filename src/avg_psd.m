function [f, Pxx] = avg_psd(x, fs, nwin)
% AVG_PSD  Averaged (Welch) one-sided power spectral density. No toolbox.
%   [f, Pxx] = avg_psd(x, fs, nwin)
%
%   Hann-windowed, 50%-overlap averaged periodogram -- a from-scratch stand-in
%   for pwelch so the project has no Signal Processing toolbox dependency.
%
%   Inputs:
%     x     signal (vector)
%     fs    sample rate [Hz]
%     nwin  segment / FFT length (default 1024)
%
%   Outputs:
%     f     one-sided frequency vector [Hz]
%     Pxx   one-sided PSD estimate

  if nargin < 3 || isempty(nwin), nwin = 1024; end
  x = x(:);
  w = 0.5 - 0.5*cos(2*pi*(0:nwin-1).'/(nwin-1));   % Hann
  step = floor(nwin/2);
  nseg = max(1, floor((numel(x)-nwin)/step) + 1);
  Pxx  = zeros(nwin, 1);
  for s = 1:nseg
    seg = x((s-1)*step + (1:nwin)) .* w;
    Pxx = Pxx + abs(fft(seg)).^2;
  end
  Pxx  = Pxx / (nseg * fs * sum(w.^2));
  f    = (0:nwin-1).' * fs / nwin;
  half = 1:floor(nwin/2);
  f    = f(half);
  Pxx  = 2*Pxx(half);
end
