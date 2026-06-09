function h = fir_lowpass(numtaps, fc, fs, window)
% FIR_LOWPASS  Design a linear-phase lowpass FIR filter by the window method.
%   h = fir_lowpass(numtaps, fc, fs, window)
%
%   Built from first principles (no Signal Processing toolbox): the impulse
%   response is the ideal lowpass sinc, truncated to numtaps and tapered by a
%   window to trade transition width against stopband attenuation.
%
%   The filter length is forced ODD so the result is a Type-I linear-phase
%   filter with an integer group delay of (numtaps-1)/2 samples -- which makes
%   it easy to align/compensate elsewhere in the pipeline.
%
%   Inputs:
%     numtaps  desired number of taps (forced to the next odd number)
%     fc       cutoff frequency [Hz] (-6 dB point of the ideal response)
%     fs       sample rate [Hz]
%     window   'hamming' (default) | 'hann' | 'blackman' | 'rect'
%
%   Output:
%     h        column vector of FIR coefficients, normalized to unit DC gain
%
%   Example:
%     h = fir_lowpass(101, 1000, 8000, 'hamming');

  if nargin < 4 || isempty(window), window = 'hamming'; end
  if mod(numtaps, 2) == 0, numtaps = numtaps + 1; end   % force odd (Type I)

  M   = numtaps - 1;
  n   = (0:M).';
  fcn = fc / fs;                 % normalized cutoff in cycles/sample (0..0.5)

  % Ideal lowpass impulse response (shifted sinc), centered at M/2
  hd = 2*fcn * sinc_(2*fcn*(n - M/2));

  % Apply the window
  w = make_window(numtaps, window);
  h = hd .* w;

  % Normalize so the passband (DC) gain is exactly 1
  h = h / sum(h);
end

% ---------------------------------------------------------------------------
function y = sinc_(x)
% Normalized sinc: sin(pi x)/(pi x), with the removable singularity at x=0.
  y = ones(size(x));
  nz = (x ~= 0);
  y(nz) = sin(pi*x(nz)) ./ (pi*x(nz));
end

% ---------------------------------------------------------------------------
function w = make_window(N, name)
% Common windows, implemented directly so no toolbox is required.
  M = N - 1;
  n = (0:M).';
  switch lower(name)
    case 'rect'
      w = ones(N, 1);
    case 'hann'
      w = 0.5 - 0.5*cos(2*pi*n/M);
    case 'hamming'
      w = 0.54 - 0.46*cos(2*pi*n/M);
    case 'blackman'
      w = 0.42 - 0.5*cos(2*pi*n/M) + 0.08*cos(4*pi*n/M);
    otherwise
      error('fir_lowpass: unknown window "%s"', name);
  end
end
