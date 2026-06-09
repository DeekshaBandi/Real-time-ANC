% EXPORT_CREF  Export test vectors + a MATLAB reference result for the C port.
%
%   Writes the inputs (reference x, disturbance d, true path S, estimated Shat)
%   and parameters to c/data/, plus the MATLAB FxLMS error signal e_ref.txt.
%   The C program (c/fxlms_rt.c) reads the same inputs and must reproduce e.
%
%   Run:  octave --no-gui experiments/export_cref.m

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
root = fullfile(fileparts(mfilename('fullpath')), '..');
ddir = fullfile(root, 'c', 'data');
if ~exist(ddir, 'dir'), mkdir(ddir); end
randn('state', 1); rand('state', 1);

fs = 8000; dur = 1.0;
[x, t] = gen_reference(fs, dur, 120, [1.0 0.5 0.25], 0.01);
[P, S] = make_paths(fs);
d = filter(P, 1, x);
Shat = secondary_path_id(S, numel(S), 0.5, 40000, 1e-3);

L = 256; mu = 0.02;
[e, w, learn, info] = fxlms(x, d, S, Shat, L, mu);
printf('MATLAB reference reduction = %.2f dB\n', info.reduction_db);

names = {'x.txt', 'd.txt', 'S.txt', 'Shat.txt', 'e_ref.txt'};
vecs  = {x, d, S, Shat, e};
for i = 1:numel(names)
  fid = fopen(fullfile(ddir, names{i}), 'w');
  fprintf(fid, '%.10g\n', vecs{i}(:));
  fclose(fid);
end

fid = fopen(fullfile(ddir, 'params.txt'), 'w');
fprintf(fid, '%d %.10g\n', L, mu);     % L  mu
fclose(fid);

printf('Exported to c/data/: x d S Shat e_ref params\n');
printf('EXPORT OK\n');
