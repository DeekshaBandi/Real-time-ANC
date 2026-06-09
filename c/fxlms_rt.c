/* fxlms_rt.c -- single-channel normalized FxLMS (FxNLMS) inner loop in C.
 *
 * A faithful C port of src/fxlms.m, written in the style of a real-time inner
 * loop: per-sample processing with fixed-size tapped-delay buffers and no
 * per-sample allocation. It reads test vectors exported by
 * experiments/export_cref.m, runs the controller, writes the residual error,
 * and prints the steady-state noise reduction so the result can be compared
 * against the MATLAB reference (e_ref.txt).
 *
 * Build: make            (or: cc -O2 -o fxlms_rt fxlms_rt.c -lm)
 * Run:   ./fxlms_rt data
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Read a whitespace/newline-separated list of doubles; returns count via *n. */
static double *read_vec(const char *path, long *n) {
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(1); }
    long cap = 1024, k = 0;
    double *v = malloc(cap * sizeof(double)), val;
    while (fscanf(f, "%lf", &val) == 1) {
        if (k == cap) { cap *= 2; v = realloc(v, cap * sizeof(double)); }
        v[k++] = val;
    }
    fclose(f);
    *n = k;
    return v;
}

int main(int argc, char **argv) {
    const char *dir = (argc > 1) ? argv[1] : "data";
    char path[512];
    long N, Nd, M, Mh;

    #define LOAD(name, var, cnt) \
        snprintf(path, sizeof(path), "%s/" name, dir); \
        double *var = read_vec(path, &cnt);

    LOAD("x.txt",    x,    N);
    LOAD("d.txt",    d,    Nd);
    LOAD("S.txt",    S,    M);
    LOAD("Shat.txt", Shat, Mh);

    /* parameters: L  mu */
    int L; double mu;
    snprintf(path, sizeof(path), "%s/params.txt", dir);
    FILE *fp = fopen(path, "r");
    if (!fp || fscanf(fp, "%d %lf", &L, &mu) != 2) {
        fprintf(stderr, "cannot read params\n"); return 1;
    }
    fclose(fp);
    if (Nd != N) { fprintf(stderr, "x and d length mismatch\n"); return 1; }

    /* Filtered reference xf = Shat(z) * x  (FIR convolution) */
    double *xf = calloc(N, sizeof(double));
    for (long n = 0; n < N; n++) {
        double acc = 0.0;
        long kmax = (n < Mh - 1) ? n : Mh - 1;
        for (long m = 0; m <= kmax; m++) acc += Shat[m] * x[n - m];
        xf[n] = acc;
    }

    /* State buffers (index 0 = most recent sample) */
    double *w    = calloc(L, sizeof(double));   /* control filter        */
    double *xb   = calloc(L, sizeof(double));   /* reference window       */
    double *xfb  = calloc(L, sizeof(double));   /* filtered-ref window    */
    double *ybuf = calloc(M, sizeof(double));   /* past control outputs   */
    double *e    = calloc(N, sizeof(double));

    const double EPS = 2.220446049250313e-16;   /* match MATLAB eps */

    for (long n = 0; n < N; n++) {
        /* shift in current reference sample */
        for (int i = L - 1; i > 0; i--) xb[i] = xb[i - 1];
        xb[0] = x[n];

        /* control output y = w . xb */
        double y = 0.0;
        for (int i = 0; i < L; i++) y += w[i] * xb[i];

        /* propagate y through TRUE secondary path S */
        for (int i = M - 1; i > 0; i--) ybuf[i] = ybuf[i - 1];
        ybuf[0] = y;
        double yp = 0.0;
        for (long i = 0; i < M; i++) yp += S[i] * ybuf[i];

        e[n] = d[n] - yp;

        /* filtered-reference window + normalized LMS update */
        for (int i = L - 1; i > 0; i--) xfb[i] = xfb[i - 1];
        xfb[0] = xf[n];
        double nrm = EPS;
        for (int i = 0; i < L; i++) nrm += xfb[i] * xfb[i];
        double g = (mu / nrm) * e[n];
        for (int i = 0; i < L; i++) w[i] += g * xfb[i];
    }

    /* steady-state reduction over the last 25% (matches fxlms.m) */
    long s0 = (long)floor(0.75 * N);
    if (s0 < 1) s0 = 1;
    double sd = 0.0, se = 0.0; long cnt = 0;
    for (long n = s0; n < N; n++) { sd += d[n]*d[n]; se += e[n]*e[n]; cnt++; }
    double d_rms = sqrt(sd / cnt), e_rms = sqrt(se / cnt);
    double red = 20.0 * log10(d_rms / (e_rms + EPS));

    /* write residual for comparison */
    snprintf(path, sizeof(path), "%s/e_c.txt", dir);
    FILE *fo = fopen(path, "w");
    for (long n = 0; n < N; n++) fprintf(fo, "%.10g\n", e[n]);
    fclose(fo);

    printf("C FxLMS: L=%d mu=%g N=%ld  reduction=%.2f dB\n", L, mu, N, red);
    printf("wrote %s/e_c.txt\n", dir);

    free(x); free(d); free(S); free(Shat); free(xf);
    free(w); free(xb); free(xfb); free(ybuf); free(e);
    return 0;
}
