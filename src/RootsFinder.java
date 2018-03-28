//look at me, i know "math"
public class RootsFinder {
    public final static int MAXITER = 5000;

    public static int roots(double[] a, int n, double[] wr, double[] wi) {
        double sq, b2, c, disc;
        int i, m, numroots;

        m = n;
        numroots = 0;
        while (m > 1) {
            b2 = -0.5 * a[m - 2];
            c = a[m - 1];
            disc = b2 * b2 - c;
            if (disc < 0.0) {
                // complex roots
                sq = Math.sqrt(-disc);
                wr[m - 2] = b2;
                wi[m - 2] = sq;
                wr[m - 1] = b2;
                wi[m - 1] = -sq;
                numroots += 2;
            } else {
                // real roots
                sq = Math.sqrt(disc);
                wr[m - 2] = Math.abs(b2) + sq;
                if (b2 < 0.0) {
                    wr[m - 2] = -wr[m - 2];
                }
                if (wr[m - 2] == 0) {
                    wr[m - 1] = 0;
                } else {
                    wr[m - 1] = c / wr[m - 2];
                    numroots += 2;
                }
                wi[m - 2] = 0.0;
                wi[m - 1] = 0.0;
            }
            m -= 2;
        }
        if (m == 1) {
            wr[0] = -a[0];
            wi[0] = 0.0;
            numroots++;
        }
        return numroots;
    }

    public static void deflate(double[] a, int n, double[] b, double[] quad, double[] err) {
        double r, s;

        r = quad[1];
        s = quad[0];

        b[1] = a[1] - r;

        for (int i = 2; i <= n; i++) {
            b[i] = a[i] - r * b[i - 1] - s * b[i - 2];
        }
        err[0] = Math.abs(b[n]) + Math.abs(b[n - 1]);
    }

    public static void find_quad(double[] a, int n, double[] b, double[] quad, double[] err, int[] iter) {
        double[] c;
        double dn, dr = 0, ds = 0, drn, dsn, eps, r, s;

        c = new double[n + 1];
        c[0] = 1.0;
        r = quad[1];
        s = quad[0];
        eps = 1e-15;
        iter[0] = 1;

        do {
            if (iter[0] > MAXITER) {
                break;
            }
            if (iter[0] % 200 == 0) {
                eps *= 10.0;
            }
            b[1] = a[1] - r;
            c[1] = b[1] - r;

            for (int i = 2; i <= n; i++) {
                b[i] = a[i] - r * b[i - 1] - s * b[i - 2];
                c[i] = b[i] - r * c[i - 1] - s * c[i - 2];
            }
            dn = c[n - 1] * c[n - 3] - c[n - 2] * c[n - 2];
            drn = b[n] * c[n - 3] - b[n - 1] * c[n - 2];
            dsn = b[n - 1] * c[n - 1] - b[n] * c[n - 2];

            if (Math.abs(dn) < 1e-10) {
                if (dn < 0.0) {
                    dn = -1e-8;
                } else {
                    dn = 1e-8;
                }
            }
            dr = drn / dn;
            ds = dsn / dn;
            r += dr;
            s += ds;
            iter[0] = iter[0] + 1;
        } while (Math.abs(dr) + Math.abs(ds) > eps);
        quad[0] = s;
        quad[1] = r;
        err[0] = Math.abs(ds) + Math.abs(dr);
    }

    public static void diff_poly(double[] a, int n, double[] b) {
        double coef;

        coef = (double) n;
        b[0] = 1.0;
        for (int i = 1; i < n; i++) {
            b[i] = a[i] * ((double) (n - i)) / coef;
        }
    }

    public static void recurse(double[] a, int n, double[] b, int m, double[] quad, double[] err, int[] iter) {
        double[] c, x, rs = new double[2];
        double tst, e1, e2;

        if (Math.abs(b[m]) < 1e-16) {
            m--;
        }
        if (m == 2) {
            quad[0] = b[2];
            quad[1] = b[1];
            err[0] = 0;
            iter[0] = 0;
            return;
        }
        c = new double[m + 1];
        x = new double[n + 1];
        c[0] = (x[0] = 1.0);
        rs[0] = quad[0];
        rs[1] = quad[1];
        iter[0] = 0;
        find_quad(b, m, c, rs, err, iter);
        tst = Math.abs(rs[0] - quad[0]) + Math.abs(rs[1] - quad[1]);
        if (err[0] < 1e-12) {
            quad[0] = rs[0];
            quad[1] = rs[1];
        }

        if (iter[0] > 5 && tst < 1e-4 || iter[0] > 20 && tst < 1e-1) {
            diff_poly(b, m, c);
            recurse(a, n, c, m - 1, rs, err, iter);
            quad[0] = rs[0];
            quad[1] = rs[1];
        }
    }

    public static void get_quads(double a[], int n, double quad[], double x[]) {
        double b[], z[], tmp;
        double xr, xs;
        int i, m;

        double[] err = {0};
        int[] iter = {0};

        if ((tmp = a[0]) != 1.0) {
            a[0] = 1.0;
            for (i = 1; i <= n; i++) {
                a[i] /= tmp;
            }
        }
        if (n == 2) {
            x[0] = a[1];
            x[1] = a[2];
            return;
        } else if (n == 1) {
            x[0] = a[1];
            return;
        }
        m = n;
        b = new double[n + 1];
        z = new double[n + 1];
        b[0] = 1.0;
        for (i = 0; i <= n; i++) {
            z[i] = a[i];
            x[i] = 0.0;
        }
        do {
            if (n > m) {
                quad[0] = 3.14159e-1;
                quad[1] = 2.78127e-1;
            }
            do {                    // This loop tries to assure convergence
                for (i = 0; i < 5; i++) {
                    find_quad(z, m, b, quad, err, iter);
                    if ((err[0] > 1e-7) || (iter[0] > MAXITER)) {
                        diff_poly(z, m, b);
                        recurse(z, m, b, m - 1, quad, err, iter);
                    }
                    deflate(z, m, b, quad, err);
                    if (err[0] < 0.001) break;
                    // quad[0] = random(8) - 4.0;
                    // quad[1] = random(8)
                    quad[0] = Math.random() * 8 - 4.0;
                    quad[1] = Math.random() * 8 - 4.0;
                }
                if (err[0] > 0.01) {
                    System.out.println("convergence failure");
                }
            } while (err[0] > 0.01);
            x[m - 2] = quad[1];
            x[m - 1] = quad[0];
            m -= 2;
            for (i = 0; i <= m; i++) {
                z[i] = b[i];
            }
        } while (m > 2);
        if (m == 2) {
            x[0] = b[1];
            x[1] = b[2];
        } else x[0] = b[1];
    }

    public static double findRoot(double coeffs[], int n) {
        double[] x = new double[n+1];
        double wr[] = new double[n+1];
        double wi[] = new double[n+1];
        double quad[] = new double[2];
        int numr;

        // initialize estimate for 1st root pair
        quad[0] = 2.71828e-1;
        quad[1] = 3.14159e-1;

        // get roots
        get_quads(coeffs, n, quad, x);
        numr = roots(x, n, wr, wi);

        // Return only the (supposed to be unique) real positive root
        for (int i = 0; i < n;i++) {
            if ((wr[i] >= 0.0) && (wi[i] == 0.0))
                //System.out.println(wr[i]);
                return wr[i];
        }

        return 0;
    }
}
