/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadrotor_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_house CASADI_PREFIX(house)
#define casadi_if_else CASADI_PREFIX(if_else)
#define casadi_qr CASADI_PREFIX(qr)
#define casadi_qr_colcomb CASADI_PREFIX(qr_colcomb)
#define casadi_qr_mv CASADI_PREFIX(qr_mv)
#define casadi_qr_singular CASADI_PREFIX(qr_singular)
#define casadi_qr_solve CASADI_PREFIX(qr_solve)
#define casadi_qr_trs CASADI_PREFIX(qr_trs)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_scal CASADI_PREFIX(scal)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

casadi_real casadi_if_else(casadi_real c, casadi_real x, casadi_real y) { return c!=0 ? x : y;}

void casadi_scal(casadi_int n, casadi_real alpha, casadi_real* x) {
  casadi_int i;
  if (!x) return;
  for (i=0; i<n; ++i) *x++ *= alpha;
}

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

casadi_real casadi_house(casadi_real* v, casadi_real* beta, casadi_int nv) {
  casadi_int i;
  casadi_real v0, sigma, s, sigma_is_zero, v0_nonpos;
  v0 = v[0];
  sigma=0;
  for (i=1; i<nv; ++i) sigma += v[i]*v[i];
  s = sqrt(v0*v0 + sigma);
  sigma_is_zero = sigma==0;
  v0_nonpos = v0<=0;
  v[0] = casadi_if_else(sigma_is_zero, 1,
                 casadi_if_else(v0_nonpos, v0-s, -sigma/(v0+s)));
  *beta = casadi_if_else(sigma_is_zero, 2*v0_nonpos, -1/(s*v[0]));
  return s;
}
void casadi_qr(const casadi_int* sp_a, const casadi_real* nz_a, casadi_real* x,
               const casadi_int* sp_v, casadi_real* nz_v, const casadi_int* sp_r, casadi_real* nz_r, casadi_real* beta,
               const casadi_int* prinv, const casadi_int* pc) {
   casadi_int ncol, nrow, r, c, k, k1;
   casadi_real alpha;
   const casadi_int *a_colind, *a_row, *v_colind, *v_row, *r_colind, *r_row;
   ncol = sp_a[1];
   a_colind=sp_a+2; a_row=sp_a+2+ncol+1;
   nrow = sp_v[0];
   v_colind=sp_v+2; v_row=sp_v+2+ncol+1;
   r_colind=sp_r+2; r_row=sp_r+2+ncol+1;
   for (r=0; r<nrow; ++r) x[r] = 0;
   for (c=0; c<ncol; ++c) {
     for (k=a_colind[pc[c]]; k<a_colind[pc[c]+1]; ++k) x[prinv[a_row[k]]] = nz_a[k];
     for (k=r_colind[c]; k<r_colind[c+1] && (r=r_row[k])<c; ++k) {
       alpha = 0;
       for (k1=v_colind[r]; k1<v_colind[r+1]; ++k1) alpha += nz_v[k1]*x[v_row[k1]];
       alpha *= beta[r];
       for (k1=v_colind[r]; k1<v_colind[r+1]; ++k1) x[v_row[k1]] -= alpha*nz_v[k1];
       *nz_r++ = x[r];
       x[r] = 0;
     }
     for (k=v_colind[c]; k<v_colind[c+1]; ++k) {
       nz_v[k] = x[v_row[k]];
       x[v_row[k]] = 0;
     }
     *nz_r++ = casadi_house(nz_v + v_colind[c], beta + c, v_colind[c+1] - v_colind[c]);
   }
 }
void casadi_qr_mv(const casadi_int* sp_v, const casadi_real* v, const casadi_real* beta, casadi_real* x,
                  casadi_int tr) {
  casadi_int ncol, c, c1, k;
  casadi_real alpha;
  const casadi_int *colind, *row;
  ncol=sp_v[1];
  colind=sp_v+2; row=sp_v+2+ncol+1;
  for (c1=0; c1<ncol; ++c1) {
    c = tr ? c1 : ncol-1-c1;
    alpha=0;
    for (k=colind[c]; k<colind[c+1]; ++k) alpha += v[k]*x[row[k]];
    alpha *= beta[c];
    for (k=colind[c]; k<colind[c+1]; ++k) x[row[k]] -= alpha*v[k];
  }
}
void casadi_qr_trs(const casadi_int* sp_r, const casadi_real* nz_r, casadi_real* x, casadi_int tr) {
  casadi_int ncol, r, c, k;
  const casadi_int *colind, *row;
  ncol=sp_r[1];
  colind=sp_r+2; row=sp_r+2+ncol+1;
  if (tr) {
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        r = row[k];
        if (r==c) {
          x[c] /= nz_r[k];
        } else {
          x[c] -= nz_r[k]*x[r];
        }
      }
    }
  } else {
    for (c=ncol-1; c>=0; --c) {
      for (k=colind[c+1]-1; k>=colind[c]; --k) {
        r=row[k];
        if (r==c) {
          x[r] /= nz_r[k];
        } else {
          x[r] -= nz_r[k]*x[c];
        }
      }
    }
  }
}
void casadi_qr_solve(casadi_real* x, casadi_int nrhs, casadi_int tr,
                     const casadi_int* sp_v, const casadi_real* v, const casadi_int* sp_r, const casadi_real* r,
                     const casadi_real* beta, const casadi_int* prinv, const casadi_int* pc, casadi_real* w) {
  casadi_int k, c, nrow_ext, ncol;
  nrow_ext = sp_v[0]; ncol = sp_v[1];
  for (k=0; k<nrhs; ++k) {
    if (tr) {
      for (c=0; c<ncol; ++c) w[c] = x[pc[c]];
      casadi_qr_trs(sp_r, r, w, 1);
      casadi_qr_mv(sp_v, v, beta, w, 0);
      for (c=0; c<ncol; ++c) x[c] = w[prinv[c]];
    } else {
      for (c=0; c<nrow_ext; ++c) w[c] = 0;
      for (c=0; c<ncol; ++c) w[prinv[c]] = x[c];
      casadi_qr_mv(sp_v, v, beta, w, 1);
      casadi_qr_trs(sp_r, r, w, 0);
      for (c=0; c<ncol; ++c) x[pc[c]] = w[c];
    }
    x += ncol;
  }
}
casadi_int casadi_qr_singular(casadi_real* rmin, casadi_int* irmin, const casadi_real* nz_r,
                             const casadi_int* sp_r, const casadi_int* pc, casadi_real eps) {
  casadi_real rd, rd_min;
  casadi_int ncol, c, nullity;
  const casadi_int* r_colind;
  nullity = 0;
  ncol = sp_r[1];
  r_colind = sp_r + 2;
  for (c=0; c<ncol; ++c) {
    rd = fabs(nz_r[r_colind[c+1]-1]);
    if (rd<eps) nullity++;
    if (c==0 || rd < rd_min) {
      rd_min = rd;
      if (rmin) *rmin = rd;
      if (irmin) *irmin = pc[c];
    }
  }
  return nullity;
}
void casadi_qr_colcomb(casadi_real* v, const casadi_real* nz_r, const casadi_int* sp_r,
                       const casadi_int* pc, casadi_real eps, casadi_int ind) {
  casadi_int ncol, r, c, k;
  const casadi_int *r_colind, *r_row;
  ncol = sp_r[1];
  r_colind = sp_r + 2;
  r_row = r_colind + ncol + 1;
  for (c=0; c<ncol; ++c) {
    if (fabs(nz_r[r_colind[c+1]-1])<eps && 0==ind--) {
      ind = c;
      break;
    }
  }
  casadi_clear(v, ncol);
  v[pc[ind]] = 1.;
  for (k=r_colind[ind]; k<r_colind[ind+1]-1; ++k) {
    v[pc[r_row[k]]] = -nz_r[k];
  }
  for (c=ind-1; c>=0; --c) {
    for (k=r_colind[c+1]-1; k>=r_colind[c]; --k) {
      r=r_row[k];
      if (r==c) {
        if (fabs(nz_r[k])<eps) {
          v[pc[r]] = 0;
        } else {
          v[pc[r]] /= nz_r[k];
        }
      } else {
        v[pc[r]] -= nz_r[k]*v[pc[c]];
      }
    }
  }
  casadi_scal(ncol, 1./sqrt(casadi_dot(ncol, v, v)), v);
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[3] = {0, 1, 2};
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s2[12] = {3, 3, 0, 3, 5, 6, 0, 1, 2, 1, 2, 2};
static const casadi_int casadi_s3[12] = {3, 3, 0, 1, 3, 6, 0, 0, 1, 0, 1, 2};
static const casadi_int casadi_s4[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s5[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s6[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

static const casadi_real casadi_c0[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};

/* quadrotor_expl_vde_adj:(i0[13],i1[13],i2[4],i3[13])->(o0[17]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real *w0=w+4, *w1=w+17, *w2=w+20, w3, *w4=w+30, *w5=w+39, *w6=w+42, *w7=w+45, *w8=w+54, *w9=w+63, *w10=w+76, *w11=w+79, *w12=w+82, *w13=w+86, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, w25, w26, *w27=w+102, *w28=w+106, w29, *w30=w+111, *w31=w+115, *w32=w+119, *w33=w+123, *w34=w+127, w35, w36, *w37=w+145, *w38=w+154, w39, w40, w41, w42, w43, *w44=w+168;
  /* #0: @0 = zeros(13x1) */
  casadi_clear(w0, 13);
  /* #1: @1 = zeros(3x1) */
  casadi_clear(w1, 3);
  /* #2: @2 = zeros(3x3) */
  casadi_clear(w2, 9);
  /* #3: @3 = 0.00264 */
  w3 = 2.6400000000000000e-03;
  /* #4: (@2[0] = @3) */
  for (rr=w2+0, ss=(&w3); rr!=w2+1; rr+=1) *rr = *ss++;
  /* #5: @3 = 0.00264 */
  w3 = 2.6400000000000000e-03;
  /* #6: (@2[4] = @3) */
  for (rr=w2+4, ss=(&w3); rr!=w2+5; rr+=1) *rr = *ss++;
  /* #7: @3 = 0.00496 */
  w3 = 4.9600000000000000e-03;
  /* #8: (@2[8] = @3) */
  for (rr=w2+8, ss=(&w3); rr!=w2+9; rr+=1) *rr = *ss++;
  /* #9: @4 = @2' */
  for (i=0, rr=w4, cs=w2; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #10: @5 = zeros(3x1) */
  casadi_clear(w5, 3);
  /* #11: @3 = input[0][12] */
  w3 = arg[0] ? arg[0][12] : 0;
  /* #12: @6 = zeros(3x1) */
  casadi_clear(w6, 3);
  /* #13: @7 = 
  [[1, 0, 0], 
   [0, 1, 0], 
   [0, 0, 1]] */
  casadi_copy(casadi_c0, 9, w7);
  /* #14: @7 = (@2\@7) */
  rr = w7;
  ss = w2;
  {
    /* FIXME(@jaeandersson): Memory allocation can be avoided */
    casadi_real v[6], r[6], beta[3], w[6];
    casadi_qr(casadi_s1, ss, w, casadi_s2, v, casadi_s3, r, beta, casadi_s0, casadi_s0);
    casadi_qr_solve(rr, 3, 0, casadi_s2, v, casadi_s3, r, beta, casadi_s0, casadi_s0, w);
  }
  /* #15: @8 = @7' */
  for (i=0, rr=w8, cs=w7; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #16: @9 = input[1][0] */
  casadi_copy(arg[1], 13, w9);
  /* #17: {@10, @11, @12, @13} = vertsplit(@9) */
  casadi_copy(w9, 3, w10);
  casadi_copy(w9+3, 3, w11);
  casadi_copy(w9+6, 4, w12);
  casadi_copy(w9+10, 3, w13);
  /* #18: @6 = mac(@8,@13,@6) */
  for (i=0, rr=w6; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w8+j, tt=w13+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #19: @13 = (-@6) */
  for (i=0, rr=w13, cs=w6; i<3; ++i) *rr++ = (- *cs++ );
  /* #20: {@14, @15, @16} = vertsplit(@13) */
  w14 = w13[0];
  w15 = w13[1];
  w16 = w13[2];
  /* #21: @17 = (@3*@15) */
  w17  = (w3*w15);
  /* #22: @18 = input[0][11] */
  w18 = arg[0] ? arg[0][11] : 0;
  /* #23: @19 = (@18*@16) */
  w19  = (w18*w16);
  /* #24: @17 = (@17-@19) */
  w17 -= w19;
  /* #25: (@5[0] += @17) */
  for (rr=w5+0, ss=(&w17); rr!=w5+1; rr+=1) *rr += *ss++;
  /* #26: @17 = input[0][10] */
  w17 = arg[0] ? arg[0][10] : 0;
  /* #27: @19 = (@17*@16) */
  w19  = (w17*w16);
  /* #28: @20 = (@3*@14) */
  w20  = (w3*w14);
  /* #29: @19 = (@19-@20) */
  w19 -= w20;
  /* #30: (@5[1] += @19) */
  for (rr=w5+1, ss=(&w19); rr!=w5+2; rr+=1) *rr += *ss++;
  /* #31: @19 = (@18*@14) */
  w19  = (w18*w14);
  /* #32: @20 = (@17*@15) */
  w20  = (w17*w15);
  /* #33: @19 = (@19-@20) */
  w19 -= w20;
  /* #34: (@5[2] += @19) */
  for (rr=w5+2, ss=(&w19); rr!=w5+3; rr+=1) *rr += *ss++;
  /* #35: @1 = mac(@4,@5,@1) */
  for (i=0, rr=w1; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w4+j, tt=w5+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #36: (@0[10:13] += @1) */
  for (rr=w0+10, ss=w1; rr!=w0+13; rr+=1) *rr += *ss++;
  /* #37: @19 = 10 */
  w19 = 10.;
  /* #38: @20 = 1 */
  w20 = 1.;
  /* #39: @21 = input[0][6] */
  w21 = arg[0] ? arg[0][6] : 0;
  /* #40: @22 = sq(@21) */
  w22 = casadi_sq( w21 );
  /* #41: @23 = input[0][7] */
  w23 = arg[0] ? arg[0][7] : 0;
  /* #42: @24 = sq(@23) */
  w24 = casadi_sq( w23 );
  /* #43: @22 = (@22+@24) */
  w22 += w24;
  /* #44: @24 = input[0][8] */
  w24 = arg[0] ? arg[0][8] : 0;
  /* #45: @25 = sq(@24) */
  w25 = casadi_sq( w24 );
  /* #46: @22 = (@22+@25) */
  w22 += w25;
  /* #47: @25 = input[0][9] */
  w25 = arg[0] ? arg[0][9] : 0;
  /* #48: @26 = sq(@25) */
  w26 = casadi_sq( w25 );
  /* #49: @22 = (@22+@26) */
  w22 += w26;
  /* #50: @20 = (@20-@22) */
  w20 -= w22;
  /* #51: @20 = (@19*@20) */
  w20  = (w19*w20);
  /* #52: @27 = (@20*@12) */
  for (i=0, rr=w27, cs=w12; i<4; ++i) (*rr++)  = (w20*(*cs++));
  /* #53: @28 = zeros(4x1) */
  casadi_clear(w28, 4);
  /* #54: @20 = 0 */
  w20 = 0.;
  /* #55: @22 = (-@17) */
  w22 = (- w17 );
  /* #56: @26 = (-@18) */
  w26 = (- w18 );
  /* #57: @29 = (-@3) */
  w29 = (- w3 );
  /* #58: @30 = horzcat(@20, @22, @26, @29) */
  rr=w30;
  *rr++ = w20;
  *rr++ = w22;
  *rr++ = w26;
  *rr++ = w29;
  /* #59: @30 = @30' */
  /* #60: @20 = 0 */
  w20 = 0.;
  /* #61: @22 = (-@18) */
  w22 = (- w18 );
  /* #62: @31 = horzcat(@17, @20, @3, @22) */
  rr=w31;
  *rr++ = w17;
  *rr++ = w20;
  *rr++ = w3;
  *rr++ = w22;
  /* #63: @31 = @31' */
  /* #64: @20 = (-@3) */
  w20 = (- w3 );
  /* #65: @22 = 0 */
  w22 = 0.;
  /* #66: @32 = horzcat(@18, @20, @22, @17) */
  rr=w32;
  *rr++ = w18;
  *rr++ = w20;
  *rr++ = w22;
  *rr++ = w17;
  /* #67: @32 = @32' */
  /* #68: @20 = (-@17) */
  w20 = (- w17 );
  /* #69: @22 = 0 */
  w22 = 0.;
  /* #70: @33 = horzcat(@3, @18, @20, @22) */
  rr=w33;
  *rr++ = w3;
  *rr++ = w18;
  *rr++ = w20;
  *rr++ = w22;
  /* #71: @33 = @33' */
  /* #72: @34 = horzcat(@30, @31, @32, @33) */
  rr=w34;
  for (i=0, cs=w30; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w31; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w32; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w33; i<4; ++i) *rr++ = *cs++;
  /* #73: @20 = 0.5 */
  w20 = 5.0000000000000000e-01;
  /* #74: @30 = (@20*@12) */
  for (i=0, rr=w30, cs=w12; i<4; ++i) (*rr++)  = (w20*(*cs++));
  /* #75: @28 = mac(@34,@30,@28) */
  for (i=0, rr=w28; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w34+j, tt=w30+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #76: @27 = (@27+@28) */
  for (i=0, rr=w27, cs=w28; i<4; ++i) (*rr++) += (*cs++);
  /* #77: @28 = zeros(4x1) */
  casadi_clear(w28, 4);
  /* #78: @4 = zeros(3x3) */
  casadi_clear(w4, 9);
  /* #79: @20 = input[0][0] */
  w20 = arg[0] ? arg[0][0] : 0;
  /* #80: @22 = input[0][1] */
  w22 = arg[0] ? arg[0][1] : 0;
  /* #81: @26 = input[0][2] */
  w26 = arg[0] ? arg[0][2] : 0;
  /* #82: @29 = input[0][3] */
  w29 = arg[0] ? arg[0][3] : 0;
  /* #83: @35 = input[0][4] */
  w35 = arg[0] ? arg[0][4] : 0;
  /* #84: @36 = input[0][5] */
  w36 = arg[0] ? arg[0][5] : 0;
  /* #85: @9 = vertcat(@20, @22, @26, @29, @35, @36, @21, @23, @24, @25, @17, @18, @3) */
  rr=w9;
  *rr++ = w20;
  *rr++ = w22;
  *rr++ = w26;
  *rr++ = w29;
  *rr++ = w35;
  *rr++ = w36;
  *rr++ = w21;
  *rr++ = w23;
  *rr++ = w24;
  *rr++ = w25;
  *rr++ = w17;
  *rr++ = w18;
  *rr++ = w3;
  /* #86: @31 = @9[6:10] */
  for (rr=w31, ss=w9+6; ss!=w9+10; ss+=1) *rr++ = *ss;
  /* #87: @20 = ||@31||_F */
  w20 = sqrt(casadi_dot(4, w31, w31));
  /* #88: @32 = (@31/@20) */
  for (i=0, rr=w32, cr=w31; i<4; ++i) (*rr++)  = ((*cr++)/w20);
  /* #89: @22 = @32[3] */
  for (rr=(&w22), ss=w32+3; ss!=w32+4; ss+=1) *rr++ = *ss;
  /* #90: @22 = (-@22) */
  w22 = (- w22 );
  /* #91: (@4[3] = @22) */
  for (rr=w4+3, ss=(&w22); rr!=w4+4; rr+=1) *rr = *ss++;
  /* #92: @22 = @32[2] */
  for (rr=(&w22), ss=w32+2; ss!=w32+3; ss+=1) *rr++ = *ss;
  /* #93: (@4[6] = @22) */
  for (rr=w4+6, ss=(&w22); rr!=w4+7; rr+=1) *rr = *ss++;
  /* #94: @22 = @32[1] */
  for (rr=(&w22), ss=w32+1; ss!=w32+2; ss+=1) *rr++ = *ss;
  /* #95: @22 = (-@22) */
  w22 = (- w22 );
  /* #96: (@4[7] = @22) */
  for (rr=w4+7, ss=(&w22); rr!=w4+8; rr+=1) *rr = *ss++;
  /* #97: @22 = @32[3] */
  for (rr=(&w22), ss=w32+3; ss!=w32+4; ss+=1) *rr++ = *ss;
  /* #98: (@4[1] = @22) */
  for (rr=w4+1, ss=(&w22); rr!=w4+2; rr+=1) *rr = *ss++;
  /* #99: @22 = @32[2] */
  for (rr=(&w22), ss=w32+2; ss!=w32+3; ss+=1) *rr++ = *ss;
  /* #100: @22 = (-@22) */
  w22 = (- w22 );
  /* #101: (@4[2] = @22) */
  for (rr=w4+2, ss=(&w22); rr!=w4+3; rr+=1) *rr = *ss++;
  /* #102: @22 = @32[1] */
  for (rr=(&w22), ss=w32+1; ss!=w32+2; ss+=1) *rr++ = *ss;
  /* #103: (@4[5] = @22) */
  for (rr=w4+5, ss=(&w22); rr!=w4+6; rr+=1) *rr = *ss++;
  /* #104: @8 = zeros(3x3) */
  casadi_clear(w8, 9);
  /* #105: @22 = input[2][0] */
  w22 = arg[2] ? arg[2][0] : 0;
  /* #106: @1 = (@22*@11) */
  for (i=0, rr=w1, cs=w11; i<3; ++i) (*rr++)  = (w22*(*cs++));
  /* #107: @5 = zeros(3x1) */
  casadi_clear(w5, 3);
  /* #108: @22 = 1 */
  w22 = 1.;
  /* #109: (@5[2] = @22) */
  for (rr=w5+2, ss=(&w22); rr!=w5+3; rr+=1) *rr = *ss++;
  /* #110: @13 = @5' */
  casadi_copy(w5, 3, w13);
  /* #111: @8 = mac(@1,@13,@8) */
  for (i=0, rr=w8; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w1+j, tt=w13+i*1; k<1; ++k) *rr += ss[k*3]**tt++;
  /* #112: @22 = dot(@4, @8) */
  w22 = casadi_dot(9, w4, w8);
  /* #113: @22 = (2.*@22) */
  w22 = (2.* w22 );
  /* #114: (@28[0] += @22) */
  for (rr=w28+0, ss=(&w22); rr!=w28+1; rr+=1) *rr += *ss++;
  /* #115: @22 = @32[0] */
  for (rr=(&w22), ss=w32+0; ss!=w32+1; ss+=1) *rr++ = *ss;
  /* #116: @22 = (2.*@22) */
  w22 = (2.* w22 );
  /* #117: @7 = (@22*@8) */
  for (i=0, rr=w7, cs=w8; i<9; ++i) (*rr++)  = (w22*(*cs++));
  /* #118: @37 = zeros(3x3) */
  casadi_clear(w37, 9);
  /* #119: @8 = (2.*@8) */
  for (i=0, rr=w8, cs=w8; i<9; ++i) *rr++ = (2.* *cs++ );
  /* #120: @38 = @4' */
  for (i=0, rr=w38, cs=w4; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #121: @37 = mac(@8,@38,@37) */
  for (i=0, rr=w37; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w8+j, tt=w38+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #122: @7 = (@7+@37) */
  for (i=0, rr=w7, cs=w37; i<9; ++i) (*rr++) += (*cs++);
  /* #123: @37 = zeros(3x3) */
  casadi_clear(w37, 9);
  /* #124: @38 = @4' */
  for (i=0, rr=w38, cs=w4; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #125: @37 = mac(@38,@8,@37) */
  for (i=0, rr=w37; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w38+j, tt=w8+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #126: @7 = (@7+@37) */
  for (i=0, rr=w7, cs=w37; i<9; ++i) (*rr++) += (*cs++);
  /* #127: @26 = @7[5] */
  for (rr=(&w26), ss=w7+5; ss!=w7+6; ss+=1) *rr++ = *ss;
  /* #128: (@28[1] += @26) */
  for (rr=w28+1, ss=(&w26); rr!=w28+2; rr+=1) *rr += *ss++;
  /* #129: @26 = 0 */
  w26 = 0.;
  /* #130: (@7[5] = @26) */
  for (rr=w7+5, ss=(&w26); rr!=w7+6; rr+=1) *rr = *ss++;
  /* #131: @26 = @7[2] */
  for (rr=(&w26), ss=w7+2; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #132: @26 = (-@26) */
  w26 = (- w26 );
  /* #133: (@28[2] += @26) */
  for (rr=w28+2, ss=(&w26); rr!=w28+3; rr+=1) *rr += *ss++;
  /* #134: @26 = 0 */
  w26 = 0.;
  /* #135: (@7[2] = @26) */
  for (rr=w7+2, ss=(&w26); rr!=w7+3; rr+=1) *rr = *ss++;
  /* #136: @26 = @7[1] */
  for (rr=(&w26), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #137: (@28[3] += @26) */
  for (rr=w28+3, ss=(&w26); rr!=w28+4; rr+=1) *rr += *ss++;
  /* #138: @26 = 0 */
  w26 = 0.;
  /* #139: (@7[1] = @26) */
  for (rr=w7+1, ss=(&w26); rr!=w7+2; rr+=1) *rr = *ss++;
  /* #140: @26 = @7[7] */
  for (rr=(&w26), ss=w7+7; ss!=w7+8; ss+=1) *rr++ = *ss;
  /* #141: @26 = (-@26) */
  w26 = (- w26 );
  /* #142: (@28[1] += @26) */
  for (rr=w28+1, ss=(&w26); rr!=w28+2; rr+=1) *rr += *ss++;
  /* #143: @26 = 0 */
  w26 = 0.;
  /* #144: (@7[7] = @26) */
  for (rr=w7+7, ss=(&w26); rr!=w7+8; rr+=1) *rr = *ss++;
  /* #145: @26 = @7[6] */
  for (rr=(&w26), ss=w7+6; ss!=w7+7; ss+=1) *rr++ = *ss;
  /* #146: (@28[2] += @26) */
  for (rr=w28+2, ss=(&w26); rr!=w28+3; rr+=1) *rr += *ss++;
  /* #147: @26 = 0 */
  w26 = 0.;
  /* #148: (@7[6] = @26) */
  for (rr=w7+6, ss=(&w26); rr!=w7+7; rr+=1) *rr = *ss++;
  /* #149: @26 = @7[3] */
  for (rr=(&w26), ss=w7+3; ss!=w7+4; ss+=1) *rr++ = *ss;
  /* #150: @26 = (-@26) */
  w26 = (- w26 );
  /* #151: (@28[3] += @26) */
  for (rr=w28+3, ss=(&w26); rr!=w28+4; rr+=1) *rr += *ss++;
  /* #152: @33 = (@28/@20) */
  for (i=0, rr=w33, cr=w28; i<4; ++i) (*rr++)  = ((*cr++)/w20);
  /* #153: @27 = (@27+@33) */
  for (i=0, rr=w27, cs=w33; i<4; ++i) (*rr++) += (*cs++);
  /* #154: @32 = (@32/@20) */
  for (i=0, rr=w32; i<4; ++i) (*rr++) /= w20;
  /* #155: @32 = (-@32) */
  for (i=0, rr=w32, cs=w32; i<4; ++i) *rr++ = (- *cs++ );
  /* #156: @26 = dot(@32, @28) */
  w26 = casadi_dot(4, w32, w28);
  /* #157: @26 = (@26/@20) */
  w26 /= w20;
  /* #158: @32 = (@26*@31) */
  for (i=0, rr=w32, cs=w31; i<4; ++i) (*rr++)  = (w26*(*cs++));
  /* #159: @27 = (@27+@32) */
  for (i=0, rr=w27, cs=w32; i<4; ++i) (*rr++) += (*cs++);
  /* #160: (@0[6:10] += @27) */
  for (rr=w0+6, ss=w27; rr!=w0+10; rr+=1) *rr += *ss++;
  /* #161: (@0[3:6] += @10) */
  for (rr=w0+3, ss=w10; rr!=w0+6; rr+=1) *rr += *ss++;
  /* #162: {@26, @20, @29, @35, @36, @17, @18, @3, @39, @40, @41, @42, @43} = vertsplit(@0) */
  w26 = w0[0];
  w20 = w0[1];
  w29 = w0[2];
  w35 = w0[3];
  w36 = w0[4];
  w17 = w0[5];
  w18 = w0[6];
  w3 = w0[7];
  w39 = w0[8];
  w40 = w0[9];
  w41 = w0[10];
  w42 = w0[11];
  w43 = w0[12];
  /* #163: output[0][0] = @26 */
  if (res[0]) res[0][0] = w26;
  /* #164: output[0][1] = @20 */
  if (res[0]) res[0][1] = w20;
  /* #165: output[0][2] = @29 */
  if (res[0]) res[0][2] = w29;
  /* #166: output[0][3] = @35 */
  if (res[0]) res[0][3] = w35;
  /* #167: output[0][4] = @36 */
  if (res[0]) res[0][4] = w36;
  /* #168: output[0][5] = @17 */
  if (res[0]) res[0][5] = w17;
  /* #169: @21 = (2.*@21) */
  w21 = (2.* w21 );
  /* #170: @17 = dot(@31, @12) */
  w17 = casadi_dot(4, w31, w12);
  /* #171: @19 = (@19*@17) */
  w19 *= w17;
  /* #172: @21 = (@21*@19) */
  w21 *= w19;
  /* #173: @18 = (@18-@21) */
  w18 -= w21;
  /* #174: output[0][6] = @18 */
  if (res[0]) res[0][6] = w18;
  /* #175: @23 = (2.*@23) */
  w23 = (2.* w23 );
  /* #176: @23 = (@23*@19) */
  w23 *= w19;
  /* #177: @3 = (@3-@23) */
  w3 -= w23;
  /* #178: output[0][7] = @3 */
  if (res[0]) res[0][7] = w3;
  /* #179: @24 = (2.*@24) */
  w24 = (2.* w24 );
  /* #180: @24 = (@24*@19) */
  w24 *= w19;
  /* #181: @39 = (@39-@24) */
  w39 -= w24;
  /* #182: output[0][8] = @39 */
  if (res[0]) res[0][8] = w39;
  /* #183: @25 = (2.*@25) */
  w25 = (2.* w25 );
  /* #184: @25 = (@25*@19) */
  w25 *= w19;
  /* #185: @40 = (@40-@25) */
  w40 -= w25;
  /* #186: output[0][9] = @40 */
  if (res[0]) res[0][9] = w40;
  /* #187: @10 = zeros(3x1) */
  casadi_clear(w10, 3);
  /* #188: @1 = @9[10:13] */
  for (rr=w1, ss=w9+10; ss!=w9+13; ss+=1) *rr++ = *ss;
  /* #189: @10 = mac(@2,@1,@10) */
  for (i=0, rr=w10; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w2+j, tt=w1+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #190: @40 = @10[1] */
  for (rr=(&w40), ss=w10+1; ss!=w10+2; ss+=1) *rr++ = *ss;
  /* #191: @25 = (@40*@16) */
  w25  = (w40*w16);
  /* #192: @19 = @10[2] */
  for (rr=(&w19), ss=w10+2; ss!=w10+3; ss+=1) *rr++ = *ss;
  /* #193: @39 = (@19*@15) */
  w39  = (w19*w15);
  /* #194: @25 = (@25-@39) */
  w25 -= w39;
  /* #195: @34 = zeros(4x4) */
  casadi_clear(w34, 16);
  /* #196: @31 = @31' */
  /* #197: @34 = mac(@30,@31,@34) */
  for (i=0, rr=w34; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w30+j, tt=w31+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #198: @44 = @34' */
  for (i=0, rr=w44, cs=w34; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #199: {@30, @31, @12, @27} = horzsplit(@44) */
  casadi_copy(w44, 4, w30);
  casadi_copy(w44+4, 4, w31);
  casadi_copy(w44+8, 4, w12);
  casadi_copy(w44+12, 4, w27);
  /* #200: @27 = @27' */
  /* #201: {@39, @24, @3, NULL} = horzsplit(@27) */
  w39 = w27[0];
  w24 = w27[1];
  w3 = w27[2];
  /* #202: @25 = (@25-@3) */
  w25 -= w3;
  /* #203: @12 = @12' */
  /* #204: {@3, @23, NULL, @18} = horzsplit(@12) */
  w3 = w12[0];
  w23 = w12[1];
  w18 = w12[3];
  /* #205: @25 = (@25+@18) */
  w25 += w18;
  /* #206: @31 = @31' */
  /* #207: {@18, NULL, @21, @17} = horzsplit(@31) */
  w18 = w31[0];
  w21 = w31[2];
  w17 = w31[3];
  /* #208: @25 = (@25+@18) */
  w25 += w18;
  /* #209: @30 = @30' */
  /* #210: {NULL, @18, @36, @35} = horzsplit(@30) */
  w18 = w30[1];
  w36 = w30[2];
  w35 = w30[3];
  /* #211: @25 = (@25-@18) */
  w25 -= w18;
  /* #212: @25 = (@25+@41) */
  w25 += w41;
  /* #213: output[0][10] = @25 */
  if (res[0]) res[0][10] = w25;
  /* #214: @19 = (@19*@14) */
  w19 *= w14;
  /* #215: @25 = @10[0] */
  for (rr=(&w25), ss=w10+0; ss!=w10+1; ss+=1) *rr++ = *ss;
  /* #216: @16 = (@25*@16) */
  w16  = (w25*w16);
  /* #217: @19 = (@19-@16) */
  w19 -= w16;
  /* #218: @19 = (@19+@24) */
  w19 += w24;
  /* #219: @19 = (@19+@3) */
  w19 += w3;
  /* #220: @19 = (@19-@17) */
  w19 -= w17;
  /* #221: @19 = (@19-@36) */
  w19 -= w36;
  /* #222: @19 = (@19+@42) */
  w19 += w42;
  /* #223: output[0][11] = @19 */
  if (res[0]) res[0][11] = w19;
  /* #224: @25 = (@25*@15) */
  w25 *= w15;
  /* #225: @40 = (@40*@14) */
  w40 *= w14;
  /* #226: @25 = (@25-@40) */
  w25 -= w40;
  /* #227: @25 = (@25+@39) */
  w25 += w39;
  /* #228: @25 = (@25-@23) */
  w25 -= w23;
  /* #229: @25 = (@25+@21) */
  w25 += w21;
  /* #230: @25 = (@25-@35) */
  w25 -= w35;
  /* #231: @25 = (@25+@43) */
  w25 += w43;
  /* #232: output[0][12] = @25 */
  if (res[0]) res[0][12] = w25;
  /* #233: @30 = zeros(4x1) */
  casadi_clear(w30, 4);
  /* #234: (@30[1:4] += @6) */
  for (rr=w30+1, ss=w6; rr!=w30+4; rr+=1) *rr += *ss++;
  /* #235: {@25, @43, @35, @21} = vertsplit(@30) */
  w25 = w30[0];
  w43 = w30[1];
  w35 = w30[2];
  w21 = w30[3];
  /* #236: @6 = zeros(3x1) */
  casadi_clear(w6, 3);
  /* #237: @2 = 
  [[1, 0, 0], 
   [0, 1, 0], 
   [0, 0, 1]] */
  casadi_copy(casadi_c0, 9, w2);
  /* #238: @7 = zeros(3x3) */
  casadi_clear(w7, 9);
  /* #239: @7 = mac(@4,@4,@7) */
  for (i=0, rr=w7; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w4+j, tt=w4+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #240: @7 = (2.*@7) */
  for (i=0, rr=w7, cs=w7; i<9; ++i) *rr++ = (2.* *cs++ );
  /* #241: @2 = (@2+@7) */
  for (i=0, rr=w2, cs=w7; i<9; ++i) (*rr++) += (*cs++);
  /* #242: @4 = (@22*@4) */
  for (i=0, rr=w4, cs=w4; i<9; ++i) (*rr++)  = (w22*(*cs++));
  /* #243: @2 = (@2+@4) */
  for (i=0, rr=w2, cs=w4; i<9; ++i) (*rr++) += (*cs++);
  /* #244: @6 = mac(@2,@5,@6) */
  for (i=0, rr=w6; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w2+j, tt=w5+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #245: @22 = dot(@6, @11) */
  w22 = casadi_dot(3, w6, w11);
  /* #246: @25 = (@25+@22) */
  w25 += w22;
  /* #247: output[0][13] = @25 */
  if (res[0]) res[0][13] = w25;
  /* #248: output[0][14] = @43 */
  if (res[0]) res[0][14] = w43;
  /* #249: output[0][15] = @35 */
  if (res[0]) res[0][15] = w35;
  /* #250: output[0][16] = @21 */
  if (res[0]) res[0][16] = w21;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_expl_vde_adj_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_expl_vde_adj_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_expl_vde_adj_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    case 3: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 14;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 184;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
