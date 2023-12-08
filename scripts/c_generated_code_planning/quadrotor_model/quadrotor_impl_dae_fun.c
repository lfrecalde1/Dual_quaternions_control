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
  #define CASADI_PREFIX(ID) quadrotor_impl_dae_fun_ ## ID
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

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_if_else(casadi_real c, casadi_real x, casadi_real y) { return c!=0 ? x : y;}

void casadi_scal(casadi_int n, casadi_real alpha, casadi_real* x) {
  casadi_int i;
  if (!x) return;
  for (i=0; i<n; ++i) *x++ *= alpha;
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

static const casadi_int casadi_s0[3] = {0, 1, 2};
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s2[12] = {3, 3, 0, 3, 5, 6, 0, 1, 2, 1, 2, 2};
static const casadi_int casadi_s3[12] = {3, 3, 0, 1, 3, 6, 0, 0, 1, 0, 1, 2};
static const casadi_int casadi_s4[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s5[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s6[3] = {0, 0, 0};

static const casadi_real casadi_c0[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};

/* quadrotor_impl_dae_fun:(i0[13],i1[13],i2[4],i3[],i4[13])->(o0[13]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, *w13=w+17, *w14=w+30, *w15=w+43, *w16=w+46, *w17=w+49, *w18=w+58, *w19=w+67, *w20=w+76, *w21=w+80, *w22=w+84, *w23=w+87, *w24=w+91, *w25=w+95, *w26=w+99, *w27=w+103, *w28=w+119, *w29=w+135, *w30=w+138, *w31=w+141;
  /* #0: @0 = input[1][0] */
  w0 = arg[1] ? arg[1][0] : 0;
  /* #1: @1 = input[1][1] */
  w1 = arg[1] ? arg[1][1] : 0;
  /* #2: @2 = input[1][2] */
  w2 = arg[1] ? arg[1][2] : 0;
  /* #3: @3 = input[1][3] */
  w3 = arg[1] ? arg[1][3] : 0;
  /* #4: @4 = input[1][4] */
  w4 = arg[1] ? arg[1][4] : 0;
  /* #5: @5 = input[1][5] */
  w5 = arg[1] ? arg[1][5] : 0;
  /* #6: @6 = input[1][6] */
  w6 = arg[1] ? arg[1][6] : 0;
  /* #7: @7 = input[1][7] */
  w7 = arg[1] ? arg[1][7] : 0;
  /* #8: @8 = input[1][8] */
  w8 = arg[1] ? arg[1][8] : 0;
  /* #9: @9 = input[1][9] */
  w9 = arg[1] ? arg[1][9] : 0;
  /* #10: @10 = input[1][10] */
  w10 = arg[1] ? arg[1][10] : 0;
  /* #11: @11 = input[1][11] */
  w11 = arg[1] ? arg[1][11] : 0;
  /* #12: @12 = input[1][12] */
  w12 = arg[1] ? arg[1][12] : 0;
  /* #13: @13 = vertcat(@0, @1, @2, @3, @4, @5, @6, @7, @8, @9, @10, @11, @12) */
  rr=w13;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  /* #14: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #15: @1 = input[0][1] */
  w1 = arg[0] ? arg[0][1] : 0;
  /* #16: @2 = input[0][2] */
  w2 = arg[0] ? arg[0][2] : 0;
  /* #17: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #18: @4 = input[0][4] */
  w4 = arg[0] ? arg[0][4] : 0;
  /* #19: @5 = input[0][5] */
  w5 = arg[0] ? arg[0][5] : 0;
  /* #20: @6 = input[0][6] */
  w6 = arg[0] ? arg[0][6] : 0;
  /* #21: @7 = input[0][7] */
  w7 = arg[0] ? arg[0][7] : 0;
  /* #22: @8 = input[0][8] */
  w8 = arg[0] ? arg[0][8] : 0;
  /* #23: @9 = input[0][9] */
  w9 = arg[0] ? arg[0][9] : 0;
  /* #24: @10 = input[0][10] */
  w10 = arg[0] ? arg[0][10] : 0;
  /* #25: @11 = input[0][11] */
  w11 = arg[0] ? arg[0][11] : 0;
  /* #26: @12 = input[0][12] */
  w12 = arg[0] ? arg[0][12] : 0;
  /* #27: @14 = vertcat(@0, @1, @2, @3, @4, @5, @6, @7, @8, @9, @10, @11, @12) */
  rr=w14;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  /* #28: @15 = @14[3:6] */
  for (rr=w15, ss=w14+3; ss!=w14+6; ss+=1) *rr++ = *ss;
  /* #29: @0 = input[2][0] */
  w0 = arg[2] ? arg[2][0] : 0;
  /* #30: @16 = zeros(3x1) */
  casadi_clear(w16, 3);
  /* #31: @17 = 
  [[1, 0, 0], 
   [0, 1, 0], 
   [0, 0, 1]] */
  casadi_copy(casadi_c0, 9, w17);
  /* #32: @18 = zeros(3x3) */
  casadi_clear(w18, 9);
  /* #33: @19 = zeros(3x3) */
  casadi_clear(w19, 9);
  /* #34: @20 = @14[6:10] */
  for (rr=w20, ss=w14+6; ss!=w14+10; ss+=1) *rr++ = *ss;
  /* #35: @1 = ||@20||_F */
  w1 = sqrt(casadi_dot(4, w20, w20));
  /* #36: @21 = (@20/@1) */
  for (i=0, rr=w21, cr=w20; i<4; ++i) (*rr++)  = ((*cr++)/w1);
  /* #37: @1 = @21[3] */
  for (rr=(&w1), ss=w21+3; ss!=w21+4; ss+=1) *rr++ = *ss;
  /* #38: @1 = (-@1) */
  w1 = (- w1 );
  /* #39: (@19[3] = @1) */
  for (rr=w19+3, ss=(&w1); rr!=w19+4; rr+=1) *rr = *ss++;
  /* #40: @1 = @21[2] */
  for (rr=(&w1), ss=w21+2; ss!=w21+3; ss+=1) *rr++ = *ss;
  /* #41: (@19[6] = @1) */
  for (rr=w19+6, ss=(&w1); rr!=w19+7; rr+=1) *rr = *ss++;
  /* #42: @1 = @21[1] */
  for (rr=(&w1), ss=w21+1; ss!=w21+2; ss+=1) *rr++ = *ss;
  /* #43: @1 = (-@1) */
  w1 = (- w1 );
  /* #44: (@19[7] = @1) */
  for (rr=w19+7, ss=(&w1); rr!=w19+8; rr+=1) *rr = *ss++;
  /* #45: @1 = @21[3] */
  for (rr=(&w1), ss=w21+3; ss!=w21+4; ss+=1) *rr++ = *ss;
  /* #46: (@19[1] = @1) */
  for (rr=w19+1, ss=(&w1); rr!=w19+2; rr+=1) *rr = *ss++;
  /* #47: @1 = @21[2] */
  for (rr=(&w1), ss=w21+2; ss!=w21+3; ss+=1) *rr++ = *ss;
  /* #48: @1 = (-@1) */
  w1 = (- w1 );
  /* #49: (@19[2] = @1) */
  for (rr=w19+2, ss=(&w1); rr!=w19+3; rr+=1) *rr = *ss++;
  /* #50: @1 = @21[1] */
  for (rr=(&w1), ss=w21+1; ss!=w21+2; ss+=1) *rr++ = *ss;
  /* #51: (@19[5] = @1) */
  for (rr=w19+5, ss=(&w1); rr!=w19+6; rr+=1) *rr = *ss++;
  /* #52: @18 = mac(@19,@19,@18) */
  for (i=0, rr=w18; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w19+j, tt=w19+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #53: @18 = (2.*@18) */
  for (i=0, rr=w18, cs=w18; i<9; ++i) *rr++ = (2.* *cs++ );
  /* #54: @17 = (@17+@18) */
  for (i=0, rr=w17, cs=w18; i<9; ++i) (*rr++) += (*cs++);
  /* #55: @1 = @21[0] */
  for (rr=(&w1), ss=w21+0; ss!=w21+1; ss+=1) *rr++ = *ss;
  /* #56: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #57: @19 = (@1*@19) */
  for (i=0, rr=w19, cs=w19; i<9; ++i) (*rr++)  = (w1*(*cs++));
  /* #58: @17 = (@17+@19) */
  for (i=0, rr=w17, cs=w19; i<9; ++i) (*rr++) += (*cs++);
  /* #59: @22 = zeros(3x1) */
  casadi_clear(w22, 3);
  /* #60: @1 = 1 */
  w1 = 1.;
  /* #61: (@22[2] = @1) */
  for (rr=w22+2, ss=(&w1); rr!=w22+3; rr+=1) *rr = *ss++;
  /* #62: @16 = mac(@17,@22,@16) */
  for (i=0, rr=w16; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w17+j, tt=w22+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #63: @16 = (@0*@16) */
  for (i=0, rr=w16, cs=w16; i<3; ++i) (*rr++)  = (w0*(*cs++));
  /* #64: @1 = 9.8 */
  w1 = 9.8000000000000007e+00;
  /* #65: @22 = (@1*@22) */
  for (i=0, rr=w22, cs=w22; i<3; ++i) (*rr++)  = (w1*(*cs++));
  /* #66: @16 = (@16-@22) */
  for (i=0, rr=w16, cs=w22; i<3; ++i) (*rr++) -= (*cs++);
  /* #67: @1 = 0.5 */
  w1 = 5.0000000000000000e-01;
  /* #68: @21 = zeros(4x1) */
  casadi_clear(w21, 4);
  /* #69: @2 = 0 */
  w2 = 0.;
  /* #70: @3 = (-@10) */
  w3 = (- w10 );
  /* #71: @4 = (-@11) */
  w4 = (- w11 );
  /* #72: @5 = (-@12) */
  w5 = (- w12 );
  /* #73: @23 = horzcat(@2, @3, @4, @5) */
  rr=w23;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  /* #74: @23 = @23' */
  /* #75: @2 = 0 */
  w2 = 0.;
  /* #76: @3 = (-@11) */
  w3 = (- w11 );
  /* #77: @24 = horzcat(@10, @2, @12, @3) */
  rr=w24;
  *rr++ = w10;
  *rr++ = w2;
  *rr++ = w12;
  *rr++ = w3;
  /* #78: @24 = @24' */
  /* #79: @2 = (-@12) */
  w2 = (- w12 );
  /* #80: @3 = 0 */
  w3 = 0.;
  /* #81: @25 = horzcat(@11, @2, @3, @10) */
  rr=w25;
  *rr++ = w11;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w10;
  /* #82: @25 = @25' */
  /* #83: @2 = (-@10) */
  w2 = (- w10 );
  /* #84: @3 = 0 */
  w3 = 0.;
  /* #85: @26 = horzcat(@12, @11, @2, @3) */
  rr=w26;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w2;
  *rr++ = w3;
  /* #86: @26 = @26' */
  /* #87: @27 = horzcat(@23, @24, @25, @26) */
  rr=w27;
  for (i=0, cs=w23; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w24; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w25; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  /* #88: @28 = @27' */
  for (i=0, rr=w28, cs=w27; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #89: @21 = mac(@28,@20,@21) */
  for (i=0, rr=w21; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w28+j, tt=w20+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #90: @21 = (@1*@21) */
  for (i=0, rr=w21, cs=w21; i<4; ++i) (*rr++)  = (w1*(*cs++));
  /* #91: @1 = 10 */
  w1 = 10.;
  /* #92: @2 = 1 */
  w2 = 1.;
  /* #93: @6 = sq(@6) */
  w6 = casadi_sq( w6 );
  /* #94: @7 = sq(@7) */
  w7 = casadi_sq( w7 );
  /* #95: @6 = (@6+@7) */
  w6 += w7;
  /* #96: @8 = sq(@8) */
  w8 = casadi_sq( w8 );
  /* #97: @6 = (@6+@8) */
  w6 += w8;
  /* #98: @9 = sq(@9) */
  w9 = casadi_sq( w9 );
  /* #99: @6 = (@6+@9) */
  w6 += w9;
  /* #100: @2 = (@2-@6) */
  w2 -= w6;
  /* #101: @1 = (@1*@2) */
  w1 *= w2;
  /* #102: @20 = (@1*@20) */
  for (i=0, rr=w20, cs=w20; i<4; ++i) (*rr++)  = (w1*(*cs++));
  /* #103: @21 = (@21+@20) */
  for (i=0, rr=w21, cs=w20; i<4; ++i) (*rr++) += (*cs++);
  /* #104: @22 = zeros(3x1) */
  casadi_clear(w22, 3);
  /* #105: @17 = 
  [[1, 0, 0], 
   [0, 1, 0], 
   [0, 0, 1]] */
  casadi_copy(casadi_c0, 9, w17);
  /* #106: @19 = zeros(3x3) */
  casadi_clear(w19, 9);
  /* #107: @1 = 0.00264 */
  w1 = 2.6400000000000000e-03;
  /* #108: (@19[0] = @1) */
  for (rr=w19+0, ss=(&w1); rr!=w19+1; rr+=1) *rr = *ss++;
  /* #109: @1 = 0.00264 */
  w1 = 2.6400000000000000e-03;
  /* #110: (@19[4] = @1) */
  for (rr=w19+4, ss=(&w1); rr!=w19+5; rr+=1) *rr = *ss++;
  /* #111: @1 = 0.00496 */
  w1 = 4.9600000000000000e-03;
  /* #112: (@19[8] = @1) */
  for (rr=w19+8, ss=(&w1); rr!=w19+9; rr+=1) *rr = *ss++;
  /* #113: @17 = (@19\@17) */
  rr = w17;
  ss = w19;
  {
    /* FIXME(@jaeandersson): Memory allocation can be avoided */
    casadi_real v[6], r[6], beta[3], w[6];
    casadi_qr(casadi_s1, ss, w, casadi_s2, v, casadi_s3, r, beta, casadi_s0, casadi_s0);
    casadi_qr_solve(rr, 3, 0, casadi_s2, v, casadi_s3, r, beta, casadi_s0, casadi_s0, w);
  }
  /* #114: @1 = input[2][1] */
  w1 = arg[2] ? arg[2][1] : 0;
  /* #115: @2 = input[2][2] */
  w2 = arg[2] ? arg[2][2] : 0;
  /* #116: @6 = input[2][3] */
  w6 = arg[2] ? arg[2][3] : 0;
  /* #117: @20 = vertcat(@0, @1, @2, @6) */
  rr=w20;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w6;
  /* #118: @29 = @20[1:4] */
  for (rr=w29, ss=w20+1; ss!=w20+4; ss+=1) *rr++ = *ss;
  /* #119: @30 = zeros(3x1) */
  casadi_clear(w30, 3);
  /* #120: @31 = @14[10:13] */
  for (rr=w31, ss=w14+10; ss!=w14+13; ss+=1) *rr++ = *ss;
  /* #121: @30 = mac(@19,@31,@30) */
  for (i=0, rr=w30; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w19+j, tt=w31+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #122: @0 = @30[2] */
  for (rr=(&w0), ss=w30+2; ss!=w30+3; ss+=1) *rr++ = *ss;
  /* #123: @1 = (@11*@0) */
  w1  = (w11*w0);
  /* #124: @2 = @30[1] */
  for (rr=(&w2), ss=w30+1; ss!=w30+2; ss+=1) *rr++ = *ss;
  /* #125: @6 = (@12*@2) */
  w6  = (w12*w2);
  /* #126: @1 = (@1-@6) */
  w1 -= w6;
  /* #127: @6 = @30[0] */
  for (rr=(&w6), ss=w30+0; ss!=w30+1; ss+=1) *rr++ = *ss;
  /* #128: @12 = (@12*@6) */
  w12 *= w6;
  /* #129: @0 = (@10*@0) */
  w0  = (w10*w0);
  /* #130: @12 = (@12-@0) */
  w12 -= w0;
  /* #131: @10 = (@10*@2) */
  w10 *= w2;
  /* #132: @11 = (@11*@6) */
  w11 *= w6;
  /* #133: @10 = (@10-@11) */
  w10 -= w11;
  /* #134: @30 = vertcat(@1, @12, @10) */
  rr=w30;
  *rr++ = w1;
  *rr++ = w12;
  *rr++ = w10;
  /* #135: @29 = (@29-@30) */
  for (i=0, rr=w29, cs=w30; i<3; ++i) (*rr++) -= (*cs++);
  /* #136: @22 = mac(@17,@29,@22) */
  for (i=0, rr=w22; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w17+j, tt=w29+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #137: @14 = vertcat(@15, @16, @21, @22) */
  rr=w14;
  for (i=0, cs=w15; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w21; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<3; ++i) *rr++ = *cs++;
  /* #138: @13 = (@13-@14) */
  for (i=0, rr=w13, cs=w14; i<13; ++i) (*rr++) -= (*cs++);
  /* #139: output[0][0] = @13 */
  casadi_copy(w13, 13, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_impl_dae_fun_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_impl_dae_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_impl_dae_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_impl_dae_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_impl_dae_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_impl_dae_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    case 3: return casadi_s6;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_impl_dae_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 144;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif