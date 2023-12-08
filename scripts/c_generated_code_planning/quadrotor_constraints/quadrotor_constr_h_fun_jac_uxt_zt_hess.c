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
  #define CASADI_PREFIX(ID) quadrotor_constr_h_fun_jac_uxt_zt_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_densify CASADI_PREFIX(densify)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_sparsify CASADI_PREFIX(sparsify)
#define casadi_trans CASADI_PREFIX(trans)

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

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#define CASADI_CAST(x,y) ((x) y)

void casadi_densify(const casadi_real* x, const casadi_int* sp_x, casadi_real* y, casadi_int tr) {
  casadi_int nrow_x, ncol_x, i, el;
  const casadi_int *colind_x, *row_x;
  if (!y) return;
  nrow_x = sp_x[0]; ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x+ncol_x+3;
  casadi_clear(y, nrow_x*ncol_x);
  if (!x) return;
  if (tr) {
    for (i=0; i<ncol_x; ++i) {
      for (el=colind_x[i]; el!=colind_x[i+1]; ++el) {
        y[i + row_x[el]*ncol_x] = CASADI_CAST(casadi_real, *x++);
      }
    }
  } else {
    for (i=0; i<ncol_x; ++i) {
      for (el=colind_x[i]; el!=colind_x[i+1]; ++el) {
        y[row_x[el]] = CASADI_CAST(casadi_real, *x++);
      }
      y += nrow_x;
    }
  }
}

void casadi_sparsify(const casadi_real* x, casadi_real* y, const casadi_int* sp_y, casadi_int tr) {
  casadi_int nrow_y, ncol_y, i, el;
  const casadi_int *colind_y, *row_y;
  nrow_y = sp_y[0];
  ncol_y = sp_y[1];
  colind_y = sp_y+2; row_y = sp_y+ncol_y+3;
  if (tr) {
    for (i=0; i<ncol_y; ++i) {
      for (el=colind_y[i]; el!=colind_y[i+1]; ++el) {
        *y++ = CASADI_CAST(casadi_real, x[i + row_y[el]*ncol_y]);
      }
    }
  } else {
    for (i=0; i<ncol_y; ++i) {
      for (el=colind_y[i]; el!=colind_y[i+1]; ++el) {
        *y++ = CASADI_CAST(casadi_real, x[row_y[el]]);
      }
      x += nrow_y;
    }
  }
}

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
  }
}

static const casadi_int casadi_s0[5] = {4, 1, 0, 1, 0};
static const casadi_int casadi_s1[5] = {4, 1, 0, 1, 1};
static const casadi_int casadi_s2[5] = {4, 1, 0, 1, 2};
static const casadi_int casadi_s3[5] = {4, 1, 0, 1, 3};
static const casadi_int casadi_s4[36] = {17, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 8, 12, 16, 16, 16, 16, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13};
static const casadi_int casadi_s5[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s6[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s7[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s8[3] = {0, 0, 0};
static const casadi_int casadi_s9[8] = {17, 1, 0, 4, 10, 11, 12, 13};
static const casadi_int casadi_s10[3] = {1, 0, 0};

/* quadrotor_constr_h_fun_jac_uxt_zt_hess:(i0[13],i1[4],i2,i3[],i4[13])->(o0,o1[17x1,4nz],o2[17x17,16nz],o3[1x0],o4[]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cr, *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, *w13=w+13, *w14=w+26, *w15=w+30, *w16=w+34, *w17=w+38, *w22=w+54, w23, w24, w25, *w26=w+71;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = input[0][1] */
  w1 = arg[0] ? arg[0][1] : 0;
  /* #2: @2 = input[0][2] */
  w2 = arg[0] ? arg[0][2] : 0;
  /* #3: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #4: @4 = input[0][4] */
  w4 = arg[0] ? arg[0][4] : 0;
  /* #5: @5 = input[0][5] */
  w5 = arg[0] ? arg[0][5] : 0;
  /* #6: @6 = input[0][6] */
  w6 = arg[0] ? arg[0][6] : 0;
  /* #7: @7 = input[0][7] */
  w7 = arg[0] ? arg[0][7] : 0;
  /* #8: @8 = input[0][8] */
  w8 = arg[0] ? arg[0][8] : 0;
  /* #9: @9 = input[0][9] */
  w9 = arg[0] ? arg[0][9] : 0;
  /* #10: @10 = input[0][10] */
  w10 = arg[0] ? arg[0][10] : 0;
  /* #11: @11 = input[0][11] */
  w11 = arg[0] ? arg[0][11] : 0;
  /* #12: @12 = input[0][12] */
  w12 = arg[0] ? arg[0][12] : 0;
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
  /* #14: @14 = @13[6:10] */
  for (rr=w14, ss=w13+6; ss!=w13+10; ss+=1) *rr++ = *ss;
  /* #15: @0 = ||@14||_F */
  w0 = sqrt(casadi_dot(4, w14, w14));
  /* #16: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #17: @15 = zeros(17x1,4nz) */
  casadi_clear(w15, 4);
  /* #18: @13 = zeros(13x1) */
  casadi_clear(w13, 13);
  /* #19: @16 = (@14/@0) */
  for (i=0, rr=w16, cr=w14; i<4; ++i) (*rr++)  = ((*cr++)/w0);
  /* #20: (@13[6:10] += @16) */
  for (rr=w13+6, ss=w16; rr!=w13+10; rr+=1) *rr += *ss++;
  /* #21: {NULL, NULL, NULL, NULL, NULL, NULL, @1, @2, @3, @4, NULL, NULL, NULL} = vertsplit(@13) */
  w1 = w13[6];
  w2 = w13[7];
  w3 = w13[8];
  w4 = w13[9];
  /* #22: (@15[0] = @1) */
  for (rr=w15+0, ss=(&w1); rr!=w15+1; rr+=1) *rr = *ss++;
  /* #23: (@15[1] = @2) */
  for (rr=w15+1, ss=(&w2); rr!=w15+2; rr+=1) *rr = *ss++;
  /* #24: (@15[2] = @3) */
  for (rr=w15+2, ss=(&w3); rr!=w15+3; rr+=1) *rr = *ss++;
  /* #25: (@15[3] = @4) */
  for (rr=w15+3, ss=(&w4); rr!=w15+4; rr+=1) *rr = *ss++;
  /* #26: output[1][0] = @15 */
  casadi_copy(w15, 4, res[1]);
  /* #27: @17 = zeros(17x17,16nz) */
  casadi_clear(w17, 16);
  /* #28: @18 = 00 */
  /* #29: @19 = 00 */
  /* #30: @20 = 00 */
  /* #31: @21 = 00 */
  /* #32: @13 = zeros(13x1) */
  casadi_clear(w13, 13);
  /* #33: @4 = input[2][0] */
  w4 = arg[2] ? arg[2][0] : 0;
  /* #34: @4 = (@4/@0) */
  w4 /= w0;
  /* #35: @22 = ones(17x1,14nz) */
  casadi_fill(w22, 14, 1.);
  /* #36: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @3, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@22) */
  w3 = w22[10];
  /* #37: @3 = sparsity_cast(@3) */
  /* #38: @2 = (@4*@3) */
  w2  = (w4*w3);
  /* #39: @15 = dense(@2) */
  casadi_densify((&w2), casadi_s0, w15, 0);
  /* #40: @2 = (@4/@0) */
  w2  = (w4/w0);
  /* #41: @1 = project(@14) */
  casadi_sparsify(w14, (&w1), casadi_s0, 0);
  /* #42: @5 = dot(@1, @3) */
  w5 = casadi_dot(1, (&w1), (&w3));
  /* #43: @5 = (@5/@0) */
  w5 /= w0;
  /* #44: @5 = (@2*@5) */
  w5  = (w2*w5);
  /* #45: @16 = (@14*@5) */
  for (i=0, rr=w16, cr=w14; i<4; ++i) (*rr++)  = ((*cr++)*w5);
  /* #46: @15 = (@15-@16) */
  for (i=0, rr=w15, cs=w16; i<4; ++i) (*rr++) -= (*cs++);
  /* #47: (@13[6:10] += @15) */
  for (rr=w13+6, ss=w15; rr!=w13+10; rr+=1) *rr += *ss++;
  /* #48: {@5, @1, @3, @6, @7, @8, @9, @10, @11, @12, @23, @24, @25} = vertsplit(@13) */
  w5 = w13[0];
  w1 = w13[1];
  w3 = w13[2];
  w6 = w13[3];
  w7 = w13[4];
  w8 = w13[5];
  w9 = w13[6];
  w10 = w13[7];
  w11 = w13[8];
  w12 = w13[9];
  w23 = w13[10];
  w24 = w13[11];
  w25 = w13[12];
  /* #49: @13 = vertcat(@18, @19, @20, @21, @5, @1, @3, @6, @7, @8, @9, @10, @11, @12, @23, @24, @25) */
  rr=w13;
  *rr++ = w5;
  *rr++ = w1;
  *rr++ = w3;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w23;
  *rr++ = w24;
  *rr++ = w25;
  /* #50: @15 = @13[6:10] */
  for (rr=w15, ss=w13+6; ss!=w13+10; ss+=1) *rr++ = *ss;
  /* #51: (@17[:16:4] = @15) */
  for (rr=w17+0, ss=w15; rr!=w17+16; rr+=4) *rr = *ss++;
  /* #52: @13 = zeros(13x1) */
  casadi_clear(w13, 13);
  /* #53: @5 = ones(17x1,1nz) */
  w5 = 1.;
  /* #54: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @1, NULL, NULL, NULL, NULL, NULL} = vertsplit(@5) */
  w1 = w5;
  /* #55: @1 = sparsity_cast(@1) */
  /* #56: @5 = (@4*@1) */
  w5  = (w4*w1);
  /* #57: @15 = dense(@5) */
  casadi_densify((&w5), casadi_s1, w15, 0);
  /* #58: @5 = project(@14) */
  casadi_sparsify(w14, (&w5), casadi_s1, 0);
  /* #59: @3 = dot(@5, @1) */
  w3 = casadi_dot(1, (&w5), (&w1));
  /* #60: @3 = (@3/@0) */
  w3 /= w0;
  /* #61: @3 = (@2*@3) */
  w3  = (w2*w3);
  /* #62: @16 = (@14*@3) */
  for (i=0, rr=w16, cr=w14; i<4; ++i) (*rr++)  = ((*cr++)*w3);
  /* #63: @15 = (@15-@16) */
  for (i=0, rr=w15, cs=w16; i<4; ++i) (*rr++) -= (*cs++);
  /* #64: (@13[6:10] += @15) */
  for (rr=w13+6, ss=w15; rr!=w13+10; rr+=1) *rr += *ss++;
  /* #65: {@3, @5, @1, @6, @7, @8, @9, @10, @11, @12, @23, @24, @25} = vertsplit(@13) */
  w3 = w13[0];
  w5 = w13[1];
  w1 = w13[2];
  w6 = w13[3];
  w7 = w13[4];
  w8 = w13[5];
  w9 = w13[6];
  w10 = w13[7];
  w11 = w13[8];
  w12 = w13[9];
  w23 = w13[10];
  w24 = w13[11];
  w25 = w13[12];
  /* #66: @13 = vertcat(@18, @19, @20, @21, @3, @5, @1, @6, @7, @8, @9, @10, @11, @12, @23, @24, @25) */
  rr=w13;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w1;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w23;
  *rr++ = w24;
  *rr++ = w25;
  /* #67: @15 = @13[6:10] */
  for (rr=w15, ss=w13+6; ss!=w13+10; ss+=1) *rr++ = *ss;
  /* #68: (@17[1:17:4] = @15) */
  for (rr=w17+1, ss=w15; rr!=w17+17; rr+=4) *rr = *ss++;
  /* #69: @13 = zeros(13x1) */
  casadi_clear(w13, 13);
  /* #70: @3 = ones(17x1,1nz) */
  w3 = 1.;
  /* #71: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @5, NULL, NULL, NULL, NULL} = vertsplit(@3) */
  w5 = w3;
  /* #72: @5 = sparsity_cast(@5) */
  /* #73: @3 = (@4*@5) */
  w3  = (w4*w5);
  /* #74: @15 = dense(@3) */
  casadi_densify((&w3), casadi_s2, w15, 0);
  /* #75: @3 = project(@14) */
  casadi_sparsify(w14, (&w3), casadi_s2, 0);
  /* #76: @1 = dot(@3, @5) */
  w1 = casadi_dot(1, (&w3), (&w5));
  /* #77: @1 = (@1/@0) */
  w1 /= w0;
  /* #78: @1 = (@2*@1) */
  w1  = (w2*w1);
  /* #79: @16 = (@14*@1) */
  for (i=0, rr=w16, cr=w14; i<4; ++i) (*rr++)  = ((*cr++)*w1);
  /* #80: @15 = (@15-@16) */
  for (i=0, rr=w15, cs=w16; i<4; ++i) (*rr++) -= (*cs++);
  /* #81: (@13[6:10] += @15) */
  for (rr=w13+6, ss=w15; rr!=w13+10; rr+=1) *rr += *ss++;
  /* #82: {@1, @3, @5, @6, @7, @8, @9, @10, @11, @12, @23, @24, @25} = vertsplit(@13) */
  w1 = w13[0];
  w3 = w13[1];
  w5 = w13[2];
  w6 = w13[3];
  w7 = w13[4];
  w8 = w13[5];
  w9 = w13[6];
  w10 = w13[7];
  w11 = w13[8];
  w12 = w13[9];
  w23 = w13[10];
  w24 = w13[11];
  w25 = w13[12];
  /* #83: @13 = vertcat(@18, @19, @20, @21, @1, @3, @5, @6, @7, @8, @9, @10, @11, @12, @23, @24, @25) */
  rr=w13;
  *rr++ = w1;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w23;
  *rr++ = w24;
  *rr++ = w25;
  /* #84: @15 = @13[6:10] */
  for (rr=w15, ss=w13+6; ss!=w13+10; ss+=1) *rr++ = *ss;
  /* #85: (@17[2:18:4] = @15) */
  for (rr=w17+2, ss=w15; rr!=w17+18; rr+=4) *rr = *ss++;
  /* #86: @13 = zeros(13x1) */
  casadi_clear(w13, 13);
  /* #87: @1 = ones(17x1,1nz) */
  w1 = 1.;
  /* #88: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @3, NULL, NULL, NULL} = vertsplit(@1) */
  w3 = w1;
  /* #89: @3 = sparsity_cast(@3) */
  /* #90: @4 = (@4*@3) */
  w4 *= w3;
  /* #91: @15 = dense(@4) */
  casadi_densify((&w4), casadi_s3, w15, 0);
  /* #92: @4 = project(@14) */
  casadi_sparsify(w14, (&w4), casadi_s3, 0);
  /* #93: @1 = dot(@4, @3) */
  w1 = casadi_dot(1, (&w4), (&w3));
  /* #94: @1 = (@1/@0) */
  w1 /= w0;
  /* #95: @2 = (@2*@1) */
  w2 *= w1;
  /* #96: @14 = (@14*@2) */
  for (i=0, rr=w14; i<4; ++i) (*rr++) *= w2;
  /* #97: @15 = (@15-@14) */
  for (i=0, rr=w15, cs=w14; i<4; ++i) (*rr++) -= (*cs++);
  /* #98: (@13[6:10] += @15) */
  for (rr=w13+6, ss=w15; rr!=w13+10; rr+=1) *rr += *ss++;
  /* #99: {@2, @1, @0, @4, @3, @5, @6, @7, @8, @9, @10, @11, @12} = vertsplit(@13) */
  w2 = w13[0];
  w1 = w13[1];
  w0 = w13[2];
  w4 = w13[3];
  w3 = w13[4];
  w5 = w13[5];
  w6 = w13[6];
  w7 = w13[7];
  w8 = w13[8];
  w9 = w13[9];
  w10 = w13[10];
  w11 = w13[11];
  w12 = w13[12];
  /* #100: @13 = vertcat(@18, @19, @20, @21, @2, @1, @0, @4, @3, @5, @6, @7, @8, @9, @10, @11, @12) */
  rr=w13;
  *rr++ = w2;
  *rr++ = w1;
  *rr++ = w0;
  *rr++ = w4;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  /* #101: @15 = @13[6:10] */
  for (rr=w15, ss=w13+6; ss!=w13+10; ss+=1) *rr++ = *ss;
  /* #102: (@17[3:19:4] = @15) */
  for (rr=w17+3, ss=w15; rr!=w17+19; rr+=4) *rr = *ss++;
  /* #103: @26 = @17' */
  casadi_trans(w17,casadi_s4, w26, casadi_s4, iw);
  /* #104: output[2][0] = @26 */
  casadi_copy(w26, 16, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_jac_uxt_zt_hess_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_jac_uxt_zt_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_constr_h_fun_jac_uxt_zt_hess_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_jac_uxt_zt_hess_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_jac_uxt_zt_hess_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_jac_uxt_zt_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    case 2: return casadi_s7;
    case 3: return casadi_s8;
    case 4: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_jac_uxt_zt_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s7;
    case 1: return casadi_s9;
    case 2: return casadi_s4;
    case 3: return casadi_s10;
    case 4: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 22;
  if (sz_res) *sz_res = 22;
  if (sz_iw) *sz_iw = 18;
  if (sz_w) *sz_w = 87;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif