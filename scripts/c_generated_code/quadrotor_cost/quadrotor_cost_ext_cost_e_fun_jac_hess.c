/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_e_fun_jac_hess_ ## ID
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
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_mtimes CASADI_PREFIX(mtimes)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s12 CASADI_PREFIX(s12)
#define casadi_s13 CASADI_PREFIX(s13)
#define casadi_s14 CASADI_PREFIX(s14)
#define casadi_s15 CASADI_PREFIX(s15)
#define casadi_s16 CASADI_PREFIX(s16)
#define casadi_s17 CASADI_PREFIX(s17)
#define casadi_s18 CASADI_PREFIX(s18)
#define casadi_s19 CASADI_PREFIX(s19)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s20 CASADI_PREFIX(s20)
#define casadi_s21 CASADI_PREFIX(s21)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
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

void casadi_mtimes(const casadi_real* x, const casadi_int* sp_x, const casadi_real* y, const casadi_int* sp_y, casadi_real* z, const casadi_int* sp_z, casadi_real* w, casadi_int tr) {
  casadi_int ncol_x, ncol_y, ncol_z, cc;
  const casadi_int *colind_x, *row_x, *colind_y, *row_y, *colind_z, *row_z;
  ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2; row_y = sp_y + 2 + ncol_y+1;
  ncol_z = sp_z[1];
  colind_z = sp_z+2; row_z = sp_z + 2 + ncol_z+1;
  if (tr) {
    for (cc=0; cc<ncol_z; ++cc) {
      casadi_int kk;
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        w[row_y[kk]] = y[kk];
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_z[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          z[kk] += x[kk1] * w[row_x[kk1]];
        }
      }
    }
  } else {
    for (cc=0; cc<ncol_y; ++cc) {
      casadi_int kk;
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        w[row_z[kk]] = z[kk];
      }
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_y[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          w[row_x[kk1]] += x[kk1]*y[kk];
        }
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        z[kk] = w[row_z[kk]];
      }
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

static const casadi_int casadi_s0[9] = {1, 3, 0, 1, 2, 3, 0, 0, 0};
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s2[7] = {1, 3, 0, 1, 1, 1, 0};
static const casadi_int casadi_s3[11] = {4, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3};
static const casadi_int casadi_s4[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s5[7] = {0, 1, 2, 6, 7, 8, 9};
static const casadi_int casadi_s6[7] = {0, 1, 2, 9, 10, 11, 12};
static const casadi_int casadi_s7[7] = {0, 3, 6, 9, 13, 17, 21};
static const casadi_int casadi_s8[7] = {1, 3, 0, 0, 1, 1, 0};
static const casadi_int casadi_s9[11] = {4, 4, 0, 1, 2, 3, 4, 1, 0, 3, 2};
static const casadi_int casadi_s10[7] = {3, 4, 5, 13, 14, 15, 16};
static const casadi_int casadi_s11[7] = {1, 4, 7, 10, 14, 18, 22};
static const casadi_int casadi_s12[7] = {1, 3, 0, 0, 0, 1, 0};
static const casadi_int casadi_s13[11] = {4, 4, 0, 1, 2, 3, 4, 2, 3, 0, 1};
static const casadi_int casadi_s14[7] = {6, 7, 8, 17, 18, 19, 20};
static const casadi_int casadi_s15[7] = {2, 5, 8, 11, 15, 19, 23};
static const casadi_int casadi_s16[11] = {4, 4, 0, 1, 2, 3, 4, 3, 2, 1, 0};
static const casadi_int casadi_s17[41] = {13, 13, 0, 3, 6, 9, 9, 9, 9, 13, 17, 21, 25, 25, 25, 25, 0, 1, 2, 0, 1, 2, 0, 1, 2, 6, 7, 8, 9, 6, 7, 8, 9, 6, 7, 8, 9, 6, 7, 8, 9};
static const casadi_int casadi_s18[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s19[3] = {0, 0, 0};
static const casadi_int casadi_s20[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s21[16] = {0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* quadrotor_cost_ext_cost_e_fun_jac_hess:(i0[13],i1[],i2[],i3[13])->(o0,o1[13],o2[13x13,25nz],o3[],o4[0x13]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real w0, *w1=w+5, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, *w15=w+21, *w16=w+34, w17, w18, w19, w20, *w21=w+41, *w22=w+44, *w23=w+53, *w24=w+57, *w25=w+61, *w26=w+65, *w27=w+69, *w28=w+73, *w29=w+89, *w30=w+105, *w31=w+108, *w32=w+111, *w33=w+120, w34, w35, w36, *w37=w+127, *w38=w+152, w43, w44, w45, w46, w47, w48, *w49=w+166, *w50=w+173, *w57=w+175;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = zeros(1x3) */
  casadi_clear(w1, 3);
  /* #2: @2 = input[3][0] */
  w2 = arg[3] ? arg[3][0] : 0;
  /* #3: @3 = input[3][1] */
  w3 = arg[3] ? arg[3][1] : 0;
  /* #4: @4 = input[3][2] */
  w4 = arg[3] ? arg[3][2] : 0;
  /* #5: @5 = input[3][3] */
  w5 = arg[3] ? arg[3][3] : 0;
  /* #6: @6 = input[3][4] */
  w6 = arg[3] ? arg[3][4] : 0;
  /* #7: @7 = input[3][5] */
  w7 = arg[3] ? arg[3][5] : 0;
  /* #8: @8 = input[3][6] */
  w8 = arg[3] ? arg[3][6] : 0;
  /* #9: @9 = input[3][7] */
  w9 = arg[3] ? arg[3][7] : 0;
  /* #10: @10 = input[3][8] */
  w10 = arg[3] ? arg[3][8] : 0;
  /* #11: @11 = input[3][9] */
  w11 = arg[3] ? arg[3][9] : 0;
  /* #12: @12 = input[3][10] */
  w12 = arg[3] ? arg[3][10] : 0;
  /* #13: @13 = input[3][11] */
  w13 = arg[3] ? arg[3][11] : 0;
  /* #14: @14 = input[3][12] */
  w14 = arg[3] ? arg[3][12] : 0;
  /* #15: @15 = vertcat(@2, @3, @4, @5, @6, @7, @8, @9, @10, @11, @12, @13, @14) */
  rr=w15;
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
  *rr++ = w13;
  *rr++ = w14;
  /* #16: @16 = @15[:3] */
  for (rr=w16, ss=w15+0; ss!=w15+3; ss+=1) *rr++ = *ss;
  /* #17: @2 = input[0][0] */
  w2 = arg[0] ? arg[0][0] : 0;
  /* #18: @3 = input[0][1] */
  w3 = arg[0] ? arg[0][1] : 0;
  /* #19: @4 = input[0][2] */
  w4 = arg[0] ? arg[0][2] : 0;
  /* #20: @5 = input[0][3] */
  w5 = arg[0] ? arg[0][3] : 0;
  /* #21: @6 = input[0][4] */
  w6 = arg[0] ? arg[0][4] : 0;
  /* #22: @7 = input[0][5] */
  w7 = arg[0] ? arg[0][5] : 0;
  /* #23: @12 = input[0][6] */
  w12 = arg[0] ? arg[0][6] : 0;
  /* #24: @13 = input[0][7] */
  w13 = arg[0] ? arg[0][7] : 0;
  /* #25: @14 = input[0][8] */
  w14 = arg[0] ? arg[0][8] : 0;
  /* #26: @17 = input[0][9] */
  w17 = arg[0] ? arg[0][9] : 0;
  /* #27: @18 = input[0][10] */
  w18 = arg[0] ? arg[0][10] : 0;
  /* #28: @19 = input[0][11] */
  w19 = arg[0] ? arg[0][11] : 0;
  /* #29: @20 = input[0][12] */
  w20 = arg[0] ? arg[0][12] : 0;
  /* #30: @15 = vertcat(@2, @3, @4, @5, @6, @7, @12, @13, @14, @17, @18, @19, @20) */
  rr=w15;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w14;
  *rr++ = w17;
  *rr++ = w18;
  *rr++ = w19;
  *rr++ = w20;
  /* #31: @21 = @15[:3] */
  for (rr=w21, ss=w15+0; ss!=w15+3; ss+=1) *rr++ = *ss;
  /* #32: @16 = (@16-@21) */
  for (i=0, rr=w16, cs=w21; i<3; ++i) (*rr++) -= (*cs++);
  /* #33: @21 = @16' */
  casadi_copy(w16, 3, w21);
  /* #34: @22 = zeros(3x3) */
  casadi_clear(w22, 9);
  /* #35: @2 = 1.5 */
  w2 = 1.5000000000000000e+00;
  /* #36: (@22[0] = @2) */
  for (rr=w22+0, ss=(&w2); rr!=w22+1; rr+=1) *rr = *ss++;
  /* #37: @2 = 1.5 */
  w2 = 1.5000000000000000e+00;
  /* #38: (@22[4] = @2) */
  for (rr=w22+4, ss=(&w2); rr!=w22+5; rr+=1) *rr = *ss++;
  /* #39: @2 = 10 */
  w2 = 10.;
  /* #40: (@22[8] = @2) */
  for (rr=w22+8, ss=(&w2); rr!=w22+9; rr+=1) *rr = *ss++;
  /* #41: @1 = mac(@21,@22,@1) */
  for (i=0, rr=w1; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w21+j, tt=w22+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #42: @0 = mac(@1,@16,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w16+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #43: @2 = 1 */
  w2 = 1.;
  /* #44: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #45: @3 = (-@13) */
  w3 = (- w13 );
  /* #46: @4 = (-@14) */
  w4 = (- w14 );
  /* #47: @5 = (-@17) */
  w5 = (- w17 );
  /* #48: @24 = horzcat(@12, @3, @4, @5) */
  rr=w24;
  *rr++ = w12;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  /* #49: @24 = @24' */
  /* #50: @3 = (-@17) */
  w3 = (- w17 );
  /* #51: @25 = horzcat(@13, @12, @3, @14) */
  rr=w25;
  *rr++ = w13;
  *rr++ = w12;
  *rr++ = w3;
  *rr++ = w14;
  /* #52: @25 = @25' */
  /* #53: @3 = (-@13) */
  w3 = (- w13 );
  /* #54: @26 = horzcat(@14, @17, @12, @3) */
  rr=w26;
  *rr++ = w14;
  *rr++ = w17;
  *rr++ = w12;
  *rr++ = w3;
  /* #55: @26 = @26' */
  /* #56: @14 = (-@14) */
  w14 = (- w14 );
  /* #57: @27 = horzcat(@17, @14, @13, @12) */
  rr=w27;
  *rr++ = w17;
  *rr++ = w14;
  *rr++ = w13;
  *rr++ = w12;
  /* #58: @27 = @27' */
  /* #59: @28 = horzcat(@24, @25, @26, @27) */
  rr=w28;
  for (i=0, cs=w24; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w25; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w27; i<4; ++i) *rr++ = *cs++;
  /* #60: @29 = @28' */
  for (i=0, rr=w29, cs=w28; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #61: @9 = (-@9) */
  w9 = (- w9 );
  /* #62: @10 = (-@10) */
  w10 = (- w10 );
  /* #63: @11 = (-@11) */
  w11 = (- w11 );
  /* #64: @24 = vertcat(@8, @9, @10, @11) */
  rr=w24;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #65: @23 = mac(@29,@24,@23) */
  for (i=0, rr=w23; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w29+j, tt=w24+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #66: @8 = @23[0] */
  for (rr=(&w8), ss=w23+0; ss!=w23+1; ss+=1) *rr++ = *ss;
  /* #67: @2 = (@2-@8) */
  w2 -= w8;
  /* #68: @8 = 0 */
  w8 = 0.;
  /* #69: @21 = @23[1:4] */
  for (rr=w21, ss=w23+1; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #70: @30 = @21' */
  casadi_copy(w21, 3, w30);
  /* #71: @31 = @23[1:4] */
  for (rr=w31, ss=w23+1; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #72: @8 = mac(@30,@31,@8) */
  for (i=0, rr=(&w8); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w30+j, tt=w31+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #73: @2 = (@2+@8) */
  w2 += w8;
  /* #74: @0 = (@0+@2) */
  w0 += w2;
  /* #75: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #76: @15 = zeros(13x1) */
  casadi_clear(w15, 13);
  /* #77: @1 = @1' */
  /* #78: @30 = zeros(1x3) */
  casadi_clear(w30, 3);
  /* #79: @16 = @16' */
  /* #80: @32 = @22' */
  for (i=0, rr=w32, cs=w22; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #81: @30 = mac(@16,@32,@30) */
  for (i=0, rr=w30; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w16+j, tt=w32+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #82: @30 = @30' */
  /* #83: @1 = (@1+@30) */
  for (i=0, rr=w1, cs=w30; i<3; ++i) (*rr++) += (*cs++);
  /* #84: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<3; ++i) *rr++ = (- *cs++ );
  /* #85: (@15[:3] += @1) */
  for (rr=w15+0, ss=w1; rr!=w15+3; rr+=1) *rr += *ss++;
  /* #86: {@0, @2, @8, @9, @10, @11, @17, @14, @13, @12, @3, @4, @5} = vertsplit(@15) */
  w0 = w15[0];
  w2 = w15[1];
  w8 = w15[2];
  w9 = w15[3];
  w10 = w15[4];
  w11 = w15[5];
  w17 = w15[6];
  w14 = w15[7];
  w13 = w15[8];
  w12 = w15[9];
  w3 = w15[10];
  w4 = w15[11];
  w5 = w15[12];
  /* #87: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #88: output[1][1] = @2 */
  if (res[1]) res[1][1] = w2;
  /* #89: output[1][2] = @8 */
  if (res[1]) res[1][2] = w8;
  /* #90: output[1][3] = @9 */
  if (res[1]) res[1][3] = w9;
  /* #91: output[1][4] = @10 */
  if (res[1]) res[1][4] = w10;
  /* #92: output[1][5] = @11 */
  if (res[1]) res[1][5] = w11;
  /* #93: @29 = zeros(4x4) */
  casadi_clear(w29, 16);
  /* #94: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #95: (@23[1:4] += @21) */
  for (rr=w23+1, ss=w21; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #96: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #97: @11 = -1 */
  w11 = -1.;
  /* #98: (@23[0] += @11) */
  for (rr=w23+0, ss=(&w11); rr!=w23+1; rr+=1) *rr += *ss++;
  /* #99: @25 = @24' */
  casadi_copy(w24, 4, w25);
  /* #100: @29 = mac(@23,@25,@29) */
  for (i=0, rr=w29; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w25+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #101: @28 = @29' */
  for (i=0, rr=w28, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #102: {@23, @26, @27, @33} = horzsplit(@28) */
  casadi_copy(w28, 4, w23);
  casadi_copy(w28+4, 4, w26);
  casadi_copy(w28+8, 4, w27);
  casadi_copy(w28+12, 4, w33);
  /* #103: @33 = @33' */
  /* #104: {@11, @10, @9, @8} = horzsplit(@33) */
  w11 = w33[0];
  w10 = w33[1];
  w9 = w33[2];
  w8 = w33[3];
  /* #105: @27 = @27' */
  /* #106: {@2, @0, @6, @7} = horzsplit(@27) */
  w2 = w27[0];
  w0 = w27[1];
  w6 = w27[2];
  w7 = w27[3];
  /* #107: @8 = (@8+@6) */
  w8 += w6;
  /* #108: @26 = @26' */
  /* #109: {@6, @18, @19, @20} = horzsplit(@26) */
  w6 = w26[0];
  w18 = w26[1];
  w19 = w26[2];
  w20 = w26[3];
  /* #110: @8 = (@8+@18) */
  w8 += w18;
  /* #111: @23 = @23' */
  /* #112: {@18, @34, @35, @36} = horzsplit(@23) */
  w18 = w23[0];
  w34 = w23[1];
  w35 = w23[2];
  w36 = w23[3];
  /* #113: @8 = (@8+@18) */
  w8 += w18;
  /* #114: @8 = (@8+@17) */
  w8 += w17;
  /* #115: output[1][6] = @8 */
  if (res[1]) res[1][6] = w8;
  /* #116: @9 = (@9-@7) */
  w9 -= w7;
  /* #117: @9 = (@9+@6) */
  w9 += w6;
  /* #118: @9 = (@9-@34) */
  w9 -= w34;
  /* #119: @9 = (@9+@14) */
  w9 += w14;
  /* #120: output[1][7] = @9 */
  if (res[1]) res[1][7] = w9;
  /* #121: @2 = (@2-@10) */
  w2 -= w10;
  /* #122: @2 = (@2+@20) */
  w2 += w20;
  /* #123: @2 = (@2-@35) */
  w2 -= w35;
  /* #124: @2 = (@2+@13) */
  w2 += w13;
  /* #125: output[1][8] = @2 */
  if (res[1]) res[1][8] = w2;
  /* #126: @11 = (@11+@0) */
  w11 += w0;
  /* #127: @11 = (@11-@19) */
  w11 -= w19;
  /* #128: @11 = (@11-@36) */
  w11 -= w36;
  /* #129: @11 = (@11+@12) */
  w11 += w12;
  /* #130: output[1][9] = @11 */
  if (res[1]) res[1][9] = w11;
  /* #131: output[1][10] = @3 */
  if (res[1]) res[1][10] = w3;
  /* #132: output[1][11] = @4 */
  if (res[1]) res[1][11] = w4;
  /* #133: output[1][12] = @5 */
  if (res[1]) res[1][12] = w5;
  /* #134: @37 = zeros(13x13,25nz) */
  casadi_clear(w37, 25);
  /* #135: @15 = zeros(13x1) */
  casadi_clear(w15, 13);
  /* #136: @31 = zeros(1x3) */
  casadi_clear(w31, 3);
  /* #137: @38 = ones(13x1,8nz) */
  casadi_fill(w38, 8, 1.);
  /* #138: {@5, NULL, NULL, NULL, NULL, NULL, @4, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@38) */
  w5 = w38[0];
  w4 = w38[4];
  /* #139: @3 = @5[0] */
  for (rr=(&w3), ss=(&w5)+0; ss!=(&w5)+1; ss+=1) *rr++ = *ss;
  /* #140: @3 = (-@3) */
  w3 = (- w3 );
  /* #141: @5 = @3' */
  casadi_copy((&w3), 1, (&w5));
  /* #142: @31 = mac(@5,@22,@31) */
  casadi_mtimes((&w5), casadi_s2, w22, casadi_s1, w31, casadi_s0, w, 0);
  /* #143: @31 = @31' */
  /* #144: @21 = zeros(1x3) */
  casadi_clear(w21, 3);
  /* #145: @3 = @3' */
  /* #146: @21 = mac(@3,@32,@21) */
  casadi_mtimes((&w3), casadi_s2, w32, casadi_s1, w21, casadi_s0, w, 0);
  /* #147: @21 = @21' */
  /* #148: @31 = (@31+@21) */
  for (i=0, rr=w31, cs=w21; i<3; ++i) (*rr++) += (*cs++);
  /* #149: @31 = (-@31) */
  for (i=0, rr=w31, cs=w31; i<3; ++i) *rr++ = (- *cs++ );
  /* #150: (@15[:3] += @31) */
  for (rr=w15+0, ss=w31; rr!=w15+3; rr+=1) *rr += *ss++;
  /* #151: {@3, @5, @11, @12, @36, @19, @0, @2, @13, @35, @20, @10, @9} = vertsplit(@15) */
  w3 = w15[0];
  w5 = w15[1];
  w11 = w15[2];
  w12 = w15[3];
  w36 = w15[4];
  w19 = w15[5];
  w0 = w15[6];
  w2 = w15[7];
  w13 = w15[8];
  w35 = w15[9];
  w20 = w15[10];
  w10 = w15[11];
  w9 = w15[12];
  /* #152: @28 = zeros(4x4) */
  casadi_clear(w28, 16);
  /* #153: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #154: @26 = zeros(4x1) */
  casadi_clear(w26, 4);
  /* #155: @39 = 00 */
  /* #156: @40 = 00 */
  /* #157: @41 = 00 */
  /* #158: @14 = horzcat(@4, @39, @40, @41) */
  rr=(&w14);
  *rr++ = w4;
  /* #159: @14 = @14' */
  /* #160: @39 = 00 */
  /* #161: @40 = 00 */
  /* #162: @41 = 00 */
  /* #163: @34 = horzcat(@39, @4, @40, @41) */
  rr=(&w34);
  *rr++ = w4;
  /* #164: @34 = @34' */
  /* #165: @40 = 00 */
  /* #166: @42 = 00 */
  /* #167: @6 = horzcat(@41, @40, @4, @42) */
  rr=(&w6);
  *rr++ = w4;
  /* #168: @6 = @6' */
  /* #169: @41 = 00 */
  /* #170: @7 = horzcat(@40, @41, @39, @4) */
  rr=(&w7);
  *rr++ = w4;
  /* #171: @7 = @7' */
  /* #172: @27 = horzcat(@14, @34, @6, @7) */
  rr=w27;
  *rr++ = w14;
  *rr++ = w34;
  *rr++ = w6;
  *rr++ = w7;
  /* #173: @33 = @27' */
  casadi_trans(w27,casadi_s3, w33, casadi_s3, iw);
  /* #174: @26 = mac(@33,@24,@26) */
  casadi_mtimes(w33, casadi_s3, w24, casadi_s4, w26, casadi_s4, w, 0);
  /* #175: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #176: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #177: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #178: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #179: @28 = mac(@23,@25,@28) */
  for (i=0, rr=w28; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w25+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #180: @29 = @28' */
  for (i=0, rr=w29, cs=w28; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #181: {@23, @26, @33, @27} = horzsplit(@29) */
  casadi_copy(w29, 4, w23);
  casadi_copy(w29+4, 4, w26);
  casadi_copy(w29+8, 4, w33);
  casadi_copy(w29+12, 4, w27);
  /* #182: @27 = @27' */
  /* #183: {@14, @34, @6, @7} = horzsplit(@27) */
  w14 = w27[0];
  w34 = w27[1];
  w6 = w27[2];
  w7 = w27[3];
  /* #184: @33 = @33' */
  /* #185: {@4, @8, @17, @18} = horzsplit(@33) */
  w4 = w33[0];
  w8 = w33[1];
  w17 = w33[2];
  w18 = w33[3];
  /* #186: @7 = (@7+@17) */
  w7 += w17;
  /* #187: @26 = @26' */
  /* #188: {@17, @43, @44, @45} = horzsplit(@26) */
  w17 = w26[0];
  w43 = w26[1];
  w44 = w26[2];
  w45 = w26[3];
  /* #189: @7 = (@7+@43) */
  w7 += w43;
  /* #190: @23 = @23' */
  /* #191: {@43, @46, @47, @48} = horzsplit(@23) */
  w43 = w23[0];
  w46 = w23[1];
  w47 = w23[2];
  w48 = w23[3];
  /* #192: @7 = (@7+@43) */
  w7 += w43;
  /* #193: @7 = (@7+@0) */
  w7 += w0;
  /* #194: @6 = (@6-@18) */
  w6 -= w18;
  /* #195: @6 = (@6+@17) */
  w6 += w17;
  /* #196: @6 = (@6-@46) */
  w6 -= w46;
  /* #197: @6 = (@6+@2) */
  w6 += w2;
  /* #198: @4 = (@4-@34) */
  w4 -= w34;
  /* #199: @4 = (@4+@45) */
  w4 += w45;
  /* #200: @4 = (@4-@47) */
  w4 -= w47;
  /* #201: @4 = (@4+@13) */
  w4 += w13;
  /* #202: @14 = (@14+@8) */
  w14 += w8;
  /* #203: @14 = (@14-@44) */
  w14 -= w44;
  /* #204: @14 = (@14-@48) */
  w14 -= w48;
  /* #205: @14 = (@14+@35) */
  w14 += w35;
  /* #206: @15 = vertcat(@3, @5, @11, @12, @36, @19, @7, @6, @4, @14, @20, @10, @9) */
  rr=w15;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w36;
  *rr++ = w19;
  *rr++ = w7;
  *rr++ = w6;
  *rr++ = w4;
  *rr++ = w14;
  *rr++ = w20;
  *rr++ = w10;
  *rr++ = w9;
  /* #207: @49 = @15[0, 1, 2, 6, 7, 8, 9] */
  for (cii=casadi_s5, rr=w49, ss=w15; cii!=casadi_s5+7; ++cii) *rr++ = *cii>=0 ? ss[*cii] : 0;
  /* #208: (@37[0, 1, 2, 9, 10, 11, 12] = @49) */
  for (cii=casadi_s6, rr=w37, ss=w49; cii!=casadi_s6+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #209: @49 = @15[0, 1, 2, 6, 7, 8, 9] */
  for (cii=casadi_s5, rr=w49, ss=w15; cii!=casadi_s5+7; ++cii) *rr++ = *cii>=0 ? ss[*cii] : 0;
  /* #210: (@37[0, 3, 6, 9, 13, 17, 21] = @49) */
  for (cii=casadi_s7, rr=w37, ss=w49; cii!=casadi_s7+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #211: @15 = zeros(13x1) */
  casadi_clear(w15, 13);
  /* #212: @31 = zeros(1x3) */
  casadi_clear(w31, 3);
  /* #213: @50 = ones(13x1,2nz) */
  casadi_fill(w50, 2, 1.);
  /* #214: {NULL, @3, NULL, NULL, NULL, NULL, NULL, @5, NULL, NULL, NULL, NULL, NULL} = vertsplit(@50) */
  w3 = w50[0];
  w5 = w50[1];
  /* #215: @11 = @3[0] */
  for (rr=(&w11), ss=(&w3)+0; ss!=(&w3)+1; ss+=1) *rr++ = *ss;
  /* #216: @11 = (-@11) */
  w11 = (- w11 );
  /* #217: @3 = @11' */
  casadi_copy((&w11), 1, (&w3));
  /* #218: @31 = mac(@3,@22,@31) */
  casadi_mtimes((&w3), casadi_s8, w22, casadi_s1, w31, casadi_s0, w, 0);
  /* #219: @31 = @31' */
  /* #220: @21 = zeros(1x3) */
  casadi_clear(w21, 3);
  /* #221: @11 = @11' */
  /* #222: @21 = mac(@11,@32,@21) */
  casadi_mtimes((&w11), casadi_s8, w32, casadi_s1, w21, casadi_s0, w, 0);
  /* #223: @21 = @21' */
  /* #224: @31 = (@31+@21) */
  for (i=0, rr=w31, cs=w21; i<3; ++i) (*rr++) += (*cs++);
  /* #225: @31 = (-@31) */
  for (i=0, rr=w31, cs=w31; i<3; ++i) *rr++ = (- *cs++ );
  /* #226: (@15[:3] += @31) */
  for (rr=w15+0, ss=w31; rr!=w15+3; rr+=1) *rr += *ss++;
  /* #227: {@11, @3, @12, @36, @19, @7, @6, @4, @14, @20, @10, @9, @35} = vertsplit(@15) */
  w11 = w15[0];
  w3 = w15[1];
  w12 = w15[2];
  w36 = w15[3];
  w19 = w15[4];
  w7 = w15[5];
  w6 = w15[6];
  w4 = w15[7];
  w14 = w15[8];
  w20 = w15[9];
  w10 = w15[10];
  w9 = w15[11];
  w35 = w15[12];
  /* #228: @29 = zeros(4x4) */
  casadi_clear(w29, 16);
  /* #229: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #230: @26 = zeros(4x1) */
  casadi_clear(w26, 4);
  /* #231: @40 = 00 */
  /* #232: @48 = (-@5) */
  w48 = (- w5 );
  /* #233: @41 = 00 */
  /* #234: @39 = 00 */
  /* #235: @44 = horzcat(@40, @48, @41, @39) */
  rr=(&w44);
  *rr++ = w48;
  /* #236: @44 = @44' */
  /* #237: @41 = 00 */
  /* #238: @39 = 00 */
  /* #239: @48 = horzcat(@5, @40, @41, @39) */
  rr=(&w48);
  *rr++ = w5;
  /* #240: @48 = @48' */
  /* #241: @41 = 00 */
  /* #242: @8 = (-@5) */
  w8 = (- w5 );
  /* #243: @13 = horzcat(@39, @41, @40, @8) */
  rr=(&w13);
  *rr++ = w8;
  /* #244: @13 = @13' */
  /* #245: @39 = 00 */
  /* #246: @8 = horzcat(@41, @39, @5, @40) */
  rr=(&w8);
  *rr++ = w5;
  /* #247: @8 = @8' */
  /* #248: @33 = horzcat(@44, @48, @13, @8) */
  rr=w33;
  *rr++ = w44;
  *rr++ = w48;
  *rr++ = w13;
  *rr++ = w8;
  /* #249: @27 = @33' */
  casadi_trans(w33,casadi_s9, w27, casadi_s9, iw);
  /* #250: @26 = mac(@27,@24,@26) */
  casadi_mtimes(w27, casadi_s9, w24, casadi_s4, w26, casadi_s4, w, 0);
  /* #251: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #252: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #253: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #254: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #255: @29 = mac(@23,@25,@29) */
  for (i=0, rr=w29; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w25+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #256: @28 = @29' */
  for (i=0, rr=w28, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #257: {@23, @26, @27, @33} = horzsplit(@28) */
  casadi_copy(w28, 4, w23);
  casadi_copy(w28+4, 4, w26);
  casadi_copy(w28+8, 4, w27);
  casadi_copy(w28+12, 4, w33);
  /* #258: @33 = @33' */
  /* #259: {@44, @48, @13, @8} = horzsplit(@33) */
  w44 = w33[0];
  w48 = w33[1];
  w13 = w33[2];
  w8 = w33[3];
  /* #260: @27 = @27' */
  /* #261: {@5, @47, @45, @34} = horzsplit(@27) */
  w5 = w27[0];
  w47 = w27[1];
  w45 = w27[2];
  w34 = w27[3];
  /* #262: @8 = (@8+@45) */
  w8 += w45;
  /* #263: @26 = @26' */
  /* #264: {@45, @2, @46, @17} = horzsplit(@26) */
  w45 = w26[0];
  w2 = w26[1];
  w46 = w26[2];
  w17 = w26[3];
  /* #265: @8 = (@8+@2) */
  w8 += w2;
  /* #266: @23 = @23' */
  /* #267: {@2, @18, @0, @43} = horzsplit(@23) */
  w2 = w23[0];
  w18 = w23[1];
  w0 = w23[2];
  w43 = w23[3];
  /* #268: @8 = (@8+@2) */
  w8 += w2;
  /* #269: @8 = (@8+@6) */
  w8 += w6;
  /* #270: @13 = (@13-@34) */
  w13 -= w34;
  /* #271: @13 = (@13+@45) */
  w13 += w45;
  /* #272: @13 = (@13-@18) */
  w13 -= w18;
  /* #273: @13 = (@13+@4) */
  w13 += w4;
  /* #274: @5 = (@5-@48) */
  w5 -= w48;
  /* #275: @5 = (@5+@17) */
  w5 += w17;
  /* #276: @5 = (@5-@0) */
  w5 -= w0;
  /* #277: @5 = (@5+@14) */
  w5 += w14;
  /* #278: @44 = (@44+@47) */
  w44 += w47;
  /* #279: @44 = (@44-@46) */
  w44 -= w46;
  /* #280: @44 = (@44-@43) */
  w44 -= w43;
  /* #281: @44 = (@44+@20) */
  w44 += w20;
  /* #282: @15 = vertcat(@11, @3, @12, @36, @19, @7, @8, @13, @5, @44, @10, @9, @35) */
  rr=w15;
  *rr++ = w11;
  *rr++ = w3;
  *rr++ = w12;
  *rr++ = w36;
  *rr++ = w19;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w13;
  *rr++ = w5;
  *rr++ = w44;
  *rr++ = w10;
  *rr++ = w9;
  *rr++ = w35;
  /* #283: @49 = @15[0, 1, 2, 6, 7, 8, 9] */
  for (cii=casadi_s5, rr=w49, ss=w15; cii!=casadi_s5+7; ++cii) *rr++ = *cii>=0 ? ss[*cii] : 0;
  /* #284: (@37[3, 4, 5, 13, 14, 15, 16] = @49) */
  for (cii=casadi_s10, rr=w37, ss=w49; cii!=casadi_s10+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #285: @49 = @15[0, 1, 2, 6, 7, 8, 9] */
  for (cii=casadi_s5, rr=w49, ss=w15; cii!=casadi_s5+7; ++cii) *rr++ = *cii>=0 ? ss[*cii] : 0;
  /* #286: (@37[1, 4, 7, 10, 14, 18, 22] = @49) */
  for (cii=casadi_s11, rr=w37, ss=w49; cii!=casadi_s11+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #287: @15 = zeros(13x1) */
  casadi_clear(w15, 13);
  /* #288: @31 = zeros(1x3) */
  casadi_clear(w31, 3);
  /* #289: @50 = ones(13x1,2nz) */
  casadi_fill(w50, 2, 1.);
  /* #290: {NULL, NULL, @11, NULL, NULL, NULL, NULL, NULL, @3, NULL, NULL, NULL, NULL} = vertsplit(@50) */
  w11 = w50[0];
  w3 = w50[1];
  /* #291: @12 = @11[0] */
  for (rr=(&w12), ss=(&w11)+0; ss!=(&w11)+1; ss+=1) *rr++ = *ss;
  /* #292: @12 = (-@12) */
  w12 = (- w12 );
  /* #293: @11 = @12' */
  casadi_copy((&w12), 1, (&w11));
  /* #294: @31 = mac(@11,@22,@31) */
  casadi_mtimes((&w11), casadi_s12, w22, casadi_s1, w31, casadi_s0, w, 0);
  /* #295: @31 = @31' */
  /* #296: @21 = zeros(1x3) */
  casadi_clear(w21, 3);
  /* #297: @12 = @12' */
  /* #298: @21 = mac(@12,@32,@21) */
  casadi_mtimes((&w12), casadi_s12, w32, casadi_s1, w21, casadi_s0, w, 0);
  /* #299: @21 = @21' */
  /* #300: @31 = (@31+@21) */
  for (i=0, rr=w31, cs=w21; i<3; ++i) (*rr++) += (*cs++);
  /* #301: @31 = (-@31) */
  for (i=0, rr=w31, cs=w31; i<3; ++i) *rr++ = (- *cs++ );
  /* #302: (@15[:3] += @31) */
  for (rr=w15+0, ss=w31; rr!=w15+3; rr+=1) *rr += *ss++;
  /* #303: {@12, @11, @36, @19, @7, @8, @13, @5, @44, @10, @9, @35, @20} = vertsplit(@15) */
  w12 = w15[0];
  w11 = w15[1];
  w36 = w15[2];
  w19 = w15[3];
  w7 = w15[4];
  w8 = w15[5];
  w13 = w15[6];
  w5 = w15[7];
  w44 = w15[8];
  w10 = w15[9];
  w9 = w15[10];
  w35 = w15[11];
  w20 = w15[12];
  /* #304: @28 = zeros(4x4) */
  casadi_clear(w28, 16);
  /* #305: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #306: @26 = zeros(4x1) */
  casadi_clear(w26, 4);
  /* #307: @41 = 00 */
  /* #308: @39 = 00 */
  /* #309: @43 = (-@3) */
  w43 = (- w3 );
  /* #310: @40 = 00 */
  /* #311: @46 = horzcat(@41, @39, @43, @40) */
  rr=(&w46);
  *rr++ = w43;
  /* #312: @46 = @46' */
  /* #313: @39 = 00 */
  /* #314: @40 = 00 */
  /* #315: @43 = horzcat(@39, @41, @40, @3) */
  rr=(&w43);
  *rr++ = w3;
  /* #316: @43 = @43' */
  /* #317: @40 = 00 */
  /* #318: @42 = 00 */
  /* #319: @47 = horzcat(@3, @40, @41, @42) */
  rr=(&w47);
  *rr++ = w3;
  /* #320: @47 = @47' */
  /* #321: @3 = (-@3) */
  w3 = (- w3 );
  /* #322: @14 = horzcat(@40, @3, @39, @41) */
  rr=(&w14);
  *rr++ = w3;
  /* #323: @14 = @14' */
  /* #324: @27 = horzcat(@46, @43, @47, @14) */
  rr=w27;
  *rr++ = w46;
  *rr++ = w43;
  *rr++ = w47;
  *rr++ = w14;
  /* #325: @33 = @27' */
  casadi_trans(w27,casadi_s13, w33, casadi_s13, iw);
  /* #326: @26 = mac(@33,@24,@26) */
  casadi_mtimes(w33, casadi_s13, w24, casadi_s4, w26, casadi_s4, w, 0);
  /* #327: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #328: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #329: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #330: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #331: @28 = mac(@23,@25,@28) */
  for (i=0, rr=w28; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w25+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #332: @29 = @28' */
  for (i=0, rr=w29, cs=w28; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #333: {@23, @26, @33, @27} = horzsplit(@29) */
  casadi_copy(w29, 4, w23);
  casadi_copy(w29+4, 4, w26);
  casadi_copy(w29+8, 4, w33);
  casadi_copy(w29+12, 4, w27);
  /* #334: @27 = @27' */
  /* #335: {@46, @43, @47, @14} = horzsplit(@27) */
  w46 = w27[0];
  w43 = w27[1];
  w47 = w27[2];
  w14 = w27[3];
  /* #336: @33 = @33' */
  /* #337: {@3, @0, @17, @48} = horzsplit(@33) */
  w3 = w33[0];
  w0 = w33[1];
  w17 = w33[2];
  w48 = w33[3];
  /* #338: @14 = (@14+@17) */
  w14 += w17;
  /* #339: @26 = @26' */
  /* #340: {@17, @4, @18, @45} = horzsplit(@26) */
  w17 = w26[0];
  w4 = w26[1];
  w18 = w26[2];
  w45 = w26[3];
  /* #341: @14 = (@14+@4) */
  w14 += w4;
  /* #342: @23 = @23' */
  /* #343: {@4, @34, @6, @2} = horzsplit(@23) */
  w4 = w23[0];
  w34 = w23[1];
  w6 = w23[2];
  w2 = w23[3];
  /* #344: @14 = (@14+@4) */
  w14 += w4;
  /* #345: @14 = (@14+@13) */
  w14 += w13;
  /* #346: @47 = (@47-@48) */
  w47 -= w48;
  /* #347: @47 = (@47+@17) */
  w47 += w17;
  /* #348: @47 = (@47-@34) */
  w47 -= w34;
  /* #349: @47 = (@47+@5) */
  w47 += w5;
  /* #350: @3 = (@3-@43) */
  w3 -= w43;
  /* #351: @3 = (@3+@45) */
  w3 += w45;
  /* #352: @3 = (@3-@6) */
  w3 -= w6;
  /* #353: @3 = (@3+@44) */
  w3 += w44;
  /* #354: @46 = (@46+@0) */
  w46 += w0;
  /* #355: @46 = (@46-@18) */
  w46 -= w18;
  /* #356: @46 = (@46-@2) */
  w46 -= w2;
  /* #357: @46 = (@46+@10) */
  w46 += w10;
  /* #358: @15 = vertcat(@12, @11, @36, @19, @7, @8, @14, @47, @3, @46, @9, @35, @20) */
  rr=w15;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w36;
  *rr++ = w19;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w14;
  *rr++ = w47;
  *rr++ = w3;
  *rr++ = w46;
  *rr++ = w9;
  *rr++ = w35;
  *rr++ = w20;
  /* #359: @49 = @15[0, 1, 2, 6, 7, 8, 9] */
  for (cii=casadi_s5, rr=w49, ss=w15; cii!=casadi_s5+7; ++cii) *rr++ = *cii>=0 ? ss[*cii] : 0;
  /* #360: (@37[6, 7, 8, 17, 18, 19, 20] = @49) */
  for (cii=casadi_s14, rr=w37, ss=w49; cii!=casadi_s14+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #361: @49 = @15[0, 1, 2, 6, 7, 8, 9] */
  for (cii=casadi_s5, rr=w49, ss=w15; cii!=casadi_s5+7; ++cii) *rr++ = *cii>=0 ? ss[*cii] : 0;
  /* #362: (@37[2, 5, 8, 11, 15, 19, 23] = @49) */
  for (cii=casadi_s15, rr=w37, ss=w49; cii!=casadi_s15+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #363: @40 = 00 */
  /* #364: @39 = 00 */
  /* #365: @41 = 00 */
  /* #366: @42 = 00 */
  /* #367: @51 = 00 */
  /* #368: @52 = 00 */
  /* #369: @29 = zeros(4x4) */
  casadi_clear(w29, 16);
  /* #370: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #371: @26 = zeros(4x1) */
  casadi_clear(w26, 4);
  /* #372: @53 = 00 */
  /* #373: @54 = 00 */
  /* #374: @55 = 00 */
  /* #375: @12 = ones(13x1,1nz) */
  w12 = 1.;
  /* #376: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @11, NULL, NULL, NULL} = vertsplit(@12) */
  w11 = w12;
  /* #377: @12 = (-@11) */
  w12 = (- w11 );
  /* #378: @36 = horzcat(@53, @54, @55, @12) */
  rr=(&w36);
  *rr++ = w12;
  /* #379: @36 = @36' */
  /* #380: @54 = 00 */
  /* #381: @12 = (-@11) */
  w12 = (- w11 );
  /* #382: @55 = 00 */
  /* #383: @19 = horzcat(@54, @53, @12, @55) */
  rr=(&w19);
  *rr++ = w12;
  /* #384: @19 = @19' */
  /* #385: @56 = 00 */
  /* #386: @12 = horzcat(@55, @11, @53, @56) */
  rr=(&w12);
  *rr++ = w11;
  /* #387: @12 = @12' */
  /* #388: @55 = 00 */
  /* #389: @7 = horzcat(@11, @55, @54, @53) */
  rr=(&w7);
  *rr++ = w11;
  /* #390: @7 = @7' */
  /* #391: @33 = horzcat(@36, @19, @12, @7) */
  rr=w33;
  *rr++ = w36;
  *rr++ = w19;
  *rr++ = w12;
  *rr++ = w7;
  /* #392: @27 = @33' */
  casadi_trans(w33,casadi_s16, w27, casadi_s16, iw);
  /* #393: @26 = mac(@27,@24,@26) */
  casadi_mtimes(w27, casadi_s16, w24, casadi_s4, w26, casadi_s4, w, 0);
  /* #394: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #395: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #396: @31 = @26[1:4] */
  for (rr=w31, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #397: (@23[1:4] += @31) */
  for (rr=w23+1, ss=w31; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #398: @29 = mac(@23,@25,@29) */
  for (i=0, rr=w29; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w25+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #399: @28 = @29' */
  for (i=0, rr=w28, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #400: {@23, @25, @26, @27} = horzsplit(@28) */
  casadi_copy(w28, 4, w23);
  casadi_copy(w28+4, 4, w25);
  casadi_copy(w28+8, 4, w26);
  casadi_copy(w28+12, 4, w27);
  /* #401: @27 = @27' */
  /* #402: {@36, @19, @12, @7} = horzsplit(@27) */
  w36 = w27[0];
  w19 = w27[1];
  w12 = w27[2];
  w7 = w27[3];
  /* #403: @26 = @26' */
  /* #404: {@11, @8, @14, @47} = horzsplit(@26) */
  w11 = w26[0];
  w8 = w26[1];
  w14 = w26[2];
  w47 = w26[3];
  /* #405: @7 = (@7+@14) */
  w7 += w14;
  /* #406: @25 = @25' */
  /* #407: {@14, @3, @46, @9} = horzsplit(@25) */
  w14 = w25[0];
  w3 = w25[1];
  w46 = w25[2];
  w9 = w25[3];
  /* #408: @7 = (@7+@3) */
  w7 += w3;
  /* #409: @23 = @23' */
  /* #410: {@3, @35, @20, @10} = horzsplit(@23) */
  w3 = w23[0];
  w35 = w23[1];
  w20 = w23[2];
  w10 = w23[3];
  /* #411: @7 = (@7+@3) */
  w7 += w3;
  /* #412: @12 = (@12-@47) */
  w12 -= w47;
  /* #413: @12 = (@12+@14) */
  w12 += w14;
  /* #414: @12 = (@12-@35) */
  w12 -= w35;
  /* #415: @11 = (@11-@19) */
  w11 -= w19;
  /* #416: @11 = (@11+@9) */
  w11 += w9;
  /* #417: @11 = (@11-@20) */
  w11 -= w20;
  /* #418: @36 = (@36+@8) */
  w36 += w8;
  /* #419: @36 = (@36-@46) */
  w36 -= w46;
  /* #420: @36 = (@36-@10) */
  w36 -= w10;
  /* #421: @55 = 00 */
  /* #422: @54 = 00 */
  /* #423: @53 = 00 */
  /* #424: @23 = vertcat(@40, @39, @41, @42, @51, @52, @7, @12, @11, @36, @55, @54, @53) */
  rr=w23;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w36;
  /* #425: @25 = @23[:4] */
  for (rr=w25, ss=w23+0; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #426: (@37[21:25] = @25) */
  for (rr=w37+21, ss=w25; rr!=w37+25; rr+=1) *rr = *ss++;
  /* #427: @25 = @23[:4] */
  for (rr=w25, ss=w23+0; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #428: (@37[12:28:4] = @25) */
  for (rr=w37+12, ss=w25; rr!=w37+28; rr+=4) *rr = *ss++;
  /* #429: @57 = @37' */
  casadi_trans(w37,casadi_s17, w57, casadi_s17, iw);
  /* #430: output[2][0] = @57 */
  casadi_copy(w57, 25, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_e_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s18;
    case 1: return casadi_s19;
    case 2: return casadi_s19;
    case 3: return casadi_s18;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s20;
    case 1: return casadi_s18;
    case 2: return casadi_s17;
    case 3: return casadi_s19;
    case 4: return casadi_s21;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 18;
  if (sz_iw) *sz_iw = 14;
  if (sz_w) *sz_w = 200;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
