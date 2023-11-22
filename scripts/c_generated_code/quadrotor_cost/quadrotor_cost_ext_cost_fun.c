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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_fun_ ## ID
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
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_fun:(i0[13],i1[4],i2[],i3[13])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, *w1=w+5, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, *w15=w+21, *w16=w+34, w17, w18, w19, w20, *w21=w+41, *w22=w+44, *w23=w+53, *w24=w+57, *w25=w+61, *w26=w+65, *w27=w+81, *w28=w+85, *w29=w+89;
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
  /* #43: @2 = 0.5 */
  w2 = 5.0000000000000000e-01;
  /* #44: @3 = 0 */
  w3 = 0.;
  /* #45: @23 = zeros(1x4) */
  casadi_clear(w23, 4);
  /* #46: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #47: @5 = input[1][1] */
  w5 = arg[1] ? arg[1][1] : 0;
  /* #48: @6 = input[1][2] */
  w6 = arg[1] ? arg[1][2] : 0;
  /* #49: @7 = input[1][3] */
  w7 = arg[1] ? arg[1][3] : 0;
  /* #50: @24 = vertcat(@4, @5, @6, @7) */
  rr=w24;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  /* #51: @25 = @24' */
  casadi_copy(w24, 4, w25);
  /* #52: @26 = zeros(4x4) */
  casadi_clear(w26, 16);
  /* #53: @4 = 0.0505051 */
  w4 = 5.0505050505050504e-02;
  /* #54: (@26[0] = @4) */
  for (rr=w26+0, ss=(&w4); rr!=w26+1; rr+=1) *rr = *ss++;
  /* #55: @4 = 100 */
  w4 = 100.;
  /* #56: (@26[5] = @4) */
  for (rr=w26+5, ss=(&w4); rr!=w26+6; rr+=1) *rr = *ss++;
  /* #57: @4 = 100 */
  w4 = 100.;
  /* #58: (@26[10] = @4) */
  for (rr=w26+10, ss=(&w4); rr!=w26+11; rr+=1) *rr = *ss++;
  /* #59: @4 = 100 */
  w4 = 100.;
  /* #60: (@26[15] = @4) */
  for (rr=w26+15, ss=(&w4); rr!=w26+16; rr+=1) *rr = *ss++;
  /* #61: @23 = mac(@25,@26,@23) */
  for (i=0, rr=w23; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w25+j, tt=w26+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #62: @3 = mac(@23,@24,@3) */
  for (i=0, rr=(&w3); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w23+j, tt=w24+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #63: @2 = (@2*@3) */
  w2 *= w3;
  /* #64: @0 = (@0+@2) */
  w0 += w2;
  /* #65: @2 = 1 */
  w2 = 1.;
  /* #66: @23 = zeros(4x1) */
  casadi_clear(w23, 4);
  /* #67: @3 = (-@13) */
  w3 = (- w13 );
  /* #68: @4 = (-@14) */
  w4 = (- w14 );
  /* #69: @5 = (-@17) */
  w5 = (- w17 );
  /* #70: @24 = horzcat(@12, @3, @4, @5) */
  rr=w24;
  *rr++ = w12;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  /* #71: @24 = @24' */
  /* #72: @3 = (-@17) */
  w3 = (- w17 );
  /* #73: @25 = horzcat(@13, @12, @3, @14) */
  rr=w25;
  *rr++ = w13;
  *rr++ = w12;
  *rr++ = w3;
  *rr++ = w14;
  /* #74: @25 = @25' */
  /* #75: @3 = (-@13) */
  w3 = (- w13 );
  /* #76: @27 = horzcat(@14, @17, @12, @3) */
  rr=w27;
  *rr++ = w14;
  *rr++ = w17;
  *rr++ = w12;
  *rr++ = w3;
  /* #77: @27 = @27' */
  /* #78: @14 = (-@14) */
  w14 = (- w14 );
  /* #79: @28 = horzcat(@17, @14, @13, @12) */
  rr=w28;
  *rr++ = w17;
  *rr++ = w14;
  *rr++ = w13;
  *rr++ = w12;
  /* #80: @28 = @28' */
  /* #81: @26 = horzcat(@24, @25, @27, @28) */
  rr=w26;
  for (i=0, cs=w24; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w25; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w27; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  /* #82: @29 = @26' */
  for (i=0, rr=w29, cs=w26; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #83: @9 = (-@9) */
  w9 = (- w9 );
  /* #84: @10 = (-@10) */
  w10 = (- w10 );
  /* #85: @11 = (-@11) */
  w11 = (- w11 );
  /* #86: @24 = vertcat(@8, @9, @10, @11) */
  rr=w24;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #87: @23 = mac(@29,@24,@23) */
  for (i=0, rr=w23; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w29+j, tt=w24+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #88: @8 = @23[0] */
  for (rr=(&w8), ss=w23+0; ss!=w23+1; ss+=1) *rr++ = *ss;
  /* #89: @2 = (@2-@8) */
  w2 -= w8;
  /* #90: @2 = sq(@2) */
  w2 = casadi_sq( w2 );
  /* #91: @8 = 0 */
  w8 = 0.;
  /* #92: @1 = @23[1:4] */
  for (rr=w1, ss=w23+1; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #93: @1 = @1' */
  /* #94: @16 = @23[1:4] */
  for (rr=w16, ss=w23+1; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #95: @8 = mac(@1,@16,@8) */
  for (i=0, rr=(&w8); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w16+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #96: @2 = (@2+@8) */
  w2 += w8;
  /* #97: @0 = (@0+@2) */
  w0 += w2;
  /* #98: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 105;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
