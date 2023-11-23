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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_e_fun_jac_ ## ID
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
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* f_error:(i0[4])->(o0[4]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cs;
  casadi_real w0, *w1=w+1, w2, *w3=w+6;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = input[0][0] */
  casadi_copy(arg[0], 4, w1);
  /* #2: @2 = @1[0] */
  for (rr=(&w2), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #3: @0 = (@0<=@2) */
  w0  = (w0<=w2);
  /* #4: @3 = (@0?@1:0) */
  for (i=0, rr=w3, cs=w1; i<4; ++i) (*rr++)  = (w0?(*cs++):0);
  /* #5: @0 = (!@0) */
  w0 = (! w0 );
  /* #6: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) *rr++ = (- *cs++ );
  /* #7: @1 = (@0?@1:0) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) (*rr++)  = (w0?(*cs++):0);
  /* #8: @3 = (@3+@1) */
  for (i=0, rr=w3, cs=w1; i<4; ++i) (*rr++) += (*cs++);
  /* #9: output[0][0] = @3 */
  casadi_copy(w3, 4, res[0]);
  return 0;
}

/* adj1_f_error:(i0[4],out_o0[4x1,0nz],adj_o0[4])->(adj_i0[4]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cs;
  casadi_real w0, *w1=w+1, w2, *w3=w+6;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = input[0][0] */
  casadi_copy(arg[0], 4, w1);
  /* #2: @2 = @1[0] */
  for (rr=(&w2), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #3: @0 = (@0<=@2) */
  w0  = (w0<=w2);
  /* #4: @2 = 1 */
  w2 = 1.;
  /* #5: @2 = (@0?@2:0) */
  w2  = (w0?w2:0);
  /* #6: @1 = input[2][0] */
  casadi_copy(arg[2], 4, w1);
  /* #7: @3 = (@2*@1) */
  for (i=0, rr=w3, cs=w1; i<4; ++i) (*rr++)  = (w2*(*cs++));
  /* #8: @0 = (!@0) */
  w0 = (! w0 );
  /* #9: @2 = 1 */
  w2 = 1.;
  /* #10: @0 = (@0?@2:0) */
  w0  = (w0?w2:0);
  /* #11: @1 = (@0*@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) (*rr++)  = (w0*(*cs++));
  /* #12: @3 = (@3-@1) */
  for (i=0, rr=w3, cs=w1; i<4; ++i) (*rr++) -= (*cs++);
  /* #13: output[0][0] = @3 */
  casadi_copy(w3, 4, res[0]);
  return 0;
}

/* quadrotor_cost_ext_cost_e_fun_jac:(i0[13],i1[],i2[],i3[13])->(o0,o1[13]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+2, *rr, *ss, *tt;
  const casadi_real **arg1=arg+4, *cs;
  casadi_real w0, *w1=w+11, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, *w15=w+27, *w16=w+40, w17, w18, w19, w20, *w21=w+47, *w22=w+50, *w23=w+59, *w24=w+63, *w25=w+67, *w26=w+71, *w27=w+75, *w28=w+79, *w29=w+95, *w30=w+111, *w31=w+114, *w32=w+117, w34, w35, w36;
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
  /* #66: @25 = f_error(@23) */
  arg1[0]=w23;
  res1[0]=w25;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #67: @8 = @25[0] */
  for (rr=(&w8), ss=w25+0; ss!=w25+1; ss+=1) *rr++ = *ss;
  /* #68: @2 = (@2-@8) */
  w2 -= w8;
  /* #69: @8 = 0 */
  w8 = 0.;
  /* #70: @21 = @25[1:4] */
  for (rr=w21, ss=w25+1; ss!=w25+4; ss+=1) *rr++ = *ss;
  /* #71: @30 = @21' */
  casadi_copy(w21, 3, w30);
  /* #72: @31 = @25[1:4] */
  for (rr=w31, ss=w25+1; ss!=w25+4; ss+=1) *rr++ = *ss;
  /* #73: @8 = mac(@30,@31,@8) */
  for (i=0, rr=(&w8); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w30+j, tt=w31+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #74: @2 = (@2+@8) */
  w2 += w8;
  /* #75: @0 = (@0+@2) */
  w0 += w2;
  /* #76: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #77: @15 = zeros(13x1) */
  casadi_clear(w15, 13);
  /* #78: @1 = @1' */
  /* #79: @30 = zeros(1x3) */
  casadi_clear(w30, 3);
  /* #80: @16 = @16' */
  /* #81: @32 = @22' */
  for (i=0, rr=w32, cs=w22; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #82: @30 = mac(@16,@32,@30) */
  for (i=0, rr=w30; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w16+j, tt=w32+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #83: @30 = @30' */
  /* #84: @1 = (@1+@30) */
  for (i=0, rr=w1, cs=w30; i<3; ++i) (*rr++) += (*cs++);
  /* #85: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<3; ++i) *rr++ = (- *cs++ );
  /* #86: (@15[:3] += @1) */
  for (rr=w15+0, ss=w1; rr!=w15+3; rr+=1) *rr += *ss++;
  /* #87: {@0, @2, @8, @9, @10, @11, @17, @14, @13, @12, @3, @4, @5} = vertsplit(@15) */
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
  /* #88: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #89: output[1][1] = @2 */
  if (res[1]) res[1][1] = w2;
  /* #90: output[1][2] = @8 */
  if (res[1]) res[1][2] = w8;
  /* #91: output[1][3] = @9 */
  if (res[1]) res[1][3] = w9;
  /* #92: output[1][4] = @10 */
  if (res[1]) res[1][4] = w10;
  /* #93: output[1][5] = @11 */
  if (res[1]) res[1][5] = w11;
  /* #94: @29 = zeros(4x4) */
  casadi_clear(w29, 16);
  /* #95: @33 = zeros(4x1,0nz) */
  /* #96: @25 = zeros(4x1) */
  casadi_clear(w25, 4);
  /* #97: (@25[1:4] += @21) */
  for (rr=w25+1, ss=w21; rr!=w25+4; rr+=1) *rr += *ss++;
  /* #98: (@25[1:4] += @31) */
  for (rr=w25+1, ss=w31; rr!=w25+4; rr+=1) *rr += *ss++;
  /* #99: @11 = -1 */
  w11 = -1.;
  /* #100: (@25[0] += @11) */
  for (rr=w25+0, ss=(&w11); rr!=w25+1; rr+=1) *rr += *ss++;
  /* #101: @26 = adj1_f_error(@23, @33, @25) */
  arg1[0]=w23;
  arg1[1]=0;
  arg1[2]=w25;
  res1[0]=w26;
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #102: @24 = @24' */
  /* #103: @29 = mac(@26,@24,@29) */
  for (i=0, rr=w29; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w26+j, tt=w24+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #104: @28 = @29' */
  for (i=0, rr=w28, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #105: {@26, @24, @23, @25} = horzsplit(@28) */
  casadi_copy(w28, 4, w26);
  casadi_copy(w28+4, 4, w24);
  casadi_copy(w28+8, 4, w23);
  casadi_copy(w28+12, 4, w25);
  /* #106: @25 = @25' */
  /* #107: {@11, @10, @9, @8} = horzsplit(@25) */
  w11 = w25[0];
  w10 = w25[1];
  w9 = w25[2];
  w8 = w25[3];
  /* #108: @23 = @23' */
  /* #109: {@2, @0, @6, @7} = horzsplit(@23) */
  w2 = w23[0];
  w0 = w23[1];
  w6 = w23[2];
  w7 = w23[3];
  /* #110: @8 = (@8+@6) */
  w8 += w6;
  /* #111: @24 = @24' */
  /* #112: {@6, @18, @19, @20} = horzsplit(@24) */
  w6 = w24[0];
  w18 = w24[1];
  w19 = w24[2];
  w20 = w24[3];
  /* #113: @8 = (@8+@18) */
  w8 += w18;
  /* #114: @26 = @26' */
  /* #115: {@18, @34, @35, @36} = horzsplit(@26) */
  w18 = w26[0];
  w34 = w26[1];
  w35 = w26[2];
  w36 = w26[3];
  /* #116: @8 = (@8+@18) */
  w8 += w18;
  /* #117: @8 = (@8+@17) */
  w8 += w17;
  /* #118: output[1][6] = @8 */
  if (res[1]) res[1][6] = w8;
  /* #119: @9 = (@9-@7) */
  w9 -= w7;
  /* #120: @9 = (@9+@6) */
  w9 += w6;
  /* #121: @9 = (@9-@34) */
  w9 -= w34;
  /* #122: @9 = (@9+@14) */
  w9 += w14;
  /* #123: output[1][7] = @9 */
  if (res[1]) res[1][7] = w9;
  /* #124: @2 = (@2-@10) */
  w2 -= w10;
  /* #125: @2 = (@2+@20) */
  w2 += w20;
  /* #126: @2 = (@2-@35) */
  w2 -= w35;
  /* #127: @2 = (@2+@13) */
  w2 += w13;
  /* #128: output[1][8] = @2 */
  if (res[1]) res[1][8] = w2;
  /* #129: @11 = (@11+@0) */
  w11 += w0;
  /* #130: @11 = (@11-@19) */
  w11 -= w19;
  /* #131: @11 = (@11-@36) */
  w11 -= w36;
  /* #132: @11 = (@11+@12) */
  w11 += w12;
  /* #133: output[1][9] = @11 */
  if (res[1]) res[1][9] = w11;
  /* #134: output[1][10] = @3 */
  if (res[1]) res[1][10] = w3;
  /* #135: output[1][11] = @4 */
  if (res[1]) res[1][11] = w4;
  /* #136: output[1][12] = @5 */
  if (res[1]) res[1][12] = w5;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_e_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 15;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 129;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
