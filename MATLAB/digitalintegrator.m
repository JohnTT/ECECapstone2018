clear all
close all
clc

% Backward Rectangular Integrator
int_rect = filt([1],[1,-1]);

% Trapezoidal Integrator
int_trap = 0.5*filt([1, 1],[1, -1]);

% Simpson Integrator
int_simp = (1/3)*filt([1,4,1],[1,0,-1]);

% Double Integral
int_double = int_trap*int_trap;