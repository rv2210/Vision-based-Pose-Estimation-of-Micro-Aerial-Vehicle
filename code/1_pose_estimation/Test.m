clc; clear all; close all;
%x=[1 23 3 3 3 2 2 2 2;2 2 2 2 2 2 2 2 2;2 3 2 2 2 2 2 2 2;2 4 2 2 2 2 2 2 2;2 3 2 2 2 2 2 2 2; 2 2 2 2 2 2 2 2 2; 2 2 2 2 2 2 2 2 2; 2 2 2 2 2 2 2 2 2];
%y= 1:length(x);
x=[4 3 3 2 3 2 3 2 32 3 2 3 2 3 2 3 2 32 -3 ]
x= -x
z= horzcat(x);
a=rem(2,2);
lasm= [311.0520    0                 201.8727;
                    0           311.3885          113.6210;
                    0           0                 1];
a=rotz(45,'deg')*rotx(180,'deg')


cam_intrinsic= [311.0520    0                 201.8724;
                    0           311.3885          113.6210;
                    0           0                 1];
asd=inv(cam_intrinsic)