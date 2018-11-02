function out=vee(se3)

%function vee 
%input a se(3) matrix 
%output a twist

omega=se3(1:3,1:3);
v=se3(1:3,4);
out=[v' omega(3,2) omega(1,3) omega(2,1) ]';

