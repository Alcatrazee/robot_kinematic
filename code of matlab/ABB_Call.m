%% comment
% To alternate the result, we just need to change the value of joint angle 
% located in line 9, by the order of theta 1 to theta 6
% disavantage:sometimes the inverse kinematic gets the same result(angle)
% as the input ,sometimes not ,but it doesn't affect the final posture of
% the tool frame.Because the constrains of joint angle is not given,I can 
% not restrict the input.
clear 
clc
close all
%% forward kinematic 
theta_vector = [deg2rad(00) deg2rad(30) deg2rad(50) deg2rad(0) deg2rad(0) deg2rad(0)];      %input angle of each joint here 
g_st_theta = forward_of_ABB(theta_vector)                                                     %get final coordinate of final posture
%% inverse kinematic,will input the posture generated by forward kinematic
theta = inverse_of_ABB(g_st_theta);
theta = theta';
%% display
disp 'result of inverse kinematic'
theta                                                                                          %angle from inverse kinematic
disp 'input angle vector'
theta_vector                                                                                   %expected angle 
