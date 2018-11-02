function g_st_theta = forward_of_ABB(theta_vector)

%   calculate the forward kinematic of ABB elbow manipulator
%   input vecotr :theta_vector is the vector in forms of 
%                 [theta1 theta2 theta3 theta4 theta5 theta6]
%   requirements: function:e_ro(),twist_p(),hat(),make_exp(),make_epsi().

%% parameters of ABB elbow manipulator
g_st0 = [1 0 0 533;
         0 0 1  0;
         0 -1 0 889.1;
         0 0 0 1];
     
omega = [0  0  0 1  0 1;
         0  1  1 0  1 0;
         1  0  0 0  0 0];
     
q = [0   0   0   0  451  533;
     0   0   0   0   0    0 ;
     0 399.1 847.1  889.1 889.1 889.1];
%% forward kinematic
R1 = e_ro(omega(:,1),theta_vector(1));
R2 = e_ro(omega(:,2),theta_vector(2));
R3 = e_ro(omega(:,3),theta_vector(3));
R4 = e_ro(omega(:,4),theta_vector(4));
R5 = e_ro(omega(:,5),theta_vector(5));
R6 = e_ro(omega(:,6),theta_vector(6));

v = [cross(-omega(:,1),q(:,1)),cross(-omega(:,2),q(:,2)),cross(-omega(:,3),q(:,3)),cross(-omega(:,4),q(:,4)),cross(-omega(:,5),q(:,5)),cross(-omega(:,6),q(:,6))];
twist_vector = [make_epsi(v(:,1),omega(:,1)),make_epsi(v(:,2),omega(:,2)),make_epsi(v(:,3),omega(:,3)),make_epsi(v(:,4),omega(:,4)),make_epsi(v(:,5),omega(:,5)),make_epsi(v(:,6),omega(:,6)) ];
P = [twist_p(omega(:,1),v(:,1),theta_vector(1)),twist_p(omega(:,2),v(:,2),theta_vector(2)),twist_p(omega(:,3),v(:,3),theta_vector(3)),twist_p(omega(:,4),v(:,4),theta_vector(4)),twist_p(omega(:,5),v(:,5),theta_vector(5)),twist_p(omega(:,6),v(:,6),theta_vector(6))];

exp_1 = make_exp(R1,P(:,1));
exp_2 = make_exp(R2,P(:,2));
exp_3 = make_exp(R3,P(:,3));
exp_4 = make_exp(R4,P(:,4));
exp_5 = make_exp(R5,P(:,5));
exp_6 = make_exp(R6,P(:,6));

g_st_theta = exp_1*exp_2*exp_3*exp_4*exp_5*exp_6*g_st0;                    %initial state
%% display
figure('name','forward kinematic')
SA_point_a = [0 0 0 1]'
SA_point_b = exp_1*[0 0 399.1 1]'
SA_point_c = exp_1*exp_2*exp_3*[0 0 847.1 1]'
SA_point_d = exp_1*exp_2*exp_3*[0 0 889.1 1]'
SA_point_e = exp_1*exp_2*exp_3*exp_4*exp_5*[451 0 889.1 1]'
SA_point_f = g_st_theta(:,4)
x=[SA_point_a(1,1) SA_point_b(1,1) SA_point_c(1,1) SA_point_d(1,1) SA_point_e(1,1) SA_point_f(1,1)];
y=[SA_point_a(2,1) SA_point_b(2,1) SA_point_c(2,1) SA_point_d(2,1) SA_point_e(2,1) SA_point_f(2,1)];
z=[SA_point_a(3,1) SA_point_b(3,1) SA_point_c(3,1) SA_point_d(3,1) SA_point_e(3,1) SA_point_f(3,1)];
%draw the 3d plot
plot3(x,y,z,'-bo','Linewidth',2);
axis([-1000 1000 -1000 1000 0 1500])
grid on




