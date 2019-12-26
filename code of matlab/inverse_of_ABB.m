function theta_vector = inverse_of_ABB(g_st_theta)
%% params of inverse kinematic 
% tolerance of error
E=1e-5;
limit_of_iterate=20;
%% parameters of ABB elbow manipulator
% initial posture
g_st0 = [1  0  0   533  ;
         0  0  1   0    ;
         0 -1  0   889.1;
         0  0  0   1    ];
% rotation axis
omega = [0  0  0  1  0  1;
         0  1  1  0  1  0;
         1  0  0  0  0  0];
% point passed through by the axis     
q = [0   0      0      0     451    533;
     0   0      0      0      0      0 ;
     0  399.1  847.1  889.1  889.1  889.1];
%initial angle of each joint
theta_vector = zeros(6,1);
%get v
v = [cross(-omega(:,1),q(:,1)),cross(-omega(:,2),q(:,2)),cross(-omega(:,3),q(:,3)),cross(-omega(:,4),q(:,4)),cross(-omega(:,5),q(:,5)),cross(-omega(:,6),q(:,6))];
epsi1=make_epsi(v(:,1),omega(:,1));
epsi2=make_epsi(v(:,2),omega(:,2));
epsi3=make_epsi(v(:,3),omega(:,3));
epsi4=make_epsi(v(:,4),omega(:,4));
epsi5=make_epsi(v(:,5),omega(:,5));
epsi6=make_epsi(v(:,6),omega(:,6));
%% Newton's method            
%f(x)=exp_1*exp_2*exp_3*exp_4*exp_5*exp_6*g_st(0)*g^-1
figure('name','inverse kinematic');                                                                 %make a figure                                                     %the first pic of four
subplot(1,2,1)
for t=1:100
    %  make R for six axis 
    R1 = e_ro(omega(:,1),theta_vector(1));
    R2 = e_ro(omega(:,2),theta_vector(2));
    R3 = e_ro(omega(:,3),theta_vector(3));
    R4 = e_ro(omega(:,4),theta_vector(4));
    R5 = e_ro(omega(:,5),theta_vector(5));
    R6 = e_ro(omega(:,6),theta_vector(6));
    %  make P vector for six axis
    v = [cross(-omega(:,1),q(:,1)),cross(-omega(:,2),q(:,2)),cross(-omega(:,3),q(:,3)),cross(-omega(:,4),q(:,4)),cross(-omega(:,5),q(:,5)),cross(-omega(:,6),q(:,6))]
    P = [twist_p(omega(:,1),v(:,1),theta_vector(1)),twist_p(omega(:,2),v(:,2),theta_vector(2)),twist_p(omega(:,3),v(:,3),theta_vector(3)),twist_p(omega(:,4),v(:,4),theta_vector(4)),twist_p(omega(:,5),v(:,5),theta_vector(5)),twist_p(omega(:,6),v(:,6),theta_vector(6))]
    %  make exponatial for six axis
    exp_1 = make_exp(R1,P(:,1));
    exp_2 = make_exp(R2,P(:,2));
    exp_3 = make_exp(R3,P(:,3));
    exp_4 = make_exp(R4,P(:,4));
    exp_5 = make_exp(R5,P(:,5));
    exp_6 = make_exp(R6,P(:,6));
    %get phi(theta_k)
    epsi_thk=vee(logm(exp_1*exp_2*exp_3*exp_4*exp_5*exp_6*g_st0/g_st_theta));
    %get Jacobian
    Jacobian=get_Jacobian([epsi1 epsi2 epsi3 epsi4 epsi5 epsi6],exp_1,exp_2,exp_3,exp_4,exp_5,exp_6);
    %get pseudo Jacobian
    ps_Jacobian=pinv(Jacobian);
    %get theta_k+1
    theta_vector=theta_vector-ps_Jacobian*epsi_thk;
    Norm_of_phi=norm(epsi_thk);
    %display the graph
    stem(t,Norm_of_phi);
    hold on
    %calculate
    if Norm_of_phi<E
        break;
    end
end

title('Norm of phi');
disp 'µü´ú´ÎÊý:'
t
if t>=limit_of_iterate
    error( 'failed to literate or pos. out of range');
end
Norm_of_phi

%% forward kinematic again 
R1 = e_ro(omega(:,1),theta_vector(1));
R2 = e_ro(omega(:,2),theta_vector(2));
R3 = e_ro(omega(:,3),theta_vector(3));
R4 = e_ro(omega(:,4),theta_vector(4));
R5 = e_ro(omega(:,5),theta_vector(5));
R6 = e_ro(omega(:,6),theta_vector(6));

v = [cross(-omega(:,1),q(:,1)),cross(-omega(:,2),q(:,2)),cross(-omega(:,3),q(:,3)),cross(-omega(:,4),q(:,4)),cross(-omega(:,5),q(:,5)),cross(-omega(:,6),q(:,6))];
    
P = [twist_p(omega(:,1),v(:,1),theta_vector(1)),twist_p(omega(:,2),v(:,2),theta_vector(2)),twist_p(omega(:,3),v(:,3),theta_vector(3)),twist_p(omega(:,4),v(:,4),theta_vector(4)),twist_p(omega(:,5),v(:,5),theta_vector(5)),twist_p(omega(:,6),v(:,6),theta_vector(6))];

exp_1 = make_exp(R1,P(:,1));
exp_2 = make_exp(R2,P(:,2));
exp_3 = make_exp(R3,P(:,3));
exp_4 = make_exp(R4,P(:,4));
exp_5 = make_exp(R5,P(:,5));
exp_6 = make_exp(R6,P(:,6));

g_st_theta = exp_1*exp_2*exp_3*exp_4*exp_5*exp_6*g_st0;
%% display
%figure
point_a = [0 0 0 1]';
point_b = exp_1*[0 0 399.1 1]';
point_c = exp_1*exp_2*exp_3*[0 0 847.1 1]';
point_d = exp_1*exp_2*exp_3*[0 0 889.1 1]';
point_e = exp_1*exp_2*exp_3*exp_4*exp_5*[451 0 889.1 1]';
point_f = g_st_theta(:,4);
x=[point_a(1,1) point_b(1,1) point_c(1,1) point_d(1,1) point_e(1,1) point_f(1,1)];
y=[point_a(2,1) point_b(2,1) point_c(2,1) point_d(2,1) point_e(2,1) point_f(2,1)];
z=[point_a(3,1) point_b(3,1) point_c(3,1) point_d(3,1) point_e(3,1) point_f(3,1)];
%draw the 3d plot
subplot(1,2,2)
plot3(x,y,z,'-bo','Linewidth',2);
%axis equal
axis([-1000 1000 -1000 1000 0 1500])
grid on


