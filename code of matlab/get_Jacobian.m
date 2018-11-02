function out = get_Jacobian(epsi_all,exp_1,exp_2,exp_3,exp_4,exp_5,exp_6)

%function get Jacobian for inverse kinematic
%input para: epsi_all a vector of all epsi
%input para:exp_all :vector of all exp

out=[epsi_all(:,1)    Adg(exp_1)*epsi_all(:,2)    Adg(exp_1*exp_2)*epsi_all(:,3)    Adg(exp_1*exp_2*exp_3)*epsi_all(:,4)    Adg(exp_1*exp_2*exp_3*exp_4)*epsi_all(:,5)    Adg(exp_1*exp_2*exp_3*exp_4*exp_5)*epsi_all(:,6)];