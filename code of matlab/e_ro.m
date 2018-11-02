function out=e_ro(omega,theta)

%calculate the exponatial coordinate of a rotation 
%input:      omega:the axis which rotate
%            theta    :the angle (scalar:rad)

out = eye(3)+hat(omega)*sin(theta)+hat(omega)^2*(1-cos(theta));