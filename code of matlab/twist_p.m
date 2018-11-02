function out=twist_p(omega,v,theta)

%calculate the twist coordinate of the transformation
%input para:(omega,q,theta)
%omega:the axis that rotate around
%q:a dot on the rotating axis 
%theta:the angle of the rotation.rad

if rank(omega)~=0
    out = (eye(3)-e_ro(omega,theta)) * cross(omega,v) + (omega*omega'*v*theta);
else
    out=v*theta;
end