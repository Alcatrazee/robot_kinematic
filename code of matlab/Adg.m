function out=Adg(exp)

%calculate the adjoint of a transformation
%exp:exponential presentation of a transformation

out=[exp(1:3,1:3) hat(exp(1:3,4))*exp(1:3,1:3);
    zeros([3 3]) exp(1:3,1:3)];
