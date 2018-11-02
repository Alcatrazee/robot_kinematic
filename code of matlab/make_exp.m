function out = make_exp(R,P)

%usage:to make a exponantial presentation of a transformation
%input Rotation matrix and a pos


if length(R)~=3
   error('Rotation matrix is not 3x3 matrix'); 
end

if length(P)~=3
   error('Rotation matrix is not a 3x1 vector'); 
end

out=[R(1,:) P(1,1);
     R(2,:) P(2,1);
     R(3,:) P(3,1);
     0 0 0     1 ];