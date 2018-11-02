function out=hat(input)

% calculate the hat of a vector
% input should be a vector of 3x1

if length(input) ~= 3
    error('not a 3x1 vector')
end
out=[      0           -input(3,1)      input(2,1);
      input(3,1)           0            -input(1,1);
     -input(2,1)       input(1,1)           0   ];