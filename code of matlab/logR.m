function [w,theta] = logR(R)
    % first compute the trace of the matrix
    t = trace(R);
    % according to the trace, there are several cases
    if t >= 3
        % no rotation
        w = [0;0;0];
        theta = 0;
    elseif t <= -1
        % 180 deg
        theta = pi;
        w = [sqrt((R(1,1)+1)/2);sqrt((R(2,2)+1)/2);sqrt((R(3,3)+1)/2)];
    else
        % non-trivial case
        theta = acos((t-1)/2);
        w = [R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/(2*sin(theta));
    end
end