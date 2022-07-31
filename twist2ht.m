function T = twist2ht(S,theta)
%Takes a twist and theta and gives the Homogenous Transformation Matrix
%Associated with it
    omega=(S(1:3))';
    v=(S(4:6))';
    w=[0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
    I=eye(3);
    R=expm(w*theta);
    m=((I*theta) +((1-cos(theta))*w)+((theta-sin(theta))*(w*w)))*v';
    T=[R,m;0,0,0,1];
end