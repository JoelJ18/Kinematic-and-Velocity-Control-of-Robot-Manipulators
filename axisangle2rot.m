function R = axisangle2rot(omega,theta)
%Takes omega and theta as inputs and finds the rotational matrix associated
%wiht it
w=skew(omega);
R=expm(w*theta);
end