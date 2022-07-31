function Vb = adjoint(Va,T)
%Takes the twist seen from frame A and The Homogeneous Transformation
%Matrix from frame A to frame B as inputs and outputs the twist seen from
%frame B
    R=T(1:3,1:3);
    p=T(1:3,4);
    Tinv=[R',-R'*p;0,0,0,1];
    w=[0,-Va(3),Va(2);Va(3),0,-Va(1);-Va(2),Va(1),0];
    bV=[w,Va(4:6);0,0,0,0];
    q=T*bV*Tinv;
    Vb=[q(3,2),q(1,3),-q(1,2),q(1,4),q(2,4),q(3,4)]';
end

