function S_body = space2body(S,M,q)
%Converts the Screw axis in the space frame to the body frame.
    T=fkine(S,M,q,"space");
    R=T(1:3,1:3);
    P=T(1:3,4);
    Pb=[0,-P(3),P(2);P(3),0,-P(1);-P(2),P(1),0];
    o=[0,0,0;0,0,0;0,0,0];
    A=[R,o;Pb*R,R];
    S_body=pinv(A)*S;
end