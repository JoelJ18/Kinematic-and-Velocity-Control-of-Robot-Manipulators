function T = fkine(S,M,q,frame)
%Finds the forward kinematics using product of exponents
%Takes the S matrix contatining the screw axis for all the joints, the M
%matrix that represents the home configuration and the joint configurations
%as inputs.
%If the frame specifies space it denotes the screw axis is with respect to 
% the space frame so it postmultiplies M, 
%else it premultiplies M as the screw axis being passed in this case
% are with respect to the body frame.
    s=size(S);
    T=eye(4);
    for i = 1:s(2)
        e=twist2ht(S(:,i),q(i));
        T=T*e; 
    end
    
    if(frame=="space")
        T=T*M;
    else
        T=M*T;
    end
end