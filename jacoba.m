function J_a = jacoba(S,M,q)
%Finds the analyitcal jacobian by taking the screw axis in the space frame
%and finding the body jacobian.
    s=size(S);
    A=eye(4);
    for i = 1:(s(2)-1)
            A=A*twist2ht(S(1:6,i),q(i));
            J(i+1,:)=adjoint(S(1:6,i+1),A)';
       
    end
       
        J(1,:)=S(1:6,1);
        J=J';
        
        T=eye(4);
        for i = 1:s(2)
            e=twist2ht(S(:,i),q(i));
            T=T*e; 
        end
        T=T*M;
        
        R=T(1:3,1:3);
        P=T(1:3,4);
        Pb=[0,-P(3),P(2);P(3),0,-P(1);-P(2),P(1),0];
        o=[0,0,0;0,0,0;0,0,0];
        A=[R,o;Pb*R,R];
        J_b=pinv(A)*J;
        Jvb=J_b(4:6,:);
        J_a=R*Jvb;
end