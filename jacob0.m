function J = jacob0(S,q)
%Takes a twist and joint configuration as inputs and outputs the jacobian
    s=size(S); %To account for different DOFs
    A=eye(4);
    for i = 1:(s(2)-1) % ith row of J is calculated using adjoint
        A=A*twist2ht(S(1:6,i),q(i));
        J(i+1,:)=adjoint(S(1:6,i+1),A)'; %Assignment starts from second row
    end
    J(1,:)=S(1:6,1); %First row is assigned
    J=J';   %Transposed to get the jacobian
end