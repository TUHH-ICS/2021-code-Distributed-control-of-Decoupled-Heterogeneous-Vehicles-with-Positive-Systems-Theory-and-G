%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function synthesizes the diagonal weight matrix for the first order protocol  
function [P,E,status]= synthesis_exp_stability_LMI(L_c,alpha_sqr,tol)

n=size(L_c,2);
cvx_begin sdp
variable P(n,n) diagonal
variable X(n,n) diagonal
cvx_precision high

LMI_1_11=alpha_sqr*P;
LMI_1_21=P-X*L_c;
LMI_1_22=P;


LMI_1=[ LMI_1_11, LMI_1_21';
        LMI_1_21, LMI_1_22];


minimize 1

subject to:
LMI_1>= tol*eye(2*n); % Note that this condition includes P>0

cvx_end
status=cvx_status; 
%% For debugging: Check if the found P indeed satisfies the LMI
% eig(LMI_1);
% eig(P);
%% Return optimal E
E=P\X;
end