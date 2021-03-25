%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function initializes a random Laplacian matrix
function [L,D]= init_info_flow_dynamics(N,link_prob,topo)    
    A=gen_topology(N,link_prob,topo);       % Generate the Adjacency for a topology
    d=A*ones(N,1);                          % vector of node degrees
    D=diag(d);                              % Degree Matrix
    L=D-A;                                  % Construct Laplacian
end