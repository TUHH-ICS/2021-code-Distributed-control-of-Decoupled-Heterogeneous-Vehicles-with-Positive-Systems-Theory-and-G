%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function increments the state and produces the output of a 
% discrete-time dynamical system for a given input  
function [x_next,y_curr] = ss_time_increment(A,B,C,x_curr,r_curr)
x_next = A*x_curr+ B*r_curr;
y_curr = C*x_curr;
end

