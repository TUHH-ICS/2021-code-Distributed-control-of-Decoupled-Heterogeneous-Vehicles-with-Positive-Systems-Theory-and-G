%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script simulates a platooning scenario
%% Initialization
rng(5)
% Simulate trajectories
time_steps=100;
beta=100;       % impulse disturbance amplitude

n=size(G{1}.A,1);
m=size(G{1}.B,2);
r=size(G{1}.C,1);

x=zeros(N*n,time_steps);
y=zeros(N*r,time_steps);


%% Time stepping
for i=2:time_steps+1    
    for agent=1:N
        [A_cl,B_cl,C_cl,D_cl]=ssdata(G{agent});
        id_x=(agent-1)*n;
        id_y=(agent-1)*r;
        id_u=(agent-1)*m;
        
        % With disturbance
        w_dist=0;
        if rand()<0.05
            w_dist=beta*(-1+2*rand());
        end        
        input_stack=[0;0;w_dist];
        
        [x(id_x+1:id_x+n,i),y(id_y+1:id_y+r,i-1)] = ss_time_increment(A_cl,B_cl,C_cl,x(id_x+1:id_x+n,i-1),input_stack);
    end    
end
%% Go from Error dynamics to actual trajectories
% Initial conditions
y_ic=10*(-N/2+(1:N)');
y_bar=y_ic+delta_t*(1:time_steps);
y_actual=y+y_bar;

%% Plot the trajectories of the simplified systemn and actual system

bounds=gamma_w*beta;
colors = { 'r', 'g', 'b','c','m','y'};
limits = @(x,j) [x-bounds(j); x; x+bounds(j)];
t=(1:time_steps);

figure(8)
clf
for i = 1:N
    plot(t, y_actual(i,:), 'Color', colors{i})
    leg{i}=['m=',num2str(mass(i)),', b=',num2str(b(i))];
    hold on    
end
for i = 1:N
    plot_shaded(t, limits(y_bar(i,:),i), colors{i})
    hold on
end
hold off

xlim([1 time_steps])
grid('on')
xlabel('time (k)')
ylabel('x(k)')
legend(leg)


