%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
function plot_shaded(x, y, fstr)
% x: x coordinates
% y: either just one y vector, or 2xN or 3xN matrix of y-data
% fstr: format ('r' or 'b--' etc)
if size(y,1)>size(y,2)
    y=y';
end
 
restore = ~ishold;
if restore
    clf
end

if size(y,1)==1 % just plot one line
    plot(x,y,fstr);
elseif size(y,1)==2 %plot shaded area
    px=[x,fliplr(x)]; % make closed patch
    py=[y(1,:), fliplr(y(2,:))];
    patch(px,py,1,'FaceColor',fstr,'EdgeColor','none');
elseif size(y,1)==3 % also draw mean
    px=[x,fliplr(x)];
    py=[y(1,:), fliplr(y(3,:))];
    patch(px,py,1,'FaceColor',fstr,'EdgeColor','none');
    hold on
    plot(x,y(2,:),'--', 'Color', fstr);
    if restore
        hold off
    end
end
 
alpha(.2); % make patch transparent