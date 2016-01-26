function [x,y] = createCalcPoints(coords,carWpx, carLpx)
    % Starting with the second WayPoint 
    for p = 2:1:length(coords)
        if(coords(1,p)-coords(1,p-1)==0)
            angle = 0;
        elseif (coords(1,p)-coords(1,p-1)<0)
            angle = atan((coords(2,p)-coords(2,p-1))/(coords(1,p)-coords(1,p-1))) + (pi/2);
        else
            angle = atan((coords(2,p)-coords(2,p-1))/(coords(1,p)-coords(1,p-1))) - (pi/2);
        end
        [bx,by] = createBox(coords(1,p),coords(2,p),angle,carWpx,carLpx);
        
        % Show the rectagles which serve for the calculation
        %hold on
        %plot ([bx],[by])
        %hold off
        
        for i = 1:1:4
            pos = (p-2)*4+i;
            x(pos) = fix(bx(i)+0.5);
            y(pos) = fix(by(i)+0.5);
        end
    end
end
