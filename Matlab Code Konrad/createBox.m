function [x,y] = createBox(px,py,angle,carWidth,carLength)   
    p1x = -carWidth/2;
    p1y = 0;
    
    p2x = p1x;
    p2y = -carLength;
    
    p3x = carWidth/2;
    p3y = p2y;
    
    p4x = p3x;
    p4y = 0;
    
    x = [p1x*cos(angle)-p1y*sin(angle), ...
        p2x*cos(angle)-p2y*sin(angle), ...
        p3x*cos(angle)-p3y*sin(angle), ...
        p4x*cos(angle)-p4y*sin(angle), ...
        p1x*cos(angle)-p1y*sin(angle)];
    
    y = [p1x*sin(angle)+p1y*cos(angle), ...
        p2x*sin(angle)+p2y*cos(angle), ...
        p3x*sin(angle)+p3y*cos(angle), ...
        p4x*sin(angle)+p4y*cos(angle), ...
        p1x*sin(angle)+p1y*cos(angle)];
    
    x = x + px;
    y = y + py;
    
end
