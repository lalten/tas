function y = pathcreator()

minRadius   = 0.74;
length      = 3.00;
samples     = 7;    %Kurven
samples2    = 50;    %Boxen
carWidth    = 0.45;
carLength   = 0.70;
tolerance   = 0.07;

deg1 = 0:pi/180:pi/4;           % von 0° bis 45°
deg2 = (pi+pi/4):-pi/180:pi;    % von 225° bis 180°

x = [cos(deg1) - 1  ,   cos(deg2) + 1 - 2*(1-cos(pi/4)) ];  % linke halb-Kurve
y = [sin(deg1)      ,   sin(deg2) + 2*sin(pi/4)         ];  % rechte halb-Kurve

yS = 0:0.01:length;             % geradeaus
xS = 0*yS;                      % geradeaus

figure
plot(xS,yS);     % geradeaus

% Samples auf Geraden
for yS = 0:length/(samples2-1):length
    hold on
    [bx,by] = createBox(0,yS,0,carWidth,carLength);
    plot([bx],[by])
    hold off
end


maxRadius = length/(2*sin(pi/4));          % MaxRadius, so that the car looks ahead aften length 
distanceMinMax = maxRadius - minRadius;    % Differenz zwisch min und max Radius

for factor = minRadius : distanceMinMax/((samples-3)/2) : maxRadius
    hold on

    % Drive curves with given radius
    x1 = factor * x;
    y1 = factor * y;
    
    % Drive straight on untill length
    for y2 = 2*sin(pi/4)*factor : 0.01 : length
        x2 = -2*factor*(1-cos(pi/4));
    end
    
    plot([x1,x2],[y1,y2],[-x1,-x2],[y1,y2])
    hold off 
    
    % Length of the path
    circle45_length = factor*pi/4;                               % length of 45° of the circle
    path_length = length-2*factor*sin(pi/4)+2*circle45_length;   % total length
    
    for current_length = 0 :  (path_length/(samples2-1)) : path_length
        if current_length < circle45_length
            a = current_length/factor;
            sx = (cos(a)-1)*factor;
            sy = sin(a)*factor;
            [bx,by] = createBox(sx,sy,a,carWidth,carLength);
        else
            if current_length < 2 *circle45_length
                a = (current_length-circle45_length)/factor;
                offsety = sin(pi/4)*factor;
                deg2= 5*pi/4-a;
                sx = (cos(deg2) + 1 - 2*(1-cos(pi/4)))*factor;
                sy = 2*offsety + sin(a-pi/4)*factor;
                [bx,by] = createBox(sx,sy,pi/4-a,carWidth,carLength);
            else current_length > 2*circle45_length
                sx = -2*factor*(1-cos(pi/4));
                sy = (current_length - 2*circle45_length) + 2*factor*sin(pi/4);
                [bx,by] = createBox(sx,sy,0,carWidth,carLength);
            end
        end
        
        hold on
        plot([bx],[by],-[bx],[by])
        %scatter(sx,sy)
        hold off
    end            
    
end

y=0;
end