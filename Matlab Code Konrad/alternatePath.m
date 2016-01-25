function ans = alternatePath(A, hight, width, resolution, minRadius)

figure 
%mp = [hight/2, width/2];
mp = [100,100];

% Convert Image Data
for i = 1 : 1 : hight
    for j = 1 : 1 : width
        B(j,i) = A(((i-1)*200)+j);
        showB(i,j) = B(j,i);
    end
end

% Plot image
hold on
% image needs to be shown inverted
image(showB)
hold off

% Plot car
carWidth    = 0.45; % on y-axis (in m)
carLength   = 0.70; % on x-axis (in m)
carPoint1 = [mp(1) + carWidth/(2*resolution), ...
    mp(2)- carLength/(2*resolution)];       % upper left
carPoint2 = [mp(1) + carWidth/(2*resolution), ...
    mp(2)+ carLength/(2*resolution)];       % upper right
carPoint3 = [mp(1) - carWidth/(2*resolution), ...
    mp(2)+ carLength/(2*resolution)];       % lower right
carPoint4 = [mp(1) - carWidth/(2*resolution), ...
    mp(2)- carLength/(2*resolution)];       % lower left

%hold on
%plot ([carPoint1(1),carPoint2(1),carPoint3(1),carPoint4(1),carPoint1(1)], ...
 %   [carPoint1(2),carPoint2(2),carPoint3(2),carPoint4(2),carPoint1(2)])
%hold off

% Way Points
pt1 = [0;-0.5]/resolution + [mp(1);mp(2)];
pt4 = [0;3.5]/resolution + [mp(1);mp(2)];
waypointsX = pt1(1):5:pt4(1);
waypointsY = pt1(2):5:pt4(2);

cost = 0;
for yWP = waypointsY
    [bx,by] = createBox(mp(1),yWP,0,carWidth/resolution,carLength/resolution);
    for i = 1:1:4
        cx = fix(bx(i)+0.5);
        cy = fix(by(i)+0.5);
        newcost = B(cx, cy);
        cost = cost + newcost;
    end
    %hold on
    %plot ([bx],[by])
    %hold off
end
ans = cost

hold on
plot(waypointsX(1), waypointsY(1),'*')
hold off
hold on
plot(waypointsX(end), waypointsY(end),'*')
hold off

if (cost > 100)
    display ('needs to calculate alternative Path')
    
    % On alternate Path
    yMiddleOfPath = pt1(2) + ((pt4(2) - pt1(2) ) / 2)
    xC = 130;
    t = linspace(0,1,200);
    % OLD
    %pts = kron((1-t).^2,pt1) + kron(2*(1-t).*t,[xC;yMiddleOfPath])+ kron(t.^2,pt4);
    
    % NEW
    newpt2 = pt1 + (1/4)*(pt4-pt1);
    newpt3 = pt1 + (3/4)*(pt4-pt1);
    pts = kron((1-t).^3,pt1) + kron(3*(1-t).^2.*t,newpt2) + kron(3*(1-t).*t.^2,[xC;yMiddleOfPath]) + kron(t.^3,pt4);
    %newnewpt2 = pt1 + (1/8)*(pt4-pt1);
    %newnewpt3 = pt1 + (3/8)*(pt4-pt1);
    %newnewpt4 = pt1 + (5/8)*(pt4-pt1);  
    %newnewpt5 = pt1 + (7/8)*(pt4-pt1);
    %newpts1 = kron((1-t).^3,pt1) + kron(3*(1-t).^2.*t,newnewpt2) + kron(3*(1-t).*t.^2,newnewpt3) + kron(t.^3,[xC;yMiddleOfPath]);
    %newpts2 = kron((1-t).^3,[xC;yMiddleOfPath]) + kron(3*(1-t).^2.*t,newnewpt4) + kron(3*(1-t).*t.^2,newnewpt5) + kron(t.^3,pt4);
    %pts = [newpts1 newpts2]
    
    hold on
    plot(pt1(1),pt1(2),'*')
    hold off
    hold on
    plot(newpt2(1),newpt2(2),'*')
    hold off
    hold on
    plot(xC,yMiddleOfPath,'*')
    hold off
    
    

    angle = (pi/2) - sin((pts(2,2)-pts(2,1))/(pts(1,2)-pts(1,1)));
    angleDeg = angle*180/pi
    
    pathcost = 0;
    for i = 1:12:200
        angle = - tan((pts(1,i+5)-pts(1,i))/(pts(2,i+5)-pts(2,i)));
        [bx,by] = createBox(pts(1,i),pts(2,i),angle,carWidth/resolution,carLength/resolution);
        
        % calculate Cost
        for i = 1:1:4
            cx = fix(bx(i)+0.5);
            cy = fix(by(i)+0.5);
            newcost = B(cx, cy);
            pathcost = pathcost + newcost;
        end
        
        %hold on 
        %plot([bx],[by])
        %hold off
    end
    
    if pathcost < cost  
        hold on
        plot(pts(1,:),pts(2,:))
        hold off
        display('Alternative way is better!')
    end
end

end