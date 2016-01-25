function cost = calcCost(px, py, B, hight, width)
    %% Parameter
    outsideCost = 50;
    
    %% Calculate Path cost
    cost = 0;
    for i=1:1:length(px)
        % If the calculation Point is outside the costmap
        if (px(i) < 1 | py(i) < 1 | px(i) > width | py(i) > hight)
            cost = cost + outsideCost;
        else
            cost = cost + B(px(i),py(i));
        end
    end

end