function c = readLocalCostmap(A, hight, width, resolution)

figure

for i = -hight/2 : 1 : hight/2
    for j = -width/2 : 1 width/2
        hold on
        val = A((((i+hight/2+1))*200)+(j+width/2+1));
        plot(i,j,'Color', [val,val,val]);
        hold off
    end
end


end