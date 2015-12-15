filename      = 'plan';
delimiterIn   = ' ';
headerlinesIn = 7;
A = importdata(filename,delimiterIn,headerlinesIn);
a=A.data



x = a(2:4:end);
y = a(3:4:end);

figure
plot(x(1:420),y(1:420),'*')
axis equal
