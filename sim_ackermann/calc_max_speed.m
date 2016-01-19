clc
clear all
close all


l = 5; %length in m
sres= 0.01; %resolution of contraints

vmax = 6; %m/s

amax = 10;      %max acceleration
amin = -10;     %max decceleration

v_0 = 0;

speed_con = zeros(l/sres+100);
for i = 1: l/sres
    
    if i< 100
        speed_con(i) =  vmax;
    end
    
    if  100 <= i && i < 150
        speed_con(i) =  1;
    end
    
    if  150 <= i && i < 200
        speed_con(i) =  5;
    end
    
    if  150 <= i && i < 200
        speed_con(i) =  3;
    end
    
    if  200 <= i && i < 400
        speed_con(i) =  3;
    end
        
    if  400 <= i && i < 500
        speed_con(i) =  2;
    end

end

speed_con(l/sres)=0;

plot(speed_con)

dt = 0.03;  %simulation time step
v = zeros(1000);
ldt = 0.001; %lookahed resolution
s = zeros(1000);
t = 1;
a = zeros(1000);

min_vel = 1000;

while(s < l)
    
    
    steps = round(v(t)/-amin/ldt);
    lookahead_constr = zeros(steps);
    s_for = s(t);
    for i= 1: steps
        s_for = s_for + v(t) * ldt;
        const_ind = round(s_for/sres)+1;
        lookahead_constr(i) =  speed_con(const_ind);
        
        min_vel = min(lookahead_constr);
    end
    
    if min_vel > v(t)      
        if v(t) > vmax
            a(t) = 0;
        else
            a(t) = amax;
        end
    else
        a(t) = amin;
    end
    
        
    v(t+1) = v(t) + a(t)*dt; 
    s(t+1) = s(t)+ v(t)*dt;       
    t = t+1;
    
end

timesteps = 0.3:0.3:0.3*1000;
plot(timesteps,s, timesteps,v);
    
    
        
        
        
        
        
        
        
    