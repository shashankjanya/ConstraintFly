function [vi] = hand_launch(weight, planformarea, parameters, assumptions, aero)


rho = assumptions.rho;
g = assumptions.g;
planformarea = planformarea;
mass = weight/g;

% static_thrust = str2double(prop.static_thrust);
static_thrust = 10;
thrust = 0.75*static_thrust;

%% launching phase
launch_force = 50; arm_flexion = 0.7;   
v_o = sqrt(2*(launch_force/2)*arm_flexion);
v_o = [v_o; 0; 0];                                                         % velocity in kinematic frame   
gamma_o = 45*pi/180;
xi_o = 0;
vi_o = R3(-xi_o)*R2(-gamma_o)*v_o;                                         % velocity in inertial frame

lift_o = mass*g*cos(gamma_o);                                              % initial conditions at t=0 s
CL_o = CLift(lift_o, norm(v_o), planformarea, rho); 
CD_o = CDrag(aero.CD0, aero.e, parameters.AR, CL_o);
drag_o = 0.5*rho*(norm(v_o)^2)*planformarea*CD_o;
acc_o = thrust/mass - drag_o/mass - g*sin(gamma_o);                        % acceleration in kinematic frame

%% numerical integration
delt = 0.1;
endt = 1;
v = v_o; vi = vi_o; acc = acc_o;
gamma = gamma_o; xi = xi_o;
s = [0;0;2];
time = [0];

while time(end) < endt
    new_v = v(:,end) + [acc(end).*delt; 0; 0];
    v = [v, new_v];
   
    new_vi =  R3(-xi(end))*R2(-gamma(end))*v(:,end);
    vi = [vi, new_vi];
    
    new_s = s(end) + new_vi.*delt;
    s = [s, new_s];

    new_gamma = atan2(vi(3, end), vi(1, end));
    gamma = [gamma, new_gamma];
    
    new_xi = atan2(vi(2, end), vi(1, end));
    xi = [xi, new_xi];

    lift = mass*g*cos(gamma(end));                                              
    CL = CLift(lift, norm(v(:,end)), planformarea, rho); 
    CD = CDrag(aero.CD0, aero.e, parameters.AR, CL);
    drag = 0.5*rho*(norm(v(:,end))^2)*planformarea*CD;
    
    new_acc = thrust/mass - drag/mass - g*sin(gamma(end));                 % scalar value
    acc = [acc, new_acc];
    
    new_time = time + delt; 
    time = [time, new_time];
    time(end)
    
    if s(3, end) < 0.2  
        disp('unsuccessful');
        break;
    end
end
end


    
%% auxilary function
function cl = CLift(L,v, planformarea, rho)
    cl = ( 2*L/(rho*(v^2)*planformarea) );
end

function cd = CDrag(CD0, e, AR, CL)
    cd = CD0 + (1/(pi*e*AR))*(CL^2);
end

function r2 = R2(gamma)
    r2 = [cos(gamma), 0, -sin(gamma);
          0, 1, 0;
          sin(gamma), 0, cos(gamma)];
end

function r3 = R3(xi)
    r3 = [cos(xi), sin(xi), 0;
          -sin(xi), cos(xi), 0;
          0, 0, 1];
end