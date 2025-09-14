clear;
clc;
%% the limitation of this simulation is that it assumes alpha to be constant,...
...(ie, alpha_trim). however in reality, after about 2s the pilot will give the elevator input which changes the AOA...
    ... but for conceptual design, assuing a constant AOA is adequate and anyways underestimates the results (which is good). 

%% aircraft parameters
weight = 20;
mass = weight/9.81;
thrust = 12;
rho = 1.225;
Cd0 = 0.03;
AR = 12;
planformarea = 0.3;

Cl = @(alpha) 0.6 + 2*pi*alpha*pi/180;               % use the lift curve from xflr5
Cd = @(Cl) Cd0 + (1/(pi*0.9*AR))*(Cl.^2);            % update Cd0 and (1/pi*e*AR)
lift = @(v, Cl) 0.5*rho*(v.^2)*planformarea*Cl;             
drag = @(v, Cd) 0.5*rho*(v.^2)*planformarea*Cd;

alpha_trim = 0;                                
Cl_cruise = Cl(alpha_trim);
Cd_cruise = Cd(Cl_cruise);
climb_angle = @(v) atan2(v(3), v(1))*180/pi;

h_0 = 2;                                     % initial launching height
delta_t = 0.01;                              % time steps for integration 

%% states                                    
% all the states are created as arrays, which are concatenated duting the
% simulation. To see the trajectory of any of the state over time, plot it
% like this; plot(time, gamma)
time = [0];                                 
vx = [0]; vz = [0]; vy = [0];                % vy is taken only for completeness 
ax = [0]; az = [0];
sx = [0]; sz = [h_0];                         
v = [0;0;0];
vx = [4.5]; vy = [0]; vz = [4.5];            % sqrt(vx^2 + xz^2) is the velocity at the launch (70cm flexion of arm point)
gamma = [(atan2(vz(end),vx(end)))];          % initial launch angle calculation

%% simulation
while time(end) < 3
    v = [vx(end); vy(end); vz(end)];
    dgamma = climb_angle(v);
    gamma = [gamma, dgamma];
  
    ax = thrust*cos(dgamma*pi/180) - lift(norm(v),Cl_cruise)*sin(dgamma*pi/180) - drag(norm(v),Cd_cruise)*cos(dgamma*pi/180); 
    ax = ax/mass;
    az = -weight + thrust*sin(dgamma*pi/180) + lift(norm(v),Cl_cruise)*cos(dgamma*pi/180) - drag(norm(v),Cd_cruise)*sin(dgamma*pi/180);
    az = az/mass;

    dvx = vx(end) + ax*delta_t;
    dvy = 0;
    dvz = vz(end) + az*delta_t;      
    vx = [vx, dvx];
    vy = [vy, dvy];
    vz = [vz, dvz];
    
    dsx = sx(end) + vx(end)*delta_t; 
    dsz = sz(end) + vz(end)*delta_t;
    sx = [sx, dsx];
    sz = [sz, dsz];

    dtime = time(end) + delta_t;
    time = [time, dtime];

    if sz(end) < 0.2 
        disp('hand launch unsuccessful')
        break;
    end
end

%% trajectory
figure('Name','hand launch trajectory');
hold on;
scatter3(sx, zeros(1,length(sx)),sz, 'filled');
view(3); xlabel('x position'); ylabel('y position'); zlabel('z position')