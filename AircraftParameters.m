classdef AircraftParameters < handle
    properties
        % optimization parameters
        vstall
        AR
        airfoil
        powertrain 
        
        % Physical constants
        ROC = 2;           % Rate of climb (m/s)
        ROD = 1;           % Rate of descent (m/s)
        vcruise = 12;      % Velocity in the cruise (m/s)
        radiusturn = 30;    % Turn radius in cruise (m)
        altitude = 30;     % Climb altitude 70m
        range = 2000;      % Cruise range in m
    end
    
    methods
        function obj = AircraftParameters()
        end
    end
end