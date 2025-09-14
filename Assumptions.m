classdef Assumptions < handle
    properties
        rho = 1.225;       % Air density (kg/m^3)
        g = 9.81;          % Gravitational acceleration (m/s^2)
        C = 1.4;           % Vlo/Vstall (safety parameter)
        mue = 0.7;         % Coefficient of friction

        eta = 0.6;         % powertrain effiency
        n_struct = 5;      % structures limit load factor
        CD0 = 0.03;
        
        solutionstep = 0.5;  % Resolution of the solution curve
        WbyS_range = 2:0.5:40;
        PbyW_range = 0:0.5:50;
    end 

    methods
        function obj = Assumptions()
        end
    end
end
