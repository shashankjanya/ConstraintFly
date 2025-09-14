classdef AerodynamicsZeroFidelity
    properties
        airfoil_name
        AR
        Cl_a
        Cl_0
        Cl_max
        angle_Cl_0
    end
    
    methods
        function obj = AerodynamicsZeroFidelity(params)
            obj.airfoil_name = params.airfoil;
            obj.AR = params.AR;
            
            airfoils = obj.get_airfoil_data();
            
            if isfield(airfoils, obj.airfoil_name)
                airfoil_data = airfoils.(obj.airfoil_name);
                obj.Cl_a = airfoil_data.Cl_a;
                obj.Cl_0 = airfoil_data.Cl_0;
                obj.Cl_max = airfoil_data.Cl_max;
                obj.angle_Cl_0 = airfoil_data.angle_Cl_0;
            else
                error('Airfoil data not found.');
            end
        end
        
        function airfoils = get_airfoil_data(~)
            airfoils = load('modules\aerodynamics\airfoils_data.mat');
            airfoils = airfoils.airfoils_data;
        end
        
        function [CL_a, e] = CL_a(obj)
            e = 2 / (2 - obj.AR + sqrt(4 + (obj.AR^2)));
            CL_a = obj.Cl_a / (1 + (57.3 * obj.Cl_a / (pi * e * obj.AR)));
        end
        
        function CL_max = CL_maximum(obj)
            [CL_a, ~] = obj.CL_a();
            CL_max = CL_a * (15 - obj.angle_Cl_0);
            if CL_max > 1.2
                CL_max = 1.2;
            end
        end
        
        function CL_incidence = Cl_incidence(obj)
            [CL_a, ~] = obj.CL_a();
            CL_incidence = CL_a * (-obj.angle_Cl_0);
        end
        
        function wing_aerodynamics = zero_fidelity_aerodynamics(obj)
            CD0 = 0.03;
            [CL_a, e] = obj.CL_a();
            CL_maximum = obj.CL_maximum();
            CL_incidence = obj.Cl_incidence();
            wing_aerodynamics = struct('CL_a', CL_a, 'e', e,'CL_maximum', CL_maximum, ...
                'CL_incidence', CL_incidence, 'CD0', CD0);
        end
    end
end