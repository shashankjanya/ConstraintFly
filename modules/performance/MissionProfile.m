classdef MissionProfile < handle
    properties
        weight, s, aero, params, assump;
        pos, velocity, energy;
    end

    methods
        function obj = MissionProfile(weight, planformarea, aero, parameters, assumptions)
            obj.weight = weight;
            obj.s = planformarea;
            obj.aero = aero;
            obj.params = parameters;
            obj.assump = assumptions;

            obj.pos = [];
            obj.velocity = [];
            obj.energy = [];

            obj.update_parameters();
        end

        function times = claculate_mission_segment_time(obj)
            climb_time = obj.params.altitude / obj.params.ROC;
            cruise_time = obj.params.range / obj.params.vcruise;
            descent_time = obj.params.altitude / obj.params.ROD;
            times = [climb_time, cruise_time, descent_time];
        end
        
        function drag = calculate_drag(obj, v, CL)
            q = 0.5*obj.assump.rho*(v^2);
            CD = obj.assump.CD0 + (1/(pi*obj.aero.e*obj.params.AR))*(CL^2);
            drag = q * obj.s * CD;
        end

        function CL = calculate_lift_coefficient(obj, v, n, gamma)
            q = 0.5*obj.assump.rho*(v^2);
            CL = (n * obj.weight * cos(gamma))./(q * obj.s);
        end

        function flag = simulate(obj)

            times = obj.claculate_mission_segment_time();
            
            % climb simulation 
            gamma_climb = asin(obj.params.ROC/obj.params.vcruise); 
            obj.velocity(1,:) = [cos(gamma_climb)*obj.params.vcruise, 0, obj.params.ROC];
            obj.pos(1,:) = obj.velocity(1,:).*times(1);
            
            CL_climb = obj.calculate_lift_coefficient(obj.params.vcruise, 1, gamma_climb);
            drag_climb = obj.calculate_drag(obj.params.vcruise, CL_climb);
            power_climb = drag_climb * obj.params.vcruise;
            obj.energy(1) = power_climb * times(1);

            % cruise simulation 
            obj.velocity(2,:) = [cos(gamma_climb)*obj.params.vcruise, 0, 0];
            obj.pos(2,:) = obj.velocity(2,:).*times(2);
            
            CL_cruise = obj.calculate_lift_coefficient(obj.params.vcruise, 1, 0);
            drag_cruise = obj.calculate_drag(obj.params.vcruise, CL_cruise);
            power_cruise = drag_cruise * obj.params.vcruise;
            obj.energy(2) = power_cruise * times(2);

            % glide simulation
            gamma_descent = -asin(obj.params.ROD/obj.params.vcruise); 
            obj.velocity(3,:) = [cos(gamma_climb)*obj.params.vcruise, 0, obj.params.ROD];
            obj.pos(3,:) = obj.velocity(3,:).*times(3);
            
            CL_descent = obj.calculate_lift_coefficient(obj.params.vcruise, 1, gamma_descent);
            drag_descent = obj.calculate_drag(obj.params.vcruise, CL_descent);
            power_descent = drag_descent * obj.params.vcruise;
            obj.energy(3) = power_descent * times(3);
            
            propulsion_data;
            powertrain = powertrain_data.(obj.params.powertrain);
            propulsion_energy = powertrain.energy;
            
            if sum(obj.energy) <= propulsion_energy
                flag = true;
            else 
                flag = false;
            end

        end
        
        function update_parameters(obj)
            obj.params.vstall = sqrt(obj.weight ./ (0.5 * obj.assump.rho * obj.s * obj.aero.CL_maximum));
            q = 0.5 * obj.assump.rho * (obj.params.vcruise^2);
            n_max = (q * obj.aero.CL_maximum * obj.s) / obj.weight;
            obj.params.radiusturn = (obj.params.vcruise^2) / (sqrt(n_max^2 - 1) * 9.81);
        end
    end
end