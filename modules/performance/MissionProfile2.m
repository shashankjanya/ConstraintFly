classdef MissionProfile2 < handle
    properties
        weight, s, aero, params, assump;
        dt;
        pos, vel, acc, energy_rem;
        power_max, thrust_max
        flag, hl_flag;
    end

    methods
        function obj = MissionProfile2(weight, planformarea, aero, parameters, assumptions)
            obj.weight = weight;
            obj.s = planformarea;
            obj.aero = aero;
            obj.params = parameters;
            obj.assump = assumptions;
            obj.flag = true;
            obj.hl_flag = true;

            obj.pos = [0, 0, 2];
            obj.vel = [0, 0, 0];
            obj.acc = [0, 0, 0];
            propulsion_data;
            powertrain = powertrain_data.(obj.params.powertrain);
            obj.energy_rem = [powertrain.energy];
            obj.thrust_max = powertrain.static_thrust * 0.7;
            obj.power_max = powertrain.power;
            obj.dt = 0.1;            
        end

        function cruise(obj, v ,range)
            disp('Cruise')
            pos_x = 0;
            obj.vel(end, :) = [v, 0, 0];
            while pos_x < range 
                L = obj.weight;
                CL = obj.calculate_lift_coefficient(obj.vel(end, 1), 1, 0);
                D = obj.calculate_drag(obj.vel(end, 1), CL);
                power = D * obj.vel(end, 1);
                T = D;

                obj.euler_integration([T - D, 0, L - obj.weight], power)
                if obj.energy_rem(end) <= 0
                    obj.flag = false;
                    break;
                end
                pos_x = pos_x + obj.vel(end, 1) * obj.dt;
            end
        end

        function climb(obj, v, alt, roc)
            disp('Climb')
            pos_z = 0;
            obj.vel(end, :) = [v, 0, 0];
            while pos_z < abs(alt)
                gamma = asin(roc/v); 
                L = obj.weight * cos(gamma);
                CL = obj.calculate_lift_coefficient(obj.vel(end, 1), 1, gamma);
                D = obj.calculate_drag(obj.vel(end, 1), CL);
                power = ( D + obj.weight * sin(gamma)) * obj.vel(end, 1);
                T = D + obj.weight * sin(gamma);
                
                obj.euler_integration([T - D - obj.weight * sin(gamma), 0, L - obj.weight * cos(gamma)], power)
                if obj.energy_rem(end) <= 0
                    obj.flag = false;
                    break;
                end
                pos_z = pos_z + obj.vel(end, 1) * sin(abs(gamma)) * obj.dt;
            end
        end

        function hand_launch(obj, launch_force)
            disp('Hand Launch')
            ext = 0;
            while ext < 0.7
                % CL = obj.calculate_lift_coefficient(obj.vel(end,1), 1, 0);
                CL = 0.7;
                D = obj.calculate_drag(obj.vel(end,1), CL);
                power = obj.power_max;

                obj.euler_integration([launch_force + obj.thrust_max - D, 0, 0], power)
                if obj.pos(end, 3) <= 0.3
                    obj.hl_flag = false;
                    break;
                end
                ext = obj.pos(end, 1);
            end
            
            while obj.pos(end, 3) < 1 && length(obj.pos(:,3)) > (3 / obj.dt) 
                % CL = obj.calculate_lift_coefficient(obj.vel(end,1), 1, 0);
                CL = 0.7;
                D = obj.calculate_drag(obj.vel(end,1), CL);
                T = obj.thrust_max;
                L = 0.5 * obj.assump.rho * (norm(obj.vel(end,:))^2) * obj.s * CL;
                power = obj.power_max;
                
                obj.euler_integration([T - D, 0, L - obj.weight], power)
                if obj.energy_rem(end) <= 0
                    obj.flag = false;
                    break;
                end
            end
        end

        function simulate(obj)
            obj.hand_launch(10)
            obj.climb(obj.params.vcruise, obj.params.altitude, obj.params.ROC)    % climb segment 
            obj.cruise(obj.params.vcruise, 2000)                      % cruise segement
            obj.climb(obj.params.vcruise, -obj.params.altitude, -obj.params.ROD)  % descent segment
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
        
        function euler_integration(obj, acc, power)
            obj.acc(end+1, :) = [acc(1), acc(2), acc(3)]./ (obj.weight / 9.81);
            obj.vel(end+1, :) = obj.vel(end, :) + obj.acc(end, :) * obj.dt;
            obj.pos(end+1, :) = obj.pos(end, :) + obj.vel(end, :) * obj.dt;
            obj.energy_rem(end+1) = obj.energy_rem(end) - power * obj.dt;
        end
    end
end