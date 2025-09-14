classdef AircraftSizing2 < handle
    properties
        params;
        assump;
        aero;
        CLmax;
        e;
        CD0;
        constraints;       
    end

    methods
        function obj = AircraftSizing2(assump,params,aero)
            obj.params = params;
            obj.assump = assump;
            obj.aero = aero;
            obj.CLmax = obj.aero.CL_maximum;
            obj.e = obj.aero.e;
            obj.CD0 = obj.aero.CD0;

            obj.constraints = {};
        end

        function PbyW = master_equation(obj, velocity, roc, acceleration)
            q = (0.5*obj.assump.rho).*(velocity.^2);
            PbyW = @(WbyS, CL) ...
                (q .* (1 ./ WbyS) .* (obj.CD0 + (1./(pi*obj.e*obj.params.AR)).*(CL.^2))) .* velocity ...
                + roc + (velocity./obj.assump.g).*acceleration;
        end

        function add_constraint(obj, eqn, name, type)
            obj.constraints{end+1} = struct('eqn', eqn, 'name', name, 'type', type);
        end
        
        function Clcruise = calculate_lift_coefficient(obj, Vcruise, n, gamma)
            Clcruise = @(WbyS) ((2*cos(gamma)*n) ./ (obj.assump.rho .* (Vcruise.^2))) .* WbyS;
        end

        function n = calculate_load_factor(obj, velocity, r)
            n = sqrt(((velocity.^2)./(obj.assump.g*r)).^2 + 1);
        end
        
        % example constraints
        function PbyW_cruise = cruise_constraint(obj)
            CLcruise = obj.calculate_lift_coefficient(obj.params.vcruise, 1, 0);
            base_handle = obj.master_equation(obj.params.vcruise, 0, 0);
            PbyW_cruise = @(WbyS) base_handle(WbyS, CLcruise(WbyS)) ./ obj.assump.eta;
        end
        
        function PbyW_climb = climb_constraint(obj)
            gamma_climb = asin(obj.params.ROC/obj.params.vcruise); 
            CLclimb = obj.calculate_lift_coefficient(obj.params.vcruise, 1, gamma_climb);
            base_handle = obj.master_equation(obj.params.vcruise, obj.params.ROC, 0);
            PbyW_climb = @(WbyS) base_handle(WbyS, CLclimb(WbyS)) ./ obj.assump.eta;
        end

        function PbyW_turn = turn_constrraint(obj)
            n_turn = obj.calculate_load_factor(obj.params.vcruise, obj.params.radiusturn);
            CLturn = obj.calculate_lift_coefficient(obj.params.vcruise, n_turn, 0);
            base_handle = obj.master_equation(obj.params.vcruise, 0, 0);
        
            PbyW_turn = @(WbyS) arrayfun(@(WS) ...
                (CLturn(WS) > obj.CLmax) * 1e6 + ...
                (CLturn(WS) <= obj.CLmax) * (base_handle(WS, CLturn(WS)) ./ obj.assump.eta), ...
                WbyS);
        end

        function WbyS_stall = calculate_stall_constraint(obj, Vstall)
            WbyS_stall = @(WbyS) 0.5 * obj.assump.rho * (Vstall^2) * obj.CLmax;
        end

        function plot_constraints(obj, W_range, y_lim)
            if nargin < 3
                y_lim = [0,100];
            end
            figure; hold on; grid on;
            xlabel('W/S (N/m^2)');
            ylabel('P/W (W/N)');
            title('Constraint Diagram');
        
            for k = 1:numel(obj.constraints)
                c = obj.constraints{k};
        
                if strcmp(c.type, 'PbyW')
                    P_vals = c.eqn(W_range);
                    plot(W_range, P_vals, 'LineWidth', 1.5, 'DisplayName', c.name);
        
                elseif strcmp(c.type, 'WbyS')
                    W_stall = c.eqn(W_range);
                    xline(W_stall, '--r', 'LineWidth', 1.5, 'DisplayName', c.name);
                else
                    warning('Unknown constraint type: %s', c.type);
                end
            end
            ylim(y_lim);
            legend show;
         end

    end    
end