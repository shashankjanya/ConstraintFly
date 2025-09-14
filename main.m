ARs = 5:1:20;
airfoils = {'NACA4412', 'NACA2412'};

% ARs = 10;
% airfoils = {'NACA4412'};

size_list = {};
vstall = 9.5;

WbyS = 2:1:70;
PbyW = 0:2:150;

optimizer = OptFullFact(WbyS, PbyW);
for a = 1:numel(airfoils)
    for AR = ARs

        design_points = [];
            parameters = AircraftParameters();
            parameters.vstall = vstall;
            parameters.AR = AR;
            parameters.airfoil = airfoils{a};
            parameters.powertrain = 'b3_m2_p3';
            parameters.vcruise = vstall + 3;
            
            assumptions = Assumptions();
            
            aerodynamics = AerodynamicsZeroFidelity(parameters);
            aero = aerodynamics.zero_fidelity_aerodynamics();
            
            size = AircraftSizing2(assumptions, parameters, aero);
            
            cruise_constraint = size.cruise_constraint();
            size.add_constraint(cruise_constraint, 'cruise', 'PbyW');
            
            climb_constraint = size.climb_constraint();
            size.add_constraint(climb_constraint, 'climb', 'PbyW');
            
            turn_constraint = size.turn_constrraint();
            size.add_constraint(turn_constraint, 'turn', 'PbyW');
            
            stall_constraint = size.calculate_stall_constraint(parameters.vstall);
            size.add_constraint(stall_constraint, 'stall', 'WbyS')

            % new_constraint = @(WbyS) WbyS;
            % size.add_constraint(new_constraint, 'linear', 'PbyW');
            % 
            % size.plot_constraints(WbyS, [0, 50])
             
            optimizer.get_enclosed_values(size.constraints, parameters);

            design_points = unique_design_points(optimizer.enclosed_values, design_points);

        [weights, planformareas] = feasible_weights_structures(design_points(:,1), design_points(:,2), parameters);
        mask = zeros(1, numel(weights));
        weights_mission_feasible = {};
        planformareas_mission_feasible = {};

        for i = 1:numel(weights)
            mission = MissionProfile(weights(i).weight, planformareas(i), aero, parameters, assumptions);
            mask(i) = mission.simulate();
            if mask(i)
                weights_mission_feasible{end+1}  = weights(i);
                planformareas_mission_feasible{end+1} = planformareas(i);
                [weight_payload, payload_frac] = objective(weights_mission_feasible{end});
                
                % [total_weight, score, max_index] = objective(weights, v_stall,
                % t_mission); % New objective function
                weights_mission_feasible{end}.weight_payload = weight_payload;
                size_def = size_definition(weights_mission_feasible{end}, planformareas_mission_feasible{end}, payload_frac, parameters, assumptions);
                
                size_list{end+1} = size_def;
            end
        end

    end
end

size_array_struct = [size_list{:}];
T = struct2table(size_array_struct);
T = sortrows(T, 'score', 'descend');

plot_results(T);