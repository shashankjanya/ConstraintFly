classdef OptFullFact < handle
    properties
        constraints
        enclosed_points
        enclosed_values
        mask
        total_mask
        WbyS
        PbyW
    end
    
    methods
        function obj = OptFullFact(WbyS_range, PbyW_range)
            obj.enclosed_points = [];
            obj.enclosed_values = [];
            obj.mask = {};
            [obj.WbyS, obj.PbyW] = meshgrid(WbyS_range, PbyW_range);

        end

        function get_enclosed_points(obj, constraints)
            totalmask = true(size(obj.WbyS));   
            for i = 1:numel(constraints)
                c = constraints{i};
                
                if strcmp(c.type, 'PbyW')
                    obj.mask{i} = (obj.PbyW >= c.eqn(obj.WbyS));
                
                elseif strcmp(c.type, 'WbyS')
                    obj.mask{i} = (obj.WbyS <= c.eqn(obj.WbyS));
                           
                else
                    warning('Unknown constraint type: %s', c.type);
                end
                totalmask = totalmask & obj.mask{i};
            end
            obj.total_mask = totalmask;
            obj.enclosed_points = [obj.WbyS(obj.total_mask), obj.PbyW(obj.total_mask)];
        end

        function get_enclosed_values(obj, constraints, parameters)
            obj.get_enclosed_points(constraints)
            
            weights_powertrain = load("modules\propulsion\propulsion_data.mat");
            weights_powertrain = weights_powertrain.powertrain_data;
             
            if isfield(weights_powertrain, parameters.powertrain)
                weights_powertrain = weights_powertrain.(parameters.powertrain);
                power = str2double(weights_powertrain.power);
             else
                error('powertain does not match the database');
            end

            weight = power.* (1./obj.enclosed_points(:,2));
            planform_area = weight./obj.enclosed_points(:,1);
            obj.enclosed_values = [weight, planform_area];
        end
    end
end