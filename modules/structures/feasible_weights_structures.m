function [weights, planform_area] = feasible_weights_structures(weight, planformarea, parameters)
    
    propulsion_data;
    powertrian = powertrain_data.(parameters.powertrain);
    weight_propulsion = powertrian.weight;

    weight_comp = load('modules\structures\fixed_components.mat');
    values = struct2cell(weight_comp.weights_components);       
    weight_components = sum(cell2mat(values));

    weight_airframe_required = weight_airframe_3d(planformarea, parameters.AR); 
    weight_airframe_available = weight - weight_propulsion - weight_components;
    
    mask = (weight_airframe_available >= 1.1*weight_airframe_required);
    weights_struct = weight(mask);
    planform_area = planformarea(mask);
    weight_airframe = weight_airframe_required(mask);

    n = numel(weights_struct);
    weights = repmat(struct('weight', [], ...
                        'weight_airframe', [], ...
                        'weight_propulsion', [], ...
                        'weight_components', []), n, 1);

    for i = 1:numel(weights_struct)
        weights(i).weight            = weights_struct(i);
        weights(i).weight_airframe   = weight_airframe(i);
        weights(i).weight_propulsion = weight_propulsion;
        weights(i).weight_components = weight_components;
    end
end
