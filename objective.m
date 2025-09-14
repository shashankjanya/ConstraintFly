function [weight_payload, payload_frac] = objective(weights)
    weight_payload = weights.weight - weights.weight_airframe - weights.weight_propulsion...
                    - weights.weight_components;
    payload_frac = weight_payload./weights.weight;

    % [max_payload_frac, max_index] = max(payload_frac);
end

% function [total_weight, score] = objective(weights, v_stall, t_mission)
%     total_weight = weights.weight + weights.weight_airframe + weights.weight_propulsion + weights.weight_components;
%     score = 70 + 30*exp(-1.5.*(total_weight - 1.25)) - exp(4*(v_stall - 9)) - 100*exp(-t_mission./500);
% end