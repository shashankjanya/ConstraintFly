function weight_airframe = weight_airframe_3d(planform_area, AR)
    % returns weight in N
    % C - chord, b - span, S - planform area
    lin_rho_cf = 0.085; % kg/m
    boom_length = 0.5; % m

    C = planform_area ./ sqrt(AR .* planform_area);  
    b = planform_area ./ C;                                     

    weight_wing = calculate_component_weight(planform_area, C);

    Shtail = (0.5 .* planform_area .* C) ./ 0.65;
    bhtail = sqrt((AR ./ 2) .* Shtail);
    Chtail = Shtail ./ bhtail;
    htail_weight = calculate_component_weight(Shtail, Chtail);

    Svtail = (0.045 .* planform_area .* C) ./ b;
    bvtail = sqrt((AR ./ 2) .* Svtail);
    Cvtail = Svtail ./ bvtail;
    vtail_weight = calculate_component_weight(Svtail, Cvtail) .* 0.5;

    boom_weight = lin_rho_cf*boom_length*9.81;

    weight_airframe = weight_wing + htail_weight + vtail_weight + boom_weight;
end

function weight = calculate_component_weight(S, C)
    SwbyS = 2.1;    % ratio of wetted area to planform area
    t = 0.0004;     % printing thickness
    density = 1.26; % material density (g/cm^3)

    weight = (1 .* t .* SwbyS .* S + 0.687 .* S .* 0.08 .* 0.09 .* C) .* (density .* 1000 .* 9.81);
end