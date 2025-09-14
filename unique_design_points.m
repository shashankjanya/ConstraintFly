function design_points_updated = unique_design_points(current_design_points, design_points)
    if isempty(design_points)
            new_design_points = current_design_points;
    else
        new_design_points = setdiff(current_design_points, design_points, 'rows');
    end

    if isempty(design_points)
        weights = [];
        planform_area = [];
    else 
        weights = design_points(:,1);
        planform_area = design_points(:,2);
    end

    weights = [weights; new_design_points(:,1)];  
    planform_area = [planform_area; new_design_points(:,2)];
    design_points_updated = [weights, planform_area];
end