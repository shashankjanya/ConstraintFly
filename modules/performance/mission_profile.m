function [pos, flag] = mission_profile(weight, planformarea, params)
    
    function times = claculate_mission_segment_time()
            climb_time = params.altitude / params.ROC;
            cruise_time = params.range / params.vcruise;
            descent_time = params.altitude / params.ROD;
            times = [climb_time, cruise_time, descent_time];
    end
    
    

end