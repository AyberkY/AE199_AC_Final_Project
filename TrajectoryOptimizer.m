%% INITIALIZATION
[v, e, loaded] = pyversion;
if loaded == false
    conn = py.krpc.connect(pyargs('address','10.193.87.213','rpc_port',int32(50000),'stream_port',int32(50001)));
end
conn.space_center.quicksave();

%% DATA COLLECTION
starts = linspace(0, 10000, 25);
for i = 1:20
    [xVals, yVals, consumedDeltaV] = launchVessel(conn, starts(1), 55000, 150000);
    ascent(i).turn_start_altitude = starts(i);
    ascent(i).xVals = xVals;
    ascent(i).yVals = yVals;
    ascent(i).consumedDeltaV = consumedDeltaV;
    conn.space_center.quickload();
end

figure;
for i = 1:25;
   figure.plot(ascent(i).xVals, ascent(i).yVals, '-r', 'linewidth', 2); 
end


%% ASCENT FUNCTION
function [xVals, yVals, consumedDeltaV] = launchVessel(conn, turn_start_altitude, turn_end_altitude, target_altitude)

    space_center = conn.space_center;
    vessel = space_center.active_vessel;
    control = vessel.control;
    orbit = vessel.orbit;
    body = orbit.body;
    flight = vessel.flight();
    ap = vessel.auto_pilot;

    stage_2_resources = vessel.resources_in_decouple_stage(pyargs('stage',2,'cumulative',false));
    stage_3_parts = cell(vessel.parts.in_stage(3));

    pad_position = vessel.position(orbit.body.reference_frame);
    pad_ref_frame = body.reference_frame.create_relative(orbit.body.reference_frame, pad_position);
    launch_origin_ref_frame = body.reference_frame.create_relative(pad_ref_frame, py.tuple([0 0 0]), vessel.rotation(pad_ref_frame));

    xLine = conn.drawing.add_line(py.tuple([0 0 0]), py.tuple([5 0 0]), launch_origin_ref_frame);
    yLine = conn.drawing.add_line(py.tuple([0 0 0]), py.tuple([0 5 0]), launch_origin_ref_frame);
    zLine = conn.drawing.add_line(py.tuple([0 0 0]), py.tuple([0 0 5]), launch_origin_ref_frame);
    xLine.color = py.tuple([1 0 0]);
    yLine.color = py.tuple([0 1 0]);
    zLine.color = py.tuple([0 0 1]);
    
    solidFuelMass = stage_2_resources.amount('SolidFuel') * vessel.resources.density('SolidFuel');
    solidBoosterDryMass = stage_3_parts{1}.dry_mass;
    solidBoosterISP = stage_3_parts{1}.engine.kerbin_sea_level_specific_impulse;
    solidBoosterThrust = stage_3_parts{1}.engine.available_thrust;
    
    liquidEngineISP = stage_3_parts{5}.engine.kerbin_sea_level_specific_impulse;
    liquidEngineThrust = stage_3_parts{5}.engine.available_thrust;
    
    combinedISP = (solidBoosterThrust * 4 + liquidEngineThrust) / ((solidBoosterThrust / solidBoosterISP) * 4 + (liquidEngineThrust / liquidEngineISP));
    
    totalDeltaV = 9.81 * combinedISP * log(vessel.mass / vessel.dry_mass)
    
    figure;
    xlim([0 100000]);
    ylim([0 100000]);
    xVals = [];
    yVals = [];

    control.sas = false;
    control.rcs = false;
    control.throttle = 1.0;

    ap.reference_frame = vessel.surface_reference_frame;
    ap.engage();
    ap.target_pitch_and_heading(90,90);
    control.activate_next_stage();

    srbs_separated = false;
    turn_angle = 0.0;

    while true
        if flight.mean_altitude > turn_start_altitude && flight.mean_altitude < turn_end_altitude
            frac = ((flight.mean_altitude - turn_start_altitude) / (turn_end_altitude - turn_start_altitude));
            new_turn_angle = frac * 90;
            if abs(new_turn_angle - turn_angle) > 0.5
               turn_angle = new_turn_angle;
               ap.target_pitch_and_heading(90-turn_angle,90);
            end
        end

        if srbs_separated == false
            if stage_2_resources.amount('SolidFuel') < 0.1
                control.activate_next_stage();
                srbs_separated = true;
                sprintf('SRBs separated')
            end
        end

        if orbit.apoapsis_altitude > target_altitude * 0.9
           sprintf('Approaching targe apoapsis')
           break;
        end 

        position = cell(vessel.position(launch_origin_ref_frame));
        xVals = [xVals, position{1}];
        yVals = [yVals, position{2}];
        plot(xVals, yVals, '-r', 'linewidth', 3);
        xlim([0 60000]);
        ylim([0 60000]);
        drawnow;

    end

    control.throttle = 0.25;
    while orbit.apoapsis_altitude < target_altitude 
    end

    sprintf('Target apoapsis reached')
    control.throttle = 0.0;
    
    finalISP = liquidEngineISP;
    finalDeltaV = 9.81 * finalISP * log(vessel.mass / vessel.dry_mass);
    
    consumedDeltaV = totalDeltaV - finalDeltaV
    
end
