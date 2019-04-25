%% INITIALIZATION
[v, e, loaded] = pyversion;
if loaded == false
    conn = py.krpc.connect(pyargs('address','10.193.87.213','rpc_port',int32(50000),'stream_port',int32(50001)));
end

space_center = conn.space_center;
vessel = space_center.active_vessel;
control = vessel.control;
orbit = vessel.orbit;
body = orbit.body;
flight = vessel.flight();
ap = vessel.auto_pilot;

stage_2_resources = vessel.resources_in_decouple_stage(pyargs('stage',2,'cumulative',false));

pad_position = vessel.position(orbit.body.reference_frame);
pad_ref_frame = body.reference_frame.create_relative(orbit.body.reference_frame, pad_position);
launch_origin_ref_frame = body.reference_frame.create_hybrid(pad_ref_frame, vessel.reference_frame);

xLine = conn.drawing.add_line(py.tuple([0 0 0]), py.tuple([5 0 0]), launch_origin_ref_frame);
yLine = conn.drawing.add_line(py.tuple([0 0 0]), py.tuple([5 0 0]), launch_origin_ref_frame);
zLine = conn.drawing.add_line(py.tuple([0 0 0]), py.tuple([5 0 0]), launch_origin_ref_frame);
xLine.color = py.tuple([1 0 0]);
yLine.color = py.tuple([0 1 0]);
zLine.color = py.tuple([0 0 1]);

%% ASCENT
turn_start_altitude = 250;
turn_end_altitude = 45000;
target_altitude = 150000;

control.sas = false;
control.rcs = false;
control.throttle = 1.0;

ap.engage();
ap.target_pitch_and_heading(90,90);
control.activate_next_stage();

srbs_separated = false;
turn_angle = 0.0;

while true
    if getAltitude(flight) > turn_start_altitude && getAltitude(flight) < turn_end_altitude
        frac = ((getAltitude(flight) - turn_start_altitude) / (turn_end_altitude - turn_start_altitude));
        new_turn_angle = frac * 90;
        if abs(new_turn_angle - turn_angle) > 0.5
           turn_angle = new_turn_angle;
           ap.target_pitch_and_heading(90-turn_angle,90);
        end
    end
    
    if srbs_separated == false
        if getSolidFuel(stage_2_resources) < 0.1
            control.activate_next_stage();
            srbs_separated = true;
            sprintf('SRBs separated')
        end
    end
    
    if getApoapsis(orbit) > target_altitude * 0.9
       sprintf('Approaching targe apoapsis')
       break;
    end
end

control.throttle = 0.25;
while getApoapsis(orbit) < target_altitude 
end

sprintf('Target apoapsis reached')
control.throttle = 0.0;

%% CIRCULARIZATION

sprintf('Planning circularization burn')
mu = orbit.body.gravitational_parameter;
r = orbit.apoapsis;
a1 = orbit.semi_major_axis;
a2 = r;
v1 = sqrt(mu * ((2 / r) - (1 / a1)));
v2 = sqrt(mu * ((2 / r) - (1 / a2)));
delta_v = v2 - v1;
nodeTime = getUt(space_center) + orbit.time_to_apoapsis;
node = control.add_node(pyargs('ut',nodeTime,'prograde',delta_v));

F = vessel.available_thrust;
Isp = vessel.specific_impulse * 9.81;
m0 = vessel.mass;
m1 = m0 / exp(delta_v / Isp);
flow_rate = F / Isp;
burn_time = (m0 - m1) / flow_rate;

sprintf('Orientating ship for circularization burn')
ap.reference_frame = node.reference_frame;
ap.target_direction = [0 1 0];
vessel.auto_pilot.wait()

sprintf('Waiting until circularization burn')
burn_ut = getUt(space_center) + orbit.time_to_apoapsis - (burn_time / 2);
lead_time = 5;
space_center.warp_to(pyargs('ut',burn_ut - lead_time));

sprintf('Ready to execute burn')
while (orbit.time_to_apoapsis - (burn_time / 2)) > 0 
end

sprintf('Executing burn')
control.throttle = 1.0;
pause(burn_time - 0.1);

%% FINE TUNING
sprintf('Fine tuning')
control.throttle = 0.03;
while true
    remaining_burn_vector = node.remaining_burn_vector(pyargs('reference_frame',node.reference_frame));
    remaining_burn = cell(remaining_burn_vector(2));
    if remaining_burn{1} < 3.0
        break
    end
end
control.throttle = 0.0;
node.remove();
sprintf('Launch complete')
%% CLEAN UP
%conn.close()
%clear

%% Getters
function [altitude1] = getAltitude(flight)
    altitude1 = flight.mean_altitude;
end

function [apoapsis] = getApoapsis(orbit)
    apoapsis = orbit.apoapsis_altitude;
end

function [srb_fuel] = getSolidFuel(stage_resources)
    srb_fuel = stage_resources.amount('SolidFuel');
end

function [ut] = getUt(space_center)
    ut = space_center.ut;
end