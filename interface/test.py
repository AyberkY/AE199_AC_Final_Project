# import krpc
#
# conn = krpc.connect.('10.193.87.213', rpc_port=50000, stream_port=50001)
# vessel = conn.space_center.active_vessel
#
# ut = conn.add_stream(getattr, conn.space_center, 'ut')
# altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
# apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
# stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
# srb_fuel = conn.add_stream(stage_2_resources.amount, 'SolidFuel')
#
# def get_altitude():
#     return altitude()

print('hello world')
