"""Real TCP analysis stream remains independent of selected rendering pose."""
from dataclasses import asdict, replace
import json
import time
import zmq
from jsonschema import validate
from bullet_msgs import schema_bytes
from bullet_drone.config import PoseSources, SensorsSettings, BarometerSettings
from bullet_drone.simulation import RcSimulation
from bullet_drone.commands import RcInput
from examples.position_square.demo import FLIGHT_CONTROL_SETTINGS
from tools.zmq_render.publisher import RenderPublisher


def test_analysis_sample_rates_truth_and_filtered_sensor_sources(config,tmp_path):
    with zmq.Context() as context, context.socket(zmq.SUB) as subscriber:
        subscriber.setsockopt(zmq.SUBSCRIBE,b'analysis/')
        # Reserve an endpoint, then let the publisher own its normal bind.
        with context.socket(zmq.PUB) as reservation:
            port=reservation.bind_to_random_port('tcp://127.0.0.1')
        endpoint=f'tcp://127.0.0.1:{port}'
        subscriber.connect(endpoint)
        transport=tmp_path/'transport.json'
        transport.write_text(json.dumps(dict(telemetry_endpoint=endpoint,image_endpoint='tcp://127.0.0.1:5556',control_endpoint='tcp://127.0.0.1:5557')))
        path=tmp_path/'render.json'
        path.write_text(json.dumps(dict(transport_config='transport.json',fps=30,camera=dict(width_px=2,height_px=2,fov_x_deg=90))))
        config=replace(config,pose_sources=PoseSources('barometer','ground_truth'),sensors=SensorsSettings(barometer=BarometerSettings(pressure_bias_pa=30)))
        simulation=RcSimulation(config,FLIGHT_CONTROL_SETTINGS)
        publisher=RenderPublisher(path,pose_sources=config.pose_sources)
        packets=[]
        try:
            publisher.start()
            time.sleep(.15)
            observation=simulation.reset()
            source=dict(physics_id='p',episode_id=observation.ground_truth.episode_id,stream_id=publisher.stream,step_index=0,simulation_time_s=0.,measurement_us=1)
            # The worker consumes this cached source independently of flight command validation.
            publisher.feedback={'source':source}
            publisher.set_analysis_config({'simulation':asdict(config)},{'test':None})
            for step in range(241):
                if step:
                    observation=simulation.step(RcInput())
                source=dict(source,step_index=step,simulation_time_s=observation.ground_truth.simulation_time_s,measurement_us=step+1)
                publisher.consume_analysis(observation,source)
                time.sleep(config.simulation.timestep_s)
                while subscriber.poll(0):
                    topic,payload=subscriber.recv_multipart()
                    packets.append((topic,json.loads(payload)))
            time.sleep(.05)
            while subscriber.poll(0):
                topic,payload=subscriber.recv_multipart()
                packets.append((topic,json.loads(payload)))
            assert publisher.error is None
            counts={topic:sum(t==topic for t,_ in packets) for topic in (b'analysis/physics/ground_truth',b'analysis/physics/sensors',b'analysis/physics/config')}
            assert counts[b'analysis/physics/sensors']==241
            assert counts[b'analysis/physics/ground_truth']==101
            assert counts[b'analysis/physics/config']>=1
            for topic,value in packets:
                name={b'analysis/physics/ground_truth':'analysis_ground_truth',b'analysis/physics/sensors':'analysis_sensors',b'analysis/physics/config':'analysis_config'}[topic]
                validate(value,json.loads(schema_bytes(name+'.json')))
            sensor=next(v for t,v in packets if t.endswith(b'/sensors'))
            truth=next(v for t,v in packets if t.endswith(b'/ground_truth'))
            assert sensor['sensors']['barometer']['altitude_m'] != truth['ground_truth']['position_m'][2]
            assert publisher.analysis_drops==0
        finally:
            publisher.close()
            simulation.close()
