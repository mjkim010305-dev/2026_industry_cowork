"""Read-only Nav2 verification. Does not publish poses, velocities, or goals."""
import time
import json
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
rclpy.init()
n = rclpy.create_node('ai_festa_nav2_readonly_check', enable_rosout=False, start_parameter_services=False)
out = {'lifecycle': {}, 'parameters': {}, 'samples': {}, 'endpoints': {}}
subs = []
def sample(topic, msg):
    if topic == '/map':
        out['samples'][topic] = {'frame': msg.header.frame_id, 'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution, 'origin': [msg.info.origin.position.x, msg.info.origin.position.y], 'cells': len(msg.data)}
    else:
        out['samples'][topic] = {'frame': msg.header.frame_id, 'beams': len(msg.ranges)}
for topic, typ in [('/map', OccupancyGrid), ('/scan', LaserScan)]:
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE if topic == '/map' else ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/map' else DurabilityPolicy.VOLATILE)
    subs.append(n.create_subscription(typ, topic, lambda msg, t=topic: sample(t, msg), qos))
names = ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator', 'smoother_server', 'behavior_server', 'waypoint_follower', 'velocity_smoother']
clients = []; futures = []
end = time.monotonic() + 3
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.1)
for name in names:
    c = n.create_client(GetState, '/' + name + '/get_state'); clients.append(c)
    c.wait_for_service(timeout_sec=0.5)
    futures.append(('state', name, c.call_async(GetState.Request())))
    c = n.create_client(GetParameters, '/' + name + '/get_parameters'); clients.append(c)
    c.wait_for_service(timeout_sec=0.5)
    keys = ['use_sim_time'] + (['yaml_filename'] if name == 'map_server' else [])
    futures.append(('params', name, c.call_async(GetParameters.Request(names=keys))))
end = time.monotonic() + 12
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
for kind, name, f in futures:
    if not f.done():
        out['lifecycle' if kind == 'state' else 'parameters'][name] = 'NO_RESPONSE'; continue
    r = f.result()
    if kind == 'state': out['lifecycle'][name] = r.current_state.label
    else: out['parameters'][name] = [{'bool': v.bool_value, 'string': v.string_value} for v in r.values]
for topic in ['/map', '/scan', '/cmd_vel_nav', '/cmd_vel']:
    out['endpoints'][topic] = {}
    for role, infos in [('publishers', n.get_publishers_info_by_topic(topic)), ('subscribers', n.get_subscriptions_info_by_topic(topic))]:
        out['endpoints'][topic][role] = [{'node': i.node_namespace.rstrip('/') + '/' + i.node_name, 'type': i.topic_type, 'reliability': str(i.qos_profile.reliability)} for i in infos if i.node_name != n.get_name()]
print(json.dumps(out, indent=2))
Path('/home/students/workspace/ai_festa_diagnostics/latest_verification.json').write_text(json.dumps(out, indent=2) + '\n')
n.destroy_node(); rclpy.shutdown()
