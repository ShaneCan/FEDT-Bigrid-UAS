#!/usr/bin/env python3
"""Passive Webots input replay and same-grid observation; never makes avoidance decisions."""
import copy
import json
import logging
import math
from pathlib import Path
import threading
import time

from flask import Flask, jsonify, request
from werkzeug.exceptions import HTTPException
from config import load, world_metadata


class ExperimentError(RuntimeError):
    pass


def number(value):
    if isinstance(value, bool) or not math.isfinite(float(value)):
        raise ValueError("Expected a finite number")
    return float(value)


class Replay:
    def __init__(self, bridge, metadata):
        self.bridge, self.metadata = bridge, metadata
        self.lock = threading.RLock()
        self.trial = None

    def status(self):
        frames, now = self.bridge.frames(), self.bridge.now()
        ids = {robot['id'] for robot in self.metadata['robots']}
        ready = all(drone in frames and 'grid_conflicts' in frames[drone]
                    and -.1 <= now-frames[drone]['sim_time'] <= .3 for drone in ids)
        with self.lock:
            trial = copy.deepcopy(self.trial)
        contacts, conflicts, distances = [], [], []
        if trial:
            for drone in sorted(ids):
                frame = frames.get(drone, {})
                if abs(frame.get('schedule_epoch', -1)-trial['epoch']) > 1e-7:
                    continue
                conflicts.extend(frame.get('grid_conflicts', []))
                for contact in frame.get('contacts', []):
                    if contact['other_id'] in ids and contact['time'] >= trial['epoch']:
                        contacts.append(dict(drone_id=drone, **contact))
                distance = frame.get('minimum_distance')
                if isinstance(distance, (int, float)) and math.isfinite(distance):
                    distances.append((distance, frame.get('minimum_distance_pair')))
        minimum = min(distances, key=lambda v: v[0]) if distances else (None, None)
        conflicts.sort(key=lambda c: (c['time'], c['drone_id'], c['peer_id']))
        return dict(backend='webots', **self.metadata, sim_time=now, drivers_ready=ready,
                    trial=trial, frames=frames, contacts=contacts, physical_contact=bool(contacts),
                    conflict=bool(conflicts), grid_conflict=conflicts[0] if conflicts else None,
                    grid_conflicts=conflicts, minimum_distance=minimum[0], minimum_distance_pair=minimum[1])

    def arm(self, payload):
        if set(payload) != {'run_id', 'epoch', 'events', 'grid'}:
            raise ValueError("Arm requires exactly run_id, epoch, events and grid")
        p = copy.deepcopy(payload)
        if not isinstance(p['run_id'], str) or not 1 <= len(p['run_id']) <= 128:
            raise ValueError("Invalid run_id")
        p['epoch'] = number(p['epoch'])
        ids = {robot['id'] for robot in self.metadata['robots']}
        grid = p['grid']
        if not isinstance(grid, dict) or set(grid) != {'origin', 'step', 'rows', 'cols', 'layers'}:
            raise ValueError("Expected grid origin, step, rows, cols and layers")
        for key in ('origin', 'step'):
            if not isinstance(grid[key], list) or len(grid[key]) != 3:
                raise ValueError("Grid vectors must contain three numbers")
            grid[key] = [number(v) for v in grid[key]]
        if min(grid['step']) <= 0:
            raise ValueError("Grid steps must be positive")
        for key in ('rows', 'cols', 'layers'):
            value = number(grid[key])
            if value < 1 or value != math.floor(value):
                raise ValueError("Grid dimensions must be positive integers")
            grid[key] = int(value)
        if grid['rows']*grid['cols']*grid['layers'] > 2147483647:
            raise ValueError("Grid is too large")
        if not isinstance(p['events'], list):
            raise ValueError("events must be a list")
        seen, previous_end = set(), {}
        for event in p['events']:
            if not isinstance(event, dict) or set(event) != {'id', 'drone_id', 'offset', 'ramp', 'hold', 'velocity'}:
                raise ValueError("Wind event requires id, drone_id, offset, ramp, hold, velocity")
            if not isinstance(event['id'], str) or not event['id'] or event['id'] in seen:
                raise ValueError("Invalid or duplicate event id")
            seen.add(event['id'])
            if isinstance(event['drone_id'], bool) or not isinstance(event['drone_id'], int) or event['drone_id'] not in ids:
                raise ValueError("Unknown drone ID")
            for key in ('offset', 'ramp', 'hold'):
                event[key] = number(event[key])
            if not isinstance(event['velocity'], list) or len(event['velocity']) != 3:
                raise ValueError("velocity requires three axes")
            event['velocity'] = [number(v) for v in event['velocity']]
            if (event['offset'] < 0 or event['ramp'] <= 0 or event['hold'] < 0
                    or 2*event['ramp']+event['hold'] > 60 or math.hypot(*event['velocity']) > 1):
                raise ValueError("Invalid wind duration or velocity (maximum 1 m/s)")
        p['events'].sort(key=lambda e: (e['offset'], e['id']))
        for event in p['events']:
            drone = event['drone_id']
            if event['offset'] < previous_end.get(drone, 0):
                raise ValueError("Overlapping wind events on one aircraft")
            previous_end[drone] = event['offset']+2*event['ramp']+event['hold']
        with self.lock:
            if self.trial and not self.trial['cancelled']:
                if all(self.trial[k] == p[k] for k in p):
                    return self.status()
                raise ExperimentError("An active run already owns this observer")
            if not self.bridge.now()+.05 <= p['epoch'] <= self.bridge.now()+30:
                raise ValueError("epoch must be .05..30 simulation seconds ahead")
            if not self.status()['drivers_ready']:
                raise ExperimentError("Updated multi-event Webots drivers are not ready")
            self.trial = dict(p, cancelled=False)
            for drone in sorted(ids):
                events = [e for e in p['events'] if e['drone_id'] == drone]
                packet = [p['epoch'], grid['rows'], grid['cols'], grid['layers'],
                          *grid['origin'], *grid['step'], len(events)]
                for event in events:
                    packet.extend([p['epoch']+event['offset'], event['ramp'], event['hold'], *event['velocity']])
                self.bridge.publish(drone, packet)
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            frames = self.bridge.frames()
            if all(abs(frames.get(drone, {}).get('schedule_epoch', -1)-p['epoch']) < 1e-7 for drone in ids):
                return self.status()
            time.sleep(.01)
        raise ExperimentError("Webots did not acknowledge the schedule; rebuild/restart the updated image")

    def cancel(self, payload):
        if set(payload) != {'run_id'}:
            raise ValueError("Cancel requires run_id")
        with self.lock:
            if not self.trial or payload['run_id'] != self.trial['run_id']:
                raise ExperimentError("Cancel does not own the registered run")
            if not self.trial['cancelled']:
                self.trial['cancelled'] = True
                for drone in sorted(robot['id'] for robot in self.metadata['robots']):
                    self.bridge.publish(drone, [])
        return self.status()


def create_app(replay):
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 65536

    @app.errorhandler(Exception)
    def error(exc):
        if isinstance(exc, HTTPException):
            return jsonify(error=exc.description), exc.code
        if isinstance(exc, ExperimentError):
            return jsonify(error=str(exc)), 409
        if isinstance(exc, (ValueError, KeyError, TypeError)):
            return jsonify(error=str(exc)), 400
        app.logger.exception('Experiment service failure')
        return jsonify(error='Experiment service failure'), 503

    @app.get('/experiment/status')
    def status():
        return jsonify(replay.status())

    def data():
        body = request.get_json()
        if not isinstance(body, dict):
            raise ValueError("Expected a JSON object")
        return body

    @app.post('/experiment/arm')
    def arm():
        return jsonify(replay.arm(data()))

    @app.post('/experiment/cancel')
    def cancel():
        return jsonify(replay.cancel(data()))

    return app


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String, Float64MultiArray

    world = '/ds/crazywebotsworld/worlds/crazyflie.wbt'
    config = load(world)
    logging.getLogger('werkzeug').setLevel(logging.WARNING)

    class Bridge(Node):
        def __init__(self):
            super().__init__('scenario_input_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.lock = threading.RLock()
            self.latest, self.input_publishers = {}, {}
            for drone in config['drone_ids']:
                self.input_publishers[drone] = self.create_publisher(Float64MultiArray, f'/cf{drone}/experiment_disturbance', 10)
                self.create_subscription(String, f'/cf{drone}/experiment_status',
                                         lambda msg, identity=drone: self.receive(identity, msg), 10)

        def receive(self, drone, msg):
            data = json.loads(msg.data)
            if not {'sim_time', 'schedule_epoch', 'contacts', 'bias', 'position', 'grid_conflicts'} <= data.keys():
                return
            with self.lock:
                self.latest[drone] = data

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        def frames(self):
            with self.lock:
                return copy.deepcopy(self.latest)

        def publish(self, drone, data):
            message = Float64MultiArray()
            message.data = [float(value) for value in data]
            self.input_publishers[drone].publish(message)

    rclpy.init()
    node = Bridge()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        replay = Replay(node, world_metadata(world))
        create_app(replay).run(host=config['bind_host'], port=config['experiment_port'],
                               threaded=True, use_reloader=False)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=2)


if __name__ == '__main__':
    main()
