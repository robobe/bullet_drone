"""Simulation wire contracts and local transport configuration."""
from importlib.resources import files
import json
import re

SCHEMAS = ('collision.json',)

def schema_bytes(name):
    if name not in SCHEMAS:
        raise ValueError("unknown simulation schema")
    return files(__package__).joinpath("schemas", name).read_bytes()

def validate_endpoint(endpoint):
    """Validate a local TCP endpoint shared by transport and app publishers."""
    match = re.fullmatch(r"tcp://127\.0\.0\.1:([1-9][0-9]{0,4})", endpoint) if isinstance(endpoint, str) else None
    if not match or not 1 <= int(match[1]) <= 65535:
        raise ValueError("expected loopback TCP endpoint tcp://127.0.0.1:<port 1-65535>")
    return endpoint


def validate_transport(profile):
    required = {"telemetry_endpoint", "image_endpoint", "control_endpoint"}
    if not isinstance(profile, dict) or not required <= set(profile) or set(profile) - required - {"request_endpoint", "request_io"}:
        raise ValueError("expected telemetry_endpoint, image_endpoint and control_endpoint")
    endpoints = [value for key, value in profile.items() if key != 'request_io']
    for endpoint in endpoints:
        validate_endpoint(endpoint)
    if 'request_io' in profile:
        from math import isfinite
        settings = profile['request_io']
        if 'request_endpoint' not in profile or not isinstance(settings, dict) or set(settings) != {'request_timeout_s', 'request_max_queue_age_s'}:
            raise ValueError('invalid request I/O settings')
        if any(type(v) not in (int, float) or not isfinite(v) or v <= 0 for v in settings.values()):
            raise ValueError('request I/O values must be positive and finite')
    if len(set(endpoints)) != len(endpoints):
        raise ValueError("endpoints must differ")
    return profile


def load_transport(path):
    with open(path) as stream:
        return validate_transport(json.load(stream))
