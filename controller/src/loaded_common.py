"""Loaded witness serialization and read-only actuator adapter."""
import hashlib,json
from live_sleep_witness import controls as sleep_controls

def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def controls(env):
    # Preserve the reviewed signed-infinity encoding; rename only the wire key.
    return {arm:[{('velocity' if k=='velocity_target' else k):v for k,v in row.items()} for row in rows]
            for arm,rows in sleep_controls(env).items()}
