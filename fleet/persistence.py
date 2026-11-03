"""Owner-only, atomic local configuration storage."""
import json
import os
import tempfile
from pathlib import Path


def read_state(path):
    path = Path(path)
    if not path.exists():
        return {}
    with path.open() as stream:
        data = json.load(stream)
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('本地配置格式不支持，请检查 .local_state/fleet.json')
    return data


def write_state(path, data):
    path = Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.fleet-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
