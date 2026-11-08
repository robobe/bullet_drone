"""Image transport schema resources."""
from importlib.resources import files


def schema_bytes(name):
    if name != 'image.json':
        raise ValueError('unknown image schema')
    return files(__package__).joinpath('schemas', name).read_bytes()
