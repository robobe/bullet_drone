"""Tk wind editor, pumped by the simulation owner at step boundaries."""
from dataclasses import asdict
import time
from uuid import uuid4

from bullet_drone.config import WindSettings, ConfigurationError


UI_INTERVAL_S = 0.05
AXES = ('X', 'Y', 'Z')


def wind_settings(enabled, components):
    return WindSettings(enabled=enabled, velocity_world_m_s=tuple(float(v) for v in components))


def publish_wind(simulation, renderer, observation, request_id):
    snapshot = simulation.wind
    renderer.publisher.set_wind(dict(
        schema_version=1, physics_id=renderer.physics_id, controller_id=None,
        episode_id=observation.ground_truth.episode_id, request_id=request_id,
        settings=asdict(snapshot.settings), velocity_world_m_s=list(snapshot.velocity_world_m_s),
        simulation_time_s=snapshot.simulation_time_s,
        result='idle' if request_id is None else 'applied',
        reason='startup' if request_id is None else 'ui_apply'))


class WindPanel:
    def __init__(self):
        import tkinter as tk
        self.root = tk.Tk()
        self.root.title('Bullet wind control')
        self.closed = False
        self.pending = None
        self.request_id = None
        self.next_update = 0.0
        self.enabled = tk.BooleanVar(self.root, value=True)
        tk.Checkbutton(self.root, text='Enable wind', variable=self.enabled).grid(row=0, columnspan=2)
        self.components = []
        for row, axis in enumerate(AXES, start=1):
            tk.Label(self.root, text=f'World {axis} velocity (m/s)').grid(row=row, column=0)
            value = tk.StringVar(self.root, value='0')
            tk.Entry(self.root, textvariable=value).grid(row=row, column=1)
            self.components.append(value)
        tk.Button(self.root, text='Apply wind', command=self.apply).grid(row=4, columnspan=2)
        self.status = tk.Label(self.root, text='Applied wind: (0, 0, 0) m/s')
        self.status.grid(row=5, columnspan=2)
        self.root.protocol('WM_DELETE_WINDOW', self.request_close)
        self.root.bind('<Escape>', lambda event: self.request_close())

    def request_close(self):
        self.closed = True

    def apply(self):
        try:
            self.pending = wind_settings(self.enabled.get(), [v.get() for v in self.components])
            self.status.configure(text='Applying…')
        except (ValueError, ConfigurationError) as error:
            self.status.configure(text=f'Invalid wind: {error}')

    def update(self, simulation, renderer, observation):
        now = time.monotonic()
        if now < self.next_update:
            return
        self.next_update = now + UI_INTERVAL_S
        self.root.update()
        if self.closed:
            raise KeyboardInterrupt
        if self.pending is not None:
            simulation.set_wind(self.pending)
            self.pending = None
            self.request_id = str(uuid4())
            self.status.configure(text=f'Applied wind: {simulation.wind.velocity_world_m_s} m/s')
        publish_wind(simulation, renderer, observation, self.request_id)

    def close(self):
        self.root.destroy()
