"""Standalone camera window: descriptors over ZeroMQ, pixel copies from SHM."""
import json
from pathlib import Path
import sys
import time

import click
from loguru import logger


IMAGE_TOPIC = 'camera/front/image'
POLL_INTERVAL_MS = 15
MAX_PENDING_FRAMES = 4
CHANNELS = 3
EXIT_FAILURE = 1
FPS_INTERVAL_S = 1.0


class FpsCounter:
    """Count images displayed per wall-clock second, including idle intervals."""
    def __init__(self, now):
        self.started = now
        self.frames = 0

    def update(self, now, *, displayed):
        self.frames += int(displayed)
        elapsed = now - self.started
        if elapsed < FPS_INTERVAL_S:
            return None
        fps = self.frames / elapsed
        self.started = now
        self.frames = 0
        return fps


class CliParser:
    def parse(self, argv=None):
        @click.command()
        @click.option('--render-config', required=True,
                      type=click.Path(exists=True, dir_okay=False, path_type=Path))
        def command(render_config):
            """Display the renderer camera in an independent process. Close or press Escape to exit."""
            return render_config
        return command.main(args=argv, standalone_mode=False)


def ppm_frame(metadata, pixels):
    """Convert validated RGB/BGR rows, excluding padding, to Tk's binary PPM."""
    width, height, stride = metadata['width'], metadata['height'], metadata['step']
    row_bytes = width * CHANNELS
    rgb = b''.join(pixels[offset:offset + row_bytes]
                   for offset in range(0, stride * height, stride))
    if metadata['encoding'] == 'bgr8':
        converted = bytearray(rgb)
        converted[0::CHANNELS] = rgb[2::CHANNELS]
        converted[2::CHANNELS] = rgb[0::CHANNELS]
        rgb = bytes(converted)
    return f'P6\n{width} {height}\n255\n'.encode() + rgb


def run_viewer(render_config):
    import tkinter as tk
    import zmq
    from bullet_simulation_protocol import load_transport
    from bullet_image_transport.shm import FrameReader, topic_prefix

    path = Path(render_config).resolve()
    config = json.loads(path.read_text())
    transport = load_transport(path.parent / config['transport_config'])
    root = tk.Tk()
    root.title('Bullet camera — shared memory')
    label = tk.Label(root, text='Waiting for camera frames…')
    label.pack()
    fps_label = tk.Label(root, text="Display FPS: 0.0")
    fps_label.pack()
    fps_counter = FpsCounter(time.monotonic())
    root.bind('<Escape>', lambda event: root.destroy())
    reader = FrameReader(IMAGE_TOPIC)
    failure = []
    try:
        with zmq.Context() as context, context.socket(zmq.SUB) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, MAX_PENDING_FRAMES)
            socket.setsockopt(zmq.SUBSCRIBE, topic_prefix(IMAGE_TOPIC))
            socket.connect(transport['image_endpoint'])

            def refresh():
                try:
                    frame = None
                    for _ in range(MAX_PENDING_FRAMES):
                        if not socket.poll(0):
                            break
                        candidate = reader.read(socket.recv())
                        if candidate is not None:
                            frame = candidate
                    if frame is not None:
                        metadata, pixels = frame
                        image = tk.PhotoImage(data=ppm_frame(metadata, pixels), format='PPM')
                        label.configure(image=image, text='')
                        label.image = image
                    fps = fps_counter.update(time.monotonic(), displayed=frame is not None)
                    if fps is not None:
                        fps_label.configure(text=f"Display FPS: {fps:.1f}")
                    root.after(POLL_INTERVAL_MS, refresh)
                except Exception as error:
                    failure.append(error)
                    root.destroy()

            refresh()
            root.mainloop()
            if failure:
                raise failure[0]
    finally:
        reader.close()
        try:
            root.destroy()
        except tk.TclError:
            pass


def main(argv=None):
    logger.remove()
    logger.add(sys.stderr, level='INFO', diagnose=False)
    try:
        options = CliParser().parse(argv)
        if isinstance(options, int):
            return options
        run_viewer(options)
        return 0
    except click.ClickException as error:
        error.show()
        return error.exit_code
    except KeyboardInterrupt:
        return 130
    except Exception:
        logger.exception('Camera viewer failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
