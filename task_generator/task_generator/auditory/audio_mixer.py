from __future__ import annotations

from importlib.resources import path
import threading
from dataclasses import dataclass

import numpy as np
import sounddevice as sd

from task_generator.auditory.asset_lib import CachedSample


@dataclass
class Voice:
    sample: CachedSample
    position: int = 0
    loop: bool = False
    gain: float = 1.0


class AudioMixer:
    def __init__(self,*,sample_rate: int = 44100,channels: int = 2, block_size: int = 2048, device: str | int | None = None, master_gain_db: float = 0.0) -> None:
        self._channels = channels
        self._voices: list[Voice] = []
        self._lock = threading.Lock()
        self._master_gain = 10.0 ** (master_gain_db / 20.0)

        self._stream = sd.OutputStream(samplerate=sample_rate,channels=channels,dtype="float32", blocksize=block_size, device=device, callback=self._callback)
        self._stream.start()

    def play(self,sample: CachedSample,*,loop: bool = False,gain_db: float = 0.0) -> None:
        voice = Voice(
            sample=sample,
            loop=loop,
            gain=10.0 ** (gain_db / 20.0),
        )
        with self._lock:
            self._voices.append(voice)

    def stop_all(self) -> None:
        with self._lock:
            self._voices.clear()

    def close(self) -> None:
        self._stream.stop()
        self._stream.close()

    def _callback(self, outdata, frames, time_info, status) -> None:
        del time_info
        if status:
            print(f"audio callback status: {status}", flush=True)
        outdata.fill(0.0)

        with self._lock:
            active: list[Voice] = []

            for voice in self._voices:
                written = 0

                # if voice.sample.samples.size == 0 or len(voice.sample.samples) == 0:
                #     raise ValueError(f"empty WAV file: {voice.sample.path}")

                while written < frames:
                    remaining = len(voice.sample.samples) - voice.position
                    count = min(frames - written, remaining)

                    if count > 0:
                        outdata[written:written + count] += (
                            voice.sample.samples[
                                voice.position:voice.position + count
                            ]
                            * voice.gain
                        )
                        voice.position += count
                        written += count

                    if voice.position >= len(voice.sample.samples):
                        if voice.loop:
                            voice.position = 0
                        else:
                            break

                if voice.loop or voice.position < len(voice.sample.samples):
                    active.append(voice)

            self._voices = active

        outdata *= self._master_gain
        np.clip(outdata, -1.0, 1.0, out=outdata)