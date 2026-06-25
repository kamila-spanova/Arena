from __future__ import annotations
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import yaml
from scipy.io import wavfile
from scipy.signal import resample_poly
from task_generator.auditory.octave_bands import calculate_octave_band_levels_db


@dataclass(frozen=True)
class CachedSample:
    sample_id: str
    path: Path
    samples: np.ndarray
    sample_rate: int
    channels: int
    duration_sec: float
    normalization_dbfs: float
    tags: frozenset[str]
    octave_band_levels_db: dict[int, float]


@dataclass(frozen=True)
class AcousticAsset:
    asset_id: str
    category: str
    semantic_tags: frozenset[str]
    reference_level_db: float
    reference_distance_m: float
    loop: bool
    variants: tuple[CachedSample, ...]
    playback_gain_db: float = 0.0


class AcousticAssetCatalog:
    def __init__(
        self,
        config_path: Path | str,
        sound_dir: Path | str,
        *,
        output_sample_rate: int = 44100,
        output_channels: int = 2,
    ) -> None:
        config_path = Path(config_path)
        sound_dir = Path(sound_dir)

        raw = yaml.safe_load(config_path.read_text())
        self._assets: dict[str, AcousticAsset] = {}

        for asset_id, entry in raw.get("assets", {}).items():
            normalization_dbfs = float(entry.get("normalization_dbfs", -6.0))

            variants = tuple(
                self._decode_sample(
                    sample_id=str(variant["sample_id"]),
                    path=sound_dir / variant["file"],
                    tags=frozenset(map(str, variant.get("tags", []))),
                    target_rate=output_sample_rate,
                    target_channels=output_channels,
                    normalization_dbfs=normalization_dbfs,
                    octave_band_levels_db=variant.get("octave_band_levels_db", "auto"),
                )
                for variant in entry.get("variants", [])
            )

            if not variants:
                raise ValueError(f"asset {asset_id!r} has no variants")

            self._assets[asset_id] = AcousticAsset(
                asset_id=asset_id,
                category=str(entry["category"]),
                semantic_tags=frozenset(
                    map(str, entry.get("semantic_tags", []))
                ),
                reference_level_db=float(entry["reference_level_db"]),
                reference_distance_m=float(
                    entry.get("reference_distance_m", 1.0)
                ),
                playback_gain_db=float(entry.get("playback_gain_db", 0.0)),
                loop=bool(entry.get("loop", False)),
                variants=variants,
            )

    def get(self, asset_id: str) -> AcousticAsset | None:
        return self._assets.get(asset_id)

    def select(
        self,
        asset_id: str,
        *,
        episode_seed: int,
        agent_id: int,
        occurrence: int,
        required_tags: frozenset[str] = frozenset(),
    ) -> tuple[AcousticAsset, CachedSample] | None:
        asset = self.get(asset_id)
        if asset is None:
            return None

        candidates = tuple(
            sample
            for sample in asset.variants
            if required_tags.issubset(sample.tags)
        ) or asset.variants

        key = f"{episode_seed}:{agent_id}:{asset_id}:{occurrence}"
        digest = hashlib.blake2b(key.encode(), digest_size=8).digest()
        index = int.from_bytes(digest, "big") % len(candidates)

        return asset, candidates[index]

    @classmethod
    def _decode_sample(cls,*, sample_id: str, path: Path, tags: frozenset[str], target_rate: int,target_channels: int, normalization_dbfs: float, octave_band_levels_db: dict | str | None = "auto") -> CachedSample:
        source_rate, data = wavfile.read(path)
        samples = cls._to_float32(data)

        if samples.ndim == 1:
            samples = samples[:, None]

        if samples.shape[1] == 1 and target_channels == 2:
            samples = np.repeat(samples, 2, axis=1)
        elif samples.shape[1] > target_channels:
            samples = samples[:, :target_channels]

        if source_rate != target_rate:
            divisor = math.gcd(source_rate, target_rate)
            samples = resample_poly(
                samples,
                target_rate // divisor,
                source_rate // divisor,
                axis=0,
            ).astype(np.float32)

        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        target_peak = 10.0 ** (normalization_dbfs / 20.0)

        if peak > 0.0:
            samples *= target_peak / peak

        samples = np.ascontiguousarray(samples, dtype=np.float32)
        
        if octave_band_levels_db in (None, "auto"):
            measured_bands = calculate_octave_band_levels_db(
                samples,
                target_rate,
            )
        else:
            measured_bands = {
                int(frequency): float(level)
                for frequency, level in octave_band_levels_db.items()
            }

        if samples.size == 0 or len(samples) == 0:
            raise ValueError(f"empty WAV file: {path}")
        
        return CachedSample(
            sample_id=sample_id,
            path=path,
            samples=samples,
            sample_rate=target_rate,
            channels=samples.shape[1],
            duration_sec=len(samples) / target_rate,
            normalization_dbfs=normalization_dbfs,
            tags=tags,
            octave_band_levels_db=measured_bands,
        )
    
    def require(self, asset_id: str) -> AcousticAsset:
        asset = self.get(asset_id)
        if asset is None:
            raise KeyError(f"unknown acoustic asset_id={asset_id!r}")
        return asset

    @staticmethod
    def _to_float32(data: np.ndarray) -> np.ndarray:
        if np.issubdtype(data.dtype, np.floating):
            return data.astype(np.float32)

        if data.dtype == np.uint8:
            return ((data.astype(np.float32) - 128.0) / 128.0)

        info = np.iinfo(data.dtype)
        scale = float(max(abs(info.min), info.max))
        return data.astype(np.float32) / scale