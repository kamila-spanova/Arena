from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class AcousticMaterial:
    material_id: str
    canonical_name: str
    absorption: tuple[float, ...]
    transmission_loss_db: tuple[float, ...]
    scattering: float


class AcousticMaterialCatalog:
    def __init__(self, path: Path) -> None:
        raw = yaml.safe_load(path.read_text())
        self._default = raw["defaults"]
        self._materials = raw.get("materials", {})

    def get(self, material_id: str) -> AcousticMaterial:
        entry = self._materials.get(material_id, self._default)
        return AcousticMaterial(
            material_id=material_id,
            canonical_name=str(
                entry.get("canonical_name", material_id)
            ),
            absorption=tuple(entry["absorption"]),
            transmission_loss_db=tuple(
                entry["transmission_loss_db"]
            ),
            scattering=float(entry.get("scattering", 0.1)),
        )