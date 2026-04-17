"""Download and load the BrickSim structures dataset."""

import json
import os
import tarfile
import tempfile
from dataclasses import dataclass
from pathlib import Path

import requests
import tqdm


@dataclass
class BricksimDatasetItem:
    """Metadata for one BrickSim structures dataset item."""

    model_id: str
    category: str
    caption: str
    num_bricks: int
    json_path: Path
    all_connected_no_plate: bool
    all_connected_with_plate: bool
    stable_with_plate: bool
    max_stability_score_with_plate: float


def _resolve_bricksim_dataset_path() -> Path:
    dataset_path = os.environ.get("BRICKSIM_DATASET_PATH")
    if dataset_path:
        return Path(dataset_path).expanduser()
    env_xdg_cache_home = os.environ.get("XDG_CACHE_HOME")
    if env_xdg_cache_home:
        cache_home = Path(env_xdg_cache_home).expanduser()
    else:
        cache_home = Path.home() / ".cache"
    return cache_home / "bricksim" / "bricksim_dataset"


BRICKSIM_DATASET_URL = (
    "https://www.cs.cmu.edu/~haoweiw/bricksim_downloads/bricksim_dataset.tar.xz"
)
BRICKSIM_DATASET_PATH = _resolve_bricksim_dataset_path()
BRICKSIM_DATASET_CATALOG_PATH = (
    BRICKSIM_DATASET_PATH
    / "data"
    / "lego"
    / "data"
    / "simulator_testset"
    / "dataset.json"
)


def is_bricksim_dataset_available() -> bool:
    """Return whether the BrickSim structures dataset is available locally.

    Returns:
        ``True`` if the dataset catalog exists.
    """
    return BRICKSIM_DATASET_CATALOG_PATH.exists()


async def download_bricksim_dataset() -> None:
    """Download and extract the BrickSim structures dataset if missing."""
    if is_bricksim_dataset_available():
        return
    BRICKSIM_DATASET_PATH.mkdir(parents=True, exist_ok=True)
    print(
        f"Downloading BrickSim dataset from {BRICKSIM_DATASET_URL} to "
        f"{BRICKSIM_DATASET_PATH}..."
    )
    with tempfile.NamedTemporaryFile(suffix=".tar.xz") as archive_file:
        with requests.get(BRICKSIM_DATASET_URL, stream=True) as res:
            res.raise_for_status()
            total_size = int(res.headers.get("Content-Length", 0))
            with tqdm.tqdm(total=total_size, unit="B", unit_scale=True) as pbar:
                for chunk in res.iter_content(chunk_size=8192):
                    archive_file.write(chunk)
                    pbar.update(len(chunk))
        archive_file.flush()
        with tarfile.open(archive_file.name, "r:xz") as tar:
            tar.extractall(path=BRICKSIM_DATASET_PATH)
        print("Extracted BrickSim dataset.")
    if not is_bricksim_dataset_available():
        raise RuntimeError("Failed to download and extract the BrickSim dataset.")


_LOADED_BRICKSIM_DATASET: dict[str, BricksimDatasetItem] | None = None


async def load_bricksim_dataset(
    download_if_not_available: bool = True,
) -> dict[str, BricksimDatasetItem]:
    """Load the BrickSim structures dataset catalog.

    Returns:
        Dataset items keyed by model id.
    """
    global _LOADED_BRICKSIM_DATASET
    if _LOADED_BRICKSIM_DATASET is not None:
        return _LOADED_BRICKSIM_DATASET
    if not is_bricksim_dataset_available():
        if download_if_not_available:
            await download_bricksim_dataset()
        else:
            raise RuntimeError(
                "Bricksim dataset not available. Set "
                "download_if_not_available=True to download it automatically."
            )
    with open(BRICKSIM_DATASET_CATALOG_PATH, "r") as f:
        catalog = json.load(f)
    items: dict[str, BricksimDatasetItem] = dict()
    for category, models in catalog.items():
        for model_id, paths in models.items():
            for json_key, meta in paths.items():
                json_fname = str(meta.get("json_fname"))
                json_path = BRICKSIM_DATASET_PATH / json_fname.lstrip("/")
                items[model_id] = BricksimDatasetItem(
                    model_id=model_id,
                    category=category,
                    caption=str(meta.get("caption")),
                    num_bricks=int(meta.get("num_bricks")),
                    json_path=json_path,
                    all_connected_no_plate=bool(meta.get("all_connected_no_plate")),
                    all_connected_with_plate=bool(meta.get("all_connected_with_plate")),
                    stable_with_plate=bool(meta.get("stable_with_plate")),
                    max_stability_score_with_plate=float(
                        meta.get("max_stability_score_with_plate")
                    ),
                )
    _LOADED_BRICKSIM_DATASET = items
    return _LOADED_BRICKSIM_DATASET


def get_bricksim_dataset() -> dict[str, BricksimDatasetItem]:
    """Return the previously loaded BrickSim structures dataset.

    Returns:
        Dataset items keyed by model id.
    """
    global _LOADED_BRICKSIM_DATASET
    if _LOADED_BRICKSIM_DATASET is None:
        raise RuntimeError(
            "Bricksim dataset not loaded. Call load_bricksim_dataset() first."
        )
    return _LOADED_BRICKSIM_DATASET
