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
    model_id: str
    category: str
    caption: str
    num_bricks: int
    json_path: Path

BRICKSIM_DATASET_URL = "https://www.cs.cmu.edu/~haoweiw/bricksim_downloads/bricksim_dataset.tar.xz"
BRICKSIM_DATASET_PATH = (
    Path(os.environ["BRICKSIM_DATASET_PATH"])
    if "BRICKSIM_DATASET_PATH" in os.environ
    else Path(__file__).resolve().parents[3] / "resources" / "bricksim_dataset"
)
BRICKSIM_DATASET_CATALOG_PATH = BRICKSIM_DATASET_PATH / "data" / "lego" / "data" / "simulator_testset" / "dataset.json"

def is_bricksim_dataset_available() -> bool:
    return BRICKSIM_DATASET_CATALOG_PATH.exists()

async def download_bricksim_dataset() -> None:
    if is_bricksim_dataset_available():
        return
    print(f"Downloading BrickSim dataset from {BRICKSIM_DATASET_URL} to {BRICKSIM_DATASET_PATH}...")
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

async def load_bricksim_dataset(download_if_not_available: bool = True) -> dict[str, BricksimDatasetItem]:
    global _LOADED_BRICKSIM_DATASET
    if _LOADED_BRICKSIM_DATASET is not None:
        return _LOADED_BRICKSIM_DATASET
    if not is_bricksim_dataset_available():
        if download_if_not_available:
            await download_bricksim_dataset()
        else:
            raise RuntimeError("Bricksim dataset not available. Set download_if_not_available=True to download it automatically.")
    with open(BRICKSIM_DATASET_CATALOG_PATH, "r") as f:
        catalog = json.load(f)
    items: dict[str, BricksimDatasetItem] = dict()
    for category, models in catalog.items():
        for model_id, paths in models.items():
            for json_key, meta in paths.items():
                num_bricks = int(meta.get("num_bricks"))
                caption = str(meta.get("caption"))
                json_fname = str(meta.get("json_fname"))
                json_path = BRICKSIM_DATASET_PATH / json_fname.lstrip("/")
                items[model_id] = BricksimDatasetItem(
                    model_id=model_id,
                    category=category,
                    caption=caption,
                    num_bricks=num_bricks,
                    json_path=json_path,
                )
    _LOADED_BRICKSIM_DATASET = items
    return _LOADED_BRICKSIM_DATASET

def get_bricksim_dataset() -> dict[str, BricksimDatasetItem]:
    global _LOADED_BRICKSIM_DATASET
    if _LOADED_BRICKSIM_DATASET is None:
        raise RuntimeError("Bricksim dataset not loaded. Call load_bricksim_dataset() first.")
    return _LOADED_BRICKSIM_DATASET
