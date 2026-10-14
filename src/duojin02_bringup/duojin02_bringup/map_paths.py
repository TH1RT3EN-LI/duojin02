import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from duojin02_slam_tools.snapshots import validate_map


_PACKAGE_NAME = "duojin02_bringup"


def get_workspace_root(package_name: str = _PACKAGE_NAME) -> Path:
    configured_root = os.environ.get("DUOJIN02_WORKSPACE_ROOT", "").strip()
    if configured_root:
        root = Path(configured_root).expanduser()
        if not root.is_absolute():
            raise RuntimeError("DUOJIN02_WORKSPACE_ROOT must be an absolute path")
        return root.resolve(strict=False)

    package_prefix = Path(get_package_prefix(package_name))

    for candidate in (package_prefix, *package_prefix.parents):
        if candidate.name == "install" and candidate.parent != candidate:
            return candidate.parent

    raise RuntimeError(
        f"Failed to resolve workspace root from package prefix: {package_prefix}"
    )


def get_workspace_maps_dir(package_name: str = _PACKAGE_NAME) -> Path:
    return get_workspace_root(package_name) / "maps"


def pick_latest_map_yaml(package_name: str = _PACKAGE_NAME) -> str:
    maps_dir = get_workspace_maps_dir(package_name)
    numeric_maps = []
    newest_map = None
    newest_mtime = -1.0

    if maps_dir.is_dir():
        for path in maps_dir.glob("*.yaml"):
            if not path.is_file():
                continue
            if path.stem.isdigit():
                numeric_maps.append((int(path.stem), path))
            mtime = path.stat().st_mtime
            if mtime > newest_mtime:
                newest_mtime = mtime
                newest_map = path

    if numeric_maps:
        numeric_maps.sort(key=lambda item: item[0])
        validate_map(numeric_maps[-1][1])
        return str(numeric_maps[-1][1])
    if newest_map is not None:
        validate_map(newest_map)
        return str(newest_map)

    raise FileNotFoundError(
        f"No map yaml found under workspace root maps directory: {maps_dir}. "
        f"Place maps in {maps_dir} or pass an explicit map:=... path."
    )


def resolve_map_yaml(map_value: str, package_name: str = _PACKAGE_NAME) -> str:
    raw_value = map_value.strip()
    if not raw_value:
        return pick_latest_map_yaml(package_name)

    path = Path(raw_value).expanduser()
    if not path.is_absolute():
        path = get_workspace_root(package_name) / path

    resolved = path.resolve(strict=False)
    if not resolved.is_file():
        raise FileNotFoundError(f"Map yaml not found: {resolved}")
    validate_map(resolved)
    return str(resolved)
