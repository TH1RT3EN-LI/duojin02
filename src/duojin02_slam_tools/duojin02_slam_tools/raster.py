from collections import Counter
from pathlib import Path


def _token(stream):
    value = bytearray()
    while True:
        character = stream.read(1)
        if not character:
            if value:
                return bytes(value)
            raise ValueError("Truncated map PGM header")
        if character == b"#" and not value:
            stream.readline()
        elif character.isspace():
            if value:

                if character == b"\r":
                    position = stream.tell()
                    if stream.read(1) != b"\n":
                        stream.seek(position)
                return bytes(value)
        else:
            value.extend(character)


def validate_trinary_raster(metadata, image):

    if (
        metadata.get("mode") != "trinary"
        or metadata.get("negate") != 0
        or metadata.get("free_thresh") != 0.196
        or metadata.get("occupied_thresh") != 0.65
    ):
        raise ValueError(
            "New Toolbox maps require trinary PGM thresholds .196/.65 to preserve unknown"
        )
    with Path(image).open("rb") as stream:
        if _token(stream) != b"P5":
            raise ValueError("New Toolbox map image must be a binary PGM")
        try:
            width, height, maximum = (int(_token(stream)) for _ in range(3))
        except ValueError as error:
            raise ValueError("Invalid map PGM geometry") from error
        if width <= 0 or height <= 0 or maximum != 255:
            raise ValueError("Invalid map PGM geometry")
        pixels = stream.read()
    if len(pixels) != width * height:
        raise ValueError("Truncated or oversized map PGM raster")
    counts = Counter(pixels)
    if counts.keys() - {0, 205, 254}:
        raise ValueError("New Toolbox trinary map contains unexpected grayscale values")
    expected = {0: "occupied", 205: "unknown", 254: "free"}
    for gray in counts:
        occupancy = 1.0 - gray / 255.0
        loaded = (
            "occupied"
            if occupancy > metadata["occupied_thresh"]
            else "free" if occupancy < metadata["free_thresh"] else "unknown"
        )
        if loaded != expected[gray]:
            raise ValueError("Saved map reload changes an occupancy class")
    return {
        "contract": "toolbox_trinary_pgm_v1",
        "width": width,
        "height": height,
        "pixels": len(pixels),
        "loaded_class_counts": {name: counts[gray] for gray, name in expected.items()},
        "source_comparison": "Saved raster only; dynamic_map and SaveMap are not an atomic pair",
        "origin_modified": False,
    }
