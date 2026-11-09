#!/usr/bin/env python3

import glob
import os

import laspy
import numpy as np


# ============================================================
# CONFIG
# ============================================================

BASE_DIR = (
    "/home/janusz/hdmapping-benchmark-loop-closure/benchmark-PIN-SLAM-to-HDMapping"
)

OUTPUT_DIR = os.path.join(
    BASE_DIR,
    "laz"
)

SOURCE_GLOB = os.path.join(
    BASE_DIR,
    "loop_*_current_*_global.pcd"
)

TARGET_GLOB = os.path.join(
    BASE_DIR,
    "loop_*_history_*_global.pcd"
)


# ============================================================
# PCD READER
# ============================================================

def read_pcd_binary(filename):

    with open(filename, "rb") as f:

        fields = None
        sizes = None
        counts = None
        points = None
        data_type = None
        width = None
        height = None

        while True:

            line = f.readline()

            if not line:
                raise RuntimeError(
                    f"Niepoprawny lub uszkodzony PCD: {filename}"
                )

            text = line.decode(
                "ascii",
                errors="ignore"
            ).strip()

            parts = text.split()

            if not parts:
                continue

            key = parts[0]

            if key == "FIELDS":
                fields = parts[1:]

            elif key == "SIZE":
                sizes = list(
                    map(int, parts[1:])
                )

            elif key == "COUNT":
                counts = list(
                    map(int, parts[1:])
                )

            elif key == "WIDTH":
                width = int(parts[1])

            elif key == "HEIGHT":
                height = int(parts[1])

            elif key == "POINTS":
                points = int(parts[1])

            elif key == "DATA":
                data_type = parts[1]
                break

        if data_type != "binary":
            raise RuntimeError(
                f"{filename}: obsługiwane jest tylko DATA binary"
            )

        if fields != [
            "x",
            "y",
            "z",
            "intensity"
        ]:
            raise RuntimeError(
                f"{filename}: nieoczekiwane FIELDS: {fields}"
            )

        if sizes != [
            4, 4, 4, 4
        ]:
            raise RuntimeError(
                f"{filename}: nieoczekiwane SIZE: {sizes}"
            )

        if counts is None:
            counts = [1] * len(fields)

        if counts != [
            1, 1, 1, 1
        ]:
            raise RuntimeError(
                f"{filename}: nieoczekiwane COUNT: {counts}"
            )

        if points is None:

            if width is None or height is None:
                raise RuntimeError(
                    f"{filename}: brak POINTS/WIDTH/HEIGHT"
                )

            points = width * height

        point_step = 16

        raw = f.read(
            points * point_step
        )

    expected_bytes = (
        points * point_step
    )

    if len(raw) != expected_bytes:
        raise RuntimeError(
            f"{filename}: niepełne dane "
            f"({len(raw)} / {expected_bytes} bajtów)"
        )

    dtype = np.dtype([
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
    ])

    return np.frombuffer(
        raw,
        dtype=dtype
    )


# ============================================================
# PCD -> XYZ
# ============================================================

def read_pcd(filename):

    data = read_pcd_binary(
        filename
    )

    xyz = np.column_stack([
        data["x"],
        data["y"],
        data["z"]
    ]).astype(
        np.float64
    )

    intensity = (
        data["intensity"]
        .astype(np.float64)
    )

    valid = np.isfinite(
        xyz
    ).all(axis=1)

    xyz = xyz[valid]
    intensity = intensity[valid]

    return xyz, intensity


# ============================================================
# WRITE LAZ
# ============================================================

def write_laz(
    filename,
    xyz,
    intensity
):

    if len(xyz) == 0:
        raise RuntimeError(
            f"Pusta chmura: {filename}"
        )

    header = laspy.LasHeader(
        point_format=3,
        version="1.2"
    )

    header.scales = np.array([
        0.001,
        0.001,
        0.001
    ])

    header.offsets = xyz.min(
        axis=0
    )

    las = laspy.LasData(
        header
    )

    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]

    las.intensity = np.clip(
        intensity,
        0,
        65535
    ).astype(
        np.uint16
    )

    try:

        las.write(
            filename
        )

    except Exception as exc:

        raise RuntimeError(
            "Nie można zapisać LAZ.\n"
            "Zainstaluj backend lazrs:\n"
            "python3 -m pip install lazrs\n\n"
            f"Oryginalny błąd:\n{exc}"
        ) from exc


# ============================================================
# ONE PCD -> ONE LAZ
# ============================================================

def convert_one(
    pcd_file,
    laz_file
):

    print()
    print(
        "[PCD -> LAZ]"
    )

    print(
        "  input :",
        pcd_file
    )

    print(
        "  output:",
        laz_file
    )

    xyz, intensity = read_pcd(
        pcd_file
    )

    write_laz(
        laz_file,
        xyz,
        intensity
    )

    print(
        "  points:",
        len(xyz)
    )

    print(
        "  size:",
        os.path.getsize(laz_file),
        "bytes"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 70)
    print("PCD -> LAZ")
    print("1 PCD = 1 LAZ")
    print("=" * 70)

    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )

    # --------------------------------------------------------
    # FIND CURRENT / SOURCE
    # --------------------------------------------------------

    source_files = sorted(
        glob.glob(
            SOURCE_GLOB
        )
    )

    # --------------------------------------------------------
    # FIND HISTORY / TARGET
    # --------------------------------------------------------

    target_files = sorted(
        glob.glob(
            TARGET_GLOB
        )
    )

    print()
    print(
        "SOURCE PCD:",
        len(source_files)
    )

    print(
        "TARGET PCD:",
        len(target_files)
    )

    print()

    # --------------------------------------------------------
    # SOURCE
    # --------------------------------------------------------

    for pcd_file in source_files:

        basename = os.path.basename(
            pcd_file
        )

        stem = os.path.splitext(
            basename
        )[0]

        laz_file = os.path.join(
            OUTPUT_DIR,
            stem + ".laz"
        )

        convert_one(
            pcd_file,
            laz_file
        )

    # --------------------------------------------------------
    # TARGET
    # --------------------------------------------------------

    for pcd_file in target_files:

        basename = os.path.basename(
            pcd_file
        )

        stem = os.path.splitext(
            basename
        )[0]

        laz_file = os.path.join(
            OUTPUT_DIR,
            stem + ".laz"
        )

        convert_one(
            pcd_file,
            laz_file
        )

    # --------------------------------------------------------
    # SUMMARY
    # --------------------------------------------------------

    print()
    print("=" * 70)
    print("GOTOWE")
    print("=" * 70)

    print(
        "Wyniki:",
        OUTPUT_DIR
    )

    print()

    laz_files = sorted(
        glob.glob(
            os.path.join(
                OUTPUT_DIR,
                "*.laz"
            )
        )
    )

    print(
        "LAZ files:",
        len(laz_files)
    )

    print("=" * 70)


if __name__ == "__main__":
    main()