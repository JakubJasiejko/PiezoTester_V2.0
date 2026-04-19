#!/usr/bin/env python3
from __future__ import annotations

import csv
import re
from dataclasses import dataclass, field
from decimal import Decimal, getcontext
from pathlib import Path


REF_IGNORE_PREFIXES = ("TP", "T", "NT", "REF", "G", "H")

# Based on JLCPCB's KiJLC plugin:
# https://github.com/JLCPCB/KiJLC
ROTATION_OFFSETS = {
    r"^SOT-223": Decimal("180"),
    r"^SOT-23": Decimal("180"),
    r"^SOT-353": Decimal("180"),
    r"^QFN-": Decimal("270"),
    r"^LQFP-": Decimal("270"),
    r"^TQFP-": Decimal("270"),
    r"^SOP-(?!18_)": Decimal("270"),
    r"^TSSOP-": Decimal("270"),
    r"^SOIC-": Decimal("90"),
    r"^SOP-18_": Decimal("0"),
    r"^VSSOP-10_": Decimal("270"),
    r"^CP_EIA-3216-18_": Decimal("180"),
    r"^CP_Elec_8x10\.5": Decimal("180"),
    r"^CP_Elec_6\.3x7\.7": Decimal("180"),
    r"^CP_Elec_8x6\.7": Decimal("180"),
    r"^(.*?_|V)?QFN-(16|20|24|28|40)(-|_|$)": Decimal("270"),
    r"^MSOP-10_": Decimal("90"),
    r"^R_Array_Convex_4x0603": Decimal("90"),
    r"^XCVR_ESP32-WROVER-B": Decimal("270"),
    r"^PinSocket_1x(04|05)_P2\.54mm_Vertical": Decimal("270"),
    r"Buzzer_MLT-8530_C94599": Decimal("0"),
    r"SW_Tactile_SPST_Angled_PTS645Vx58-2LFS": Decimal("180"),
    r"USB_C_Receptacle_HRO_TYPE-C-31-M-12": Decimal("180"),
    r"USB_Micro-B_Molex-105017-0001": Decimal("270"),
}

MIDPOINT_CORRECTIONS = {
    r"^PinSocket_1x04_P2\.54mm_Vertical": (Decimal("0"), Decimal("-3.81")),
    r"^PinSocket_1x05_P2\.54mm_Vertical": (Decimal("0"), Decimal("-5.08")),
    r"^XCVR_ESP32-WROVER-B": (Decimal("0"), Decimal("0.04")),
    r"BarrelJack": (Decimal("-6.5"), Decimal("0")),
    r"^SW_SPST_HRO": (Decimal("0"), Decimal("1.65")),
    r"USB_C_Receptacle_HRO_TYPE-C-31-M-12": (Decimal("1.8"), Decimal("0.65")),
    r"SW_Tactile_SPST_Angled_PTS645Vx58-2LFS": (Decimal("2.2"), Decimal("-1")),
}

PROPERTY_RE = re.compile(r'\(property\s+"((?:[^"\\]|\\.)*)"\s+"((?:[^"\\]|\\.)*)"')
LAYER_RE = re.compile(r'\(layer\s+"([^"]+)"\)')
AT_RE = re.compile(r'\(at\s+([-0-9.]+)\s+([-0-9.]+)(?:\s+([-0-9.]+))?\)')
ATTR_RE = re.compile(r'\(attr\s+([^)]+)\)')


def unescape_quoted(value: str) -> str:
    return value.replace(r"\\", "\\").replace(r"\"", '"')


def format_decimal(value: Decimal) -> str:
    normalized = value.normalize()
    text = format(normalized, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text or "0"


def sanitize_csv_text(value: str) -> str:
    cleaned = value.replace("\u00a0", " ").replace("µ", "u").replace("μ", "u")
    return re.sub(r"\s+", " ", cleaned).strip()


def natural_sort_key(value: str) -> list[object]:
    return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", value)]


def normalize_rotation(rotation: Decimal) -> Decimal:
    value = rotation % Decimal("360")
    if value < 0:
        value += Decimal("360")
    return value


def pi() -> Decimal:
    getcontext().prec += 2
    three = Decimal(3)
    lasts, term, total, n, n_add, d, d_add = 0, three, 3, 1, 0, 0, 24
    while total != lasts:
        lasts = total
        n, n_add = n + n_add, n_add + 8
        d, d_add = d + d_add, d_add + 32
        term = (term * n) / d
        total += term
    getcontext().prec -= 2
    return +total


def sin(x: Decimal) -> Decimal:
    getcontext().prec += 2
    idx, lasts, total, fact, num, sign = 1, 0, x, 1, x, 1
    while total != lasts:
        lasts = total
        idx += 2
        fact *= idx * (idx - 1)
        num *= x * x
        sign *= -1
        total += (num / fact) * sign
    getcontext().prec -= 2
    return +total


def cos(x: Decimal) -> Decimal:
    getcontext().prec += 2
    idx, lasts, total, fact, num, sign = 0, 0, 1, 1, 1, 1
    while total != lasts:
        lasts = total
        idx += 2
        fact *= idx * (idx - 1)
        num *= x * x
        sign *= -1
        total += (num / fact) * sign
    getcontext().prec -= 2
    return +total


@dataclass
class Footprint:
    footprint_id: str
    layer: str
    ref: str
    value: str
    x_mm: Decimal
    y_mm: Decimal
    rotation_deg: Decimal
    attrs: set[str] = field(default_factory=set)
    properties: dict[str, str] = field(default_factory=dict)

    @property
    def local_name(self) -> str:
        return self.footprint_id.split(":", 1)[-1]

    def is_smd(self) -> bool:
        return "smd" in self.attrs

    def is_position_excluded(self) -> bool:
        return "exclude_from_pos_files" in self.attrs

    def is_bom_excluded(self) -> bool:
        return "exclude_from_bom" in self.attrs

    def is_ignored_reference(self) -> bool:
        return self.ref.startswith(REF_IGNORE_PREFIXES) or self.ref.startswith("#")

    def is_test_feature(self) -> bool:
        return (
            "testpoint" in self.footprint_id.lower()
            or self.value.lower() == "testpoint"
            or "fiducial" in self.footprint_id.lower()
        )

    def include_for_cpl(self) -> bool:
        return (
            self.is_smd()
            and not self.is_position_excluded()
            and not self.is_bom_excluded()
            and not self.is_ignored_reference()
            and not self.is_test_feature()
        )

    def include_for_bom(self) -> bool:
        return self.include_for_cpl()


def extract_blocks(text: str, head: str) -> list[str]:
    blocks: list[str] = []
    idx = 0
    marker = f"({head} "
    while True:
        start = text.find(marker, idx)
        if start < 0:
            break
        depth = 0
        in_string = False
        escaped = False
        pos = start
        while pos < len(text):
            char = text[pos]
            if in_string:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    in_string = False
            else:
                if char == '"':
                    in_string = True
                elif char == "(":
                    depth += 1
                elif char == ")":
                    depth -= 1
                    if depth == 0:
                        pos += 1
                        break
            pos += 1
        blocks.append(text[start:pos])
        idx = pos
    return blocks


def parse_footprint_block(block: str) -> Footprint:
    first_line = block.splitlines()[0]
    footprint_id = re.search(r'\(footprint\s+"([^"]+)"', first_line).group(1)

    layer = LAYER_RE.search(block).group(1)
    at_match = AT_RE.search(block)
    x_mm = Decimal(at_match.group(1))
    y_mm = Decimal(at_match.group(2))
    rotation_deg = Decimal(at_match.group(3) or "0")

    properties = {unescape_quoted(name): unescape_quoted(value) for name, value in PROPERTY_RE.findall(block)}
    attrs_match = ATTR_RE.search(block)
    attrs = set(attrs_match.group(1).split()) if attrs_match else set()

    ref = properties.get("Reference", "")
    value = properties.get("Value", "")

    return Footprint(
        footprint_id=footprint_id,
        layer=layer,
        ref=ref,
        value=value,
        x_mm=x_mm,
        y_mm=y_mm,
        rotation_deg=rotation_deg,
        attrs=attrs,
        properties=properties,
    )


def load_footprints(board_path: Path) -> list[Footprint]:
    text = board_path.read_text(encoding="utf-8")
    return [parse_footprint_block(block) for block in extract_blocks(text, "footprint")]


def apply_midpoint_correction(footprint: Footprint) -> tuple[Decimal, Decimal]:
    x_mm = footprint.x_mm
    y_mm = footprint.y_mm

    for pattern, (px, py) in MIDPOINT_CORRECTIONS.items():
        if re.match(pattern, footprint.local_name):
            rad = footprint.rotation_deg * pi() / Decimal("180")
            qx = cos(rad) * px - sin(rad) * py
            qy = sin(rad) * px + cos(rad) * py
            return (
                x_mm + qx.quantize(Decimal("0.001")),
                y_mm + qy.quantize(Decimal("0.001")),
            )

    return x_mm, y_mm


def corrected_rotation(footprint: Footprint) -> Decimal:
    rotation = footprint.rotation_deg
    for pattern, offset in ROTATION_OFFSETS.items():
        if re.match(pattern, footprint.local_name):
            rotation += offset
            break
    return normalize_rotation(rotation)


def write_cpl(output_path: Path, footprints: list[Footprint]) -> None:
    rows = []
    for footprint in footprints:
        x_mm, y_mm = apply_midpoint_correction(footprint)
        rows.append(
            {
                "Designator": footprint.ref,
                "Mid X": f"{format_decimal(x_mm)}mm",
                "Mid Y": f"{format_decimal(y_mm)}mm",
                "Layer": "top" if footprint.layer == "F.Cu" else "bottom",
                "Rotation": format_decimal(corrected_rotation(footprint)),
            }
        )

    rows.sort(key=lambda row: natural_sort_key(row["Designator"]))

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["Designator", "Mid X", "Mid Y", "Layer", "Rotation"],
        )
        writer.writeheader()
        writer.writerows(rows)


def write_bom(output_path: Path, footprints: list[Footprint]) -> int:
    grouped: dict[tuple[str, ...], list[Footprint]] = {}

    for footprint in footprints:
        manufacturer = sanitize_csv_text(footprint.properties.get("Manufacturer_Name", ""))
        mpn = sanitize_csv_text(footprint.properties.get("Manufacturer_Part_Number", ""))
        lcsc = sanitize_csv_text(footprint.properties.get("LCSC", ""))
        jlc_fp = sanitize_csv_text(footprint.properties.get("JLC", ""))
        mouser = sanitize_csv_text(footprint.properties.get("Mouser Part Number", ""))

        key = (
            sanitize_csv_text(footprint.value),
            sanitize_csv_text(footprint.footprint_id),
            manufacturer,
            mpn,
            lcsc,
            jlc_fp,
            mouser,
        )
        grouped.setdefault(key, []).append(footprint)

    rows = []
    for key, refs in grouped.items():
        value, footprint_id, manufacturer, mpn, lcsc, jlc_fp, mouser = key
        refs_sorted = sorted((item.ref for item in refs), key=natural_sort_key)
        rows.append(
            {
                "Comment": value,
                "Designator": ",".join(refs_sorted),
                "Footprint": footprint_id,
                "LCSC Part #": lcsc,
                "Quantity": str(len(refs_sorted)),
                "Manufacturer": manufacturer,
                "Manufacturer Part Number": mpn,
                "JLC Footprint": jlc_fp,
                "Mouser Part Number": mouser,
            }
        )

    rows.sort(key=lambda row: natural_sort_key(row["Designator"].split(",")[0]))

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "Comment",
                "Designator",
                "Footprint",
                "LCSC Part #",
                "Quantity",
                "Manufacturer",
                "Manufacturer Part Number",
                "JLC Footprint",
                "Mouser Part Number",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    return len(rows)


def main() -> None:
    board_path = Path("piezoTester v2.2.kicad_pcb")
    if not board_path.exists():
        raise SystemExit(f"Missing board file: {board_path}")

    footprints = load_footprints(board_path)
    bom_footprints = [fp for fp in footprints if fp.include_for_bom()]
    cpl_footprints = [fp for fp in footprints if fp.include_for_cpl()]

    bom_rows = write_bom(Path("piezoTester_v2.2_JLCPCB_BOM.csv"), bom_footprints)
    write_cpl(Path("piezoTester_v2.2_JLCPCB_CPL.csv"), cpl_footprints)

    top_count = sum(1 for fp in cpl_footprints if fp.layer == "F.Cu")
    bottom_count = sum(1 for fp in cpl_footprints if fp.layer == "B.Cu")
    missing_lcsc = sum(1 for fp in bom_footprints if not fp.properties.get("LCSC", "").strip())

    print(f"BOM rows: {bom_rows}")
    print(f"Placed components: {len(cpl_footprints)} (top: {top_count}, bottom: {bottom_count})")
    print(f"Components without LCSC code: {missing_lcsc}")
    print("Generated piezoTester_v2.2_JLCPCB_BOM.csv")
    print("Generated piezoTester_v2.2_JLCPCB_CPL.csv")


if __name__ == "__main__":
    main()
