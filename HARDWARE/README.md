# Hardware

This directory contains the physical implementation of PiezoTester V2.0: electronics, PCB revisions, and mechanical CAD.

## Scope

- Measurement electronics
- Actuator-drive electronics
- PCB revision history
- Local KiCad symbol, footprint, and 3D assets
- Mechanical fixture and assembly parts

## Subdirectories

- [PCB/README.md](PCB/README.md)
  KiCad projects, fabrication outputs, local libraries, and revision-specific notes.
- [CAD/README.md](CAD/README.md)
  Mechanical parts and assembly files used to build the test fixture.

## Design Philosophy

The hardware side of this repository is treated as a first-class engineering asset rather than a binary dump. Board revisions, local libraries, and CAD files are intentionally kept close to the firmware and desktop software so the full system stays reproducible.
