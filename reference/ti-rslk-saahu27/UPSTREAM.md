# TI-RSLK learning projects from saahu27

This directory contains a reference snapshot of [saahu27/TI-RSLK](https://github.com/saahu27/TI-RSLK), pinned to commit [`aba15495d8057eb3378c277bf707a7cabdfb3c03`](https://github.com/saahu27/TI-RSLK/commit/aba15495d8057eb3378c277bf707a7cabdfb3c03). The original project root is nested at [`tirslk_max_1_00_00/tirslk_max_1_00_00`](https://github.com/saahu27/TI-RSLK/tree/aba15495d8057eb3378c277bf707a7cabdfb3c03/tirslk_max_1_00_00/tirslk_max_1_00_00).

The snapshot brings 52 lab/example projects and the shared `inc` project into this repository, together with supporting source needed by those projects. It sits alongside the active [MSP432 firmware](../../firmware/msp432/) and the preserved [original TI-RSLK reference selection](../tirslk/). Those existing sources are not replaced. The normal firmware build does not compile this snapshot, and its examples are not connected to the Python motion planner or host control commands.

The exact retained file inventory and hashes are recorded in [MANIFEST.json](MANIFEST.json). Use that manifest and the pinned commit to identify the imported material; this is a filtered snapshot of source and supporting assets, not a complete mirror of the upstream checkout or its Git history.

## What is retained and omitted

Project source, CCS project metadata, linker/target configuration files, shared support source, and original source/license notices are retained. Relative project layouts remain intact so references to sibling `inc`, `driverlib`, and required CC3100 SDK source can resolve. The upstream `documentation/` directory and its root HTML redirect are also retained.

The 94 project-level `.obj` files shipped outside upstream `Debug`/`Release` directories are retained because CCS educational projects can use them as link dependencies. They are upstream binary reference assets; they were not rebuilt or validated as firmware by this integration.

The snapshot excludes browser profiles and caches, generated `Debug`/`Release` build outputs, launch presets, datasheets, Windows executables, SDK tools/documentation, and SDK examples other than the shared `examples/common` support. These omissions are intentional; the manifest is the authoritative file list. The snapshot does not supply an installed CCS toolchain or a complete TI SDK installation.

## Relationship to the existing references

The existing `reference/tirslk/` selection came from the original UT-hosted TI-RSLK archive, as recorded in [its provenance](../PROVENANCE.md). Comparing its 16 source/header files with this GitHub revision gives:

| Comparison | Files |
|---|---|
| Byte-identical | `RobotArm/startup_msp432p401r_ccs.c`, `RobotArm/system_msp432p401r.c` |
| Line-ending differences only | The other 10 files not listed as content changes below |
| Content changes | `Lab12_Motors/Lab12_Motorsmain.c`, `inc/Motor.c`, `inc/MotorSimple.c`, `inc/PWM.c` |

The three changed drivers fill motor initialization, direction, software PWM, and timer PWM exercise sections. The changed Lab12 entry point selects a repeated right-motor test with its pause calls disabled. These differences explain why the original archive references and this course-solutions snapshot are preserved separately. RobotArm, ADC14, and Tachometer contents match the original selection after line-ending normalization.

## Open a project in Code Composer Studio

1. Install a suitable TI Code Composer Studio environment with the MSP432 device support, headers, and compiler required by the chosen project's metadata. The archived projects may refer to older TI tool versions or SDK path variables; resolve those dependencies in the installed environment.
2. Keep this directory's sibling project/support folders together. Use CCS project import with this snapshot as the search root, then select the desired lab/example and any required shared project. Import projects at their existing locations; copying a single lab into an unrelated workspace directory can break its relative shared-source paths.
3. `CC3100_GetWeather` also uses `WORKSPACE_LOC` in its linker search paths. Point those search paths at this snapshot's shared `driverlib`, `inc`, and `cc3100-sdk` folders if your CCS workspace is elsewhere.
4. Check the imported project's processor, include paths, linked resources, compiler, and linker configuration before building. Most projects concern MSP432P401R, but project assumptions are not uniform; for example, `Challenge1` includes MSP432E401Y startup/system material.
5. Build only the selected example after resolving its dependencies. The repository's `tools/build_msp432.sh` continues to build the existing application in `firmware/msp432/`; it is not a build command for these CCS examples.

Importing or building source does not program a board. No upstream lab has been compiled or flashed as part of this snapshot integration, and no physical behavior is claimed as verified. Some projects still contain exercise blanks or incomplete code. Example duty cycles, servo positions, board revisions, peripherals, and wiring assumptions require review for the actual hardware. The active application's calibration, arming, timeout, and stop behavior does not automatically apply to these independent example programs.

## Licenses and attribution

Keep [license.txt](license.txt) and every retained per-file notice with the corresponding source. Much of the learning material carries Jonathan Valvano's Simplified BSD/FreeBSD-style notice, while TI startup, driver, SDK, and utility files carry their own notices. Some TI utility material has TI-device-specific terms. The root notice does not replace those per-file terms, and the snapshot should not be described as uniformly licensed under one BSD license.

The upstream repository describes its contents as TI-RSLK MAX learning-module solutions and additional ENEE467 robotics-laboratory modules. Inclusion here records their source and provenance; it does not imply endorsement by TI, the original authors, or the upstream repository maintainer.
