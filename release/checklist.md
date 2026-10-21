# Release checklist (AeroCore 0.41.0)

Accounts, uploads and pushes are done by the user; each public step is agreed first. AeroCore goes up before hdfv.

## Before

- [ ] `.\dev.ps1 clean build publish` passes (`verifyReleaseJar` included); `.\dev.ps1 :neoforge:runGameTestServer` passes.
- [ ] The release test (`scripts/release-test/release_test.py`) with hdfv: no suspicious lines.
- [ ] The JAR: `aerocore-neoforge-1.21.1-0.41.0.jar` (not `-sources`, never `aerocore-devtools`), from the local build
      folder (`.\dev.ps1 showBuildDirectories -q`, `neoforge\libs`).

## Modrinth

| Field | Value |
| --- | --- |
| Name | AeroCore |
| Slug | aerocore |
| Summary | release/summary.txt |
| Description | release/page.md |
| Categories | Library, Technology (and Transportation) |
| Environment | client and server: required on both |
| License | LGPL-3.0-only |
| Source / issues | https://github.com/fiseleo/aerocore, /issues |
| Icon | `%USERPROFILE%\.mcmod\release\aerocore\icon.png` (512 x 512) |
| Gallery | `%USERPROFILE%\.mcmod\release\aerocore\gallery\*.png` |
| Version | 0.41.0, release type **Beta**, loader NeoForge, game version 1.21.1 |
| Changelog | release/notes-0.41.0.md |

## CurseForge

Same texts and pictures; project type Mod, main category API and Library, extra Technology; release type Beta;
"NeoForge" and "1.21.1".

## GitHub

- [ ] Push `main` and the tag `v0.41.0` to `fiseleo/aerocore` (agreed).
- [ ] A release `v0.41.0` from the tag, the notes, the JAR attached; marked pre-release (beta).
