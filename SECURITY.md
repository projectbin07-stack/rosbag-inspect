# Security policy

## Supported versions

Only the latest release receives fixes.

## Reporting a vulnerability

Please do not open a public issue for security problems. Use GitHub's private reporting instead:
<https://github.com/projectbin07-stack/rosbag-inspect/security/advisories/new>.

Include a description, a minimal input that triggers the problem and the version you used. Reports
are handled on a best-effort basis by a single maintainer.

## Scope

`rosbag-inspect` reads untrusted bag files: SQLite databases (opened read-only), MCAP files, YAML
metadata and CDR payloads. It never executes file content, makes no network requests and writes
only to the paths given on the command line (`--report`, the `export` output). The YAML parser
builds only plain data types. Reports of interest include crashes or excessive memory/CPU use on
crafted input (for example huge length fields in MCAP records, CDR sequences or LZ4 blocks), and
HTML injection through topic names, frame ids or other strings into the generated report.
