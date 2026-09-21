## What and why

<!-- The diff shows what changed. Say why it needed to change. -->

## Definition of Done

- [ ] `bash scripts/verify.sh` exits 0 (`--e2e` if you touched the UI or gateway)
- [ ] Tests added at the right layer, and the scenario-matrix row flipped
- [ ] `CHANGELOG.md` updated
- [ ] Tunables live in `config/robonode.settings.json`, not as literals
- [ ] No seam bypassed; no new dependency without a seam we own

## Anything a reviewer should push back on

<!-- Trade-offs you made, limits you know about, decisions you are unsure of. -->
