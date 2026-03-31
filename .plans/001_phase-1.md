# Plan: retina-spectrum — Phase 1 Scaffold

## Context
Building a wideband SDR spectrum analyser for passive radar site survey and onboarding.
Phase 1 is infrastructure only — get a repo, a container, and remote access working
before touching any SDR or C++ code.

## Phase 1 Scope

### Repo structure
```
retina-spectrum/
├── CLAUDE.md
├── .plans/phase-1.md        ← this file
├── Dockerfile               ← nginx:alpine, serves web/ on port 3020
├── docker-compose.yml
└── web/
    └── index.html           ← static placeholder
```

### Docker container
- Base: nginx:alpine
- Serves web/index.html on port 3020
- Run on Pi: docker compose up -d

### Cloudflare
- cloudflared already installed on owl-os
- Add port 3020 to tunnel config
- Verify access from Mac via Cloudflare URL

## Verification
1. docker compose up -d → owl.local:3020 shows placeholder
2. Cloudflare tunnel → accessible remotely from Mac

## Phase 2 (future)
See CLAUDE.md for full architecture.
