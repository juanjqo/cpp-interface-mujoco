#!/usr/bin/env bash
# Bring debian/changelog in line with the rolling version (YY.MM.NN).
# Idempotent: a no-op when the changelog already carries that version.
# Run this before `dpkg-buildpackage` so the produced .deb is stamped with
# the version computed at build time.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="$(bash "${ROOT}/tools/version.sh")"
CURRENT="$(head -n1 "${ROOT}/debian/changelog" | sed -nE 's/^[A-Za-z0-9._+-]+ \(([^)]*)\).*$/\1/p')"

if [ "${VERSION}" = "${CURRENT}" ]; then
    echo "Changelog already at ${VERSION}."
else
    # dch identity: prefer the package maintainer (single source of truth in
    # debian/control). Setting DEBFULLNAME/DEBEMAIL keeps dch non-interactive
    # (it takes the name from DEBFULLNAME, not DEBNAME) -- without them dch
    # prints warnings and waits for a keypress, which would hang unattended
    # builds.
    MAINTAINER="$(sed -nE 's/^Maintainer:[[:space:]]*(.*)$/\1/p' "${ROOT}/debian/control" | head -n1)"
    EMAIL_RE='<([^>]+)>'
    if [[ "${MAINTAINER}" =~ ${EMAIL_RE} ]]; then
        NAME="${MAINTAINER%%" <"*}"
        EMAIL="${BASH_REMATCH[1]}"
    else
        NAME="$(git -C "${ROOT}" config user.name || echo maintainer)"
        EMAIL="$(git -C "${ROOT}" config user.email || echo maintainer@example.com)"
    fi
    DEBFULLNAME="${NAME}" DEBEMAIL="${EMAIL}" \
        dch --distribution noble -v "${VERSION}" \
        "Bumped to rolling version ${VERSION}."
fi
