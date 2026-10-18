#!/usr/bin/env bash
# Compute the rolling project version: YY.MM.NN
#   YY.MM : year.month of the current date (e.g. 26.09)
#   NN    : number of commits since the monthly version tag (e.g. 26.09).
#           00 when no such tag exists yet.
# The monthly tag is looked up without a "v" prefix (the project
# convention -- the raw tag name is what sas_py's
# setuptools-git-versioning turns into the PyPI version); a "v"-prefixed
# tag is accepted as a fallback.
# Output: 26.09.05
set -euo pipefail

YYMM="$(date +%y.%m)"

# Echo the first existing monthly tag (bare YY.MM, then v-prefixed).
find_version_tag() {
    local t
    for t in "${YYMM}" "v${YYMM}"; do
        if git rev-parse --verify "refs/tags/${t}" >/dev/null 2>&1; then
            printf '%s\n' "${t}"
            return 0
        fi
    done
    return 1
}

NN=0
if command -v git >/dev/null 2>&1 && git rev-parse --git-dir >/dev/null 2>&1; then
    if TAG="$(find_version_tag)"; then
        NN="$(git rev-list "${TAG}..HEAD" --count 2>/dev/null || echo 0)"
    fi
fi

printf '%s.%02d\n' "${YYMM}" "${NN:-0}"
