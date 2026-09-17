#!/usr/bin/env bash
# ==============================================================================
# Setup Nix Configuration (Flakes & Binary Caches)
# ==============================================================================
set -euo pipefail

NIX_CONF_DIR="${HOME}/.config/nix"
NIX_CONF_FILE="${NIX_CONF_DIR}/nix.conf"

echo "==> [NixConf] Setting up user Nix configuration at ${NIX_CONF_FILE}..."

mkdir -p "${NIX_CONF_DIR}"
touch "${NIX_CONF_FILE}"

# 1. Enable Flakes and nix-command
if grep -q "experimental-features" "${NIX_CONF_FILE}"; then
    if ! grep -q "flakes" "${NIX_CONF_FILE}"; then
        echo "Updating experimental-features in ${NIX_CONF_FILE}..."
        sed -i '/experimental-features/s/$/ flakes nix-command/' "${NIX_CONF_FILE}"
    fi
else
    echo "Adding experimental-features = nix-command flakes..."
    echo "experimental-features = nix-command flakes" >> "${NIX_CONF_FILE}"
fi

# 2. Add extra-substituters if not already configured
SUBSTITUTERS="https://roborescue-nix.cachix.org https://nix-community.cachix.org https://ros.cachix.org"
if ! grep -q "roborescue-nix.cachix.org" "${NIX_CONF_FILE}"; then
    echo "Adding extra-substituters to ${NIX_CONF_FILE}..."
    if grep -q "extra-substituters" "${NIX_CONF_FILE}"; then
        sed -i "/extra-substituters/s|$| ${SUBSTITUTERS}|" "${NIX_CONF_FILE}"
    else
        echo "extra-substituters = ${SUBSTITUTERS}" >> "${NIX_CONF_FILE}"
    fi
fi

# 3. Add extra-trusted-public-keys if not already configured
KEYS="roborescue-nix.cachix.org-1:qy3rP4VwHob/xePMW77gUxZVvPMz8izs86rIdruro0U= nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs= ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
if ! grep -q "roborescue-nix.cachix.org-1" "${NIX_CONF_FILE}"; then
    echo "Adding extra-trusted-public-keys to ${NIX_CONF_FILE}..."
    if grep -q "extra-trusted-public-keys" "${NIX_CONF_FILE}"; then
        sed -i "/extra-trusted-public-keys/s|$| ${KEYS}|" "${NIX_CONF_FILE}"
    else
        echo "extra-trusted-public-keys = ${KEYS}" >> "${NIX_CONF_FILE}"
    fi
fi

# 4. Allow direnv if present
if command -v direnv >/dev/null 2>&1; then
    echo "==> [NixConf] Allowing direnv for this workspace..."
    direnv allow || true
fi

echo "==> [NixConf] Configuration successfully deployed!"
echo "Current ~/.config/nix/nix.conf:"
echo "----------------------------------------------------------"
cat "${NIX_CONF_FILE}"
echo "----------------------------------------------------------"
