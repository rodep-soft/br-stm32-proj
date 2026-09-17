#!/usr/bin/env bash
set -e

# Re-run with sudo if not root
if [ "$(id -u)" -ne 0 ]; then
    echo "==> Requesting sudo privileges to disable firewall..."
    exec sudo bash "$0" "$@"
fi

echo "=========================================================="
echo "          Disabling Host Firewall & Packet Filters        "
echo "=========================================================="

# 1. Stop and disable systemd firewall services
for svc in ufw firewalld nftables iptables ip6tables; do
    if systemctl is-active --quiet "$svc" 2>/dev/null; then
        echo "==> Stopping $svc..."
        systemctl stop "$svc" 2>/dev/null || true
    fi
    if systemctl is-enabled --quiet "$svc" 2>/dev/null; then
        echo "==> Disabling $svc..."
        systemctl disable "$svc" 2>/dev/null || true
    fi
done

# 2. Disable UFW if present
if command -v ufw >/dev/null 2>&1; then
    echo "==> Disabling UFW..."
    ufw disable 2>/dev/null || true
fi

# 3. Flush nftables ruleset if present
if command -v nft >/dev/null 2>&1; then
    echo "==> Flushing nftables ruleset..."
    nft flush ruleset 2>/dev/null || true
fi

# 4. Reset iptables policies to ACCEPT and flush all rules
if command -v iptables >/dev/null 2>&1; then
    echo "==> Resetting iptables (IPv4) to ACCEPT and flushing..."
    iptables -P INPUT ACCEPT
    iptables -P FORWARD ACCEPT
    iptables -P OUTPUT ACCEPT
    iptables -t nat -F 2>/dev/null || true
    iptables -t mangle -F 2>/dev/null || true
    iptables -t raw -F 2>/dev/null || true
    iptables -F
    iptables -X
fi

# 5. Reset ip6tables policies to ACCEPT and flush all rules
if command -v ip6tables >/dev/null 2>&1; then
    echo "==> Resetting ip6tables (IPv6) to ACCEPT and flushing..."
    ip6tables -P INPUT ACCEPT
    ip6tables -P FORWARD ACCEPT
    ip6tables -P OUTPUT ACCEPT
    ip6tables -t nat -F 2>/dev/null || true
    ip6tables -t mangle -F 2>/dev/null || true
    ip6tables -t raw -F 2>/dev/null || true
    ip6tables -F
    ip6tables -X
fi

echo "=========================================================="
echo "All firewalls, nftables, and iptables rules disabled/cleared!"
echo "All incoming/outgoing packets are now set to ACCEPT."
echo "=========================================================="
