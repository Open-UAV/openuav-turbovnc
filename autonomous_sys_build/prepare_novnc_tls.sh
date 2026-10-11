#!/bin/sh
# Print only the selected combined PEM path. Key material never goes to stdout.
set -eu
umask 077

if [ -n "${NOVNC_CERT_FILE:-}" ]; then
    cert_file=$NOVNC_CERT_FILE
    if [ ! -f "$cert_file" ] || [ ! -r "$cert_file" ]; then
        echo "NOVNC_CERT_FILE must point to a readable combined certificate and private-key PEM." >&2
        exit 1
    fi
elif [ "${NOVNC_DEV_TLS:-0}" = "1" ]; then
    cert_dir=$(mktemp -d "${TMPDIR:-/tmp}/novnc-tls.XXXXXXXX")
    trap 'rm -rf "$cert_dir"' EXIT HUP INT TERM
    cert_file=$cert_dir/self.pem
    openssl req -newkey rsa:2048 -x509 -sha256 -nodes -days 30 \
        -subj '/CN=localhost' -addext 'subjectAltName=DNS:localhost,IP:127.0.0.1' \
        -keyout "$cert_dir/key.pem" -out "$cert_dir/cert.pem" 2>/dev/null
    cat "$cert_dir/key.pem" "$cert_dir/cert.pem" > "$cert_file"
    rm "$cert_dir/key.pem" "$cert_dir/cert.pem"
else
    echo "Mount a combined PEM and set NOVNC_CERT_FILE, or set NOVNC_DEV_TLS=1 for local development." >&2
    exit 1
fi

# Reject expired certificates, encrypted/unreadable keys, and mismatched pairs.
openssl x509 -in "$cert_file" -noout -checkend 0 >/dev/null 2>&1 || {
    echo "The noVNC certificate is invalid or expired." >&2; exit 1;
}
openssl pkey -in "$cert_file" -passin pass: -noout >/dev/null 2>&1 || {
    echo "The noVNC private key is missing, invalid, or encrypted." >&2; exit 1;
}
cert_public=$(openssl x509 -in "$cert_file" -pubkey -noout)
key_public=$(openssl pkey -in "$cert_file" -passin pass: -pubout 2>/dev/null)
[ "$cert_public" = "$key_public" ] || {
    echo "The noVNC certificate and private key do not match." >&2; exit 1;
}
# The generated identity is needed for the lifetime of this container.
trap - EXIT HUP INT TERM
printf '%s\n' "$cert_file"
