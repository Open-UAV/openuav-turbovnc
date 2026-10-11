"""Isolated TLS bootstrap checks. No desktop, network, or provider credentials."""
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "autonomous_sys_build/prepare_novnc_tls.sh"


class TLSBootstrapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="novnc-tls-test-")
        self.addCleanup(self.temp.cleanup)
        self.environment = {key: value for key, value in os.environ.items()
                            if key not in ("NOVNC_CERT_FILE", "NOVNC_DEV_TLS")}
        self.environment["TMPDIR"] = self.temp.name

    def run_helper(self, **settings):
        return subprocess.run(["sh", str(SCRIPT)],
                              env={**self.environment, **settings},
                              capture_output=True, text=True, timeout=30)

    def generate(self):
        result = self.run_helper(NOVNC_DEV_TLS="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        return Path(result.stdout.strip())

    def test_missing_configuration_fails(self):
        result = self.run_helper()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")

    def test_development_keys_are_unique_and_private(self):
        first, second = self.generate(), self.generate()
        self.assertNotEqual(first.read_bytes(), second.read_bytes())
        self.assertEqual(stat.S_IMODE(first.stat().st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(first.parent.stat().st_mode), 0o700)

    def test_valid_mount_is_used_unchanged(self):
        certificate = self.generate()
        before = certificate.read_bytes()
        result = self.run_helper(NOVNC_CERT_FILE=str(certificate))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), str(certificate))
        self.assertEqual(certificate.read_bytes(), before)

    def test_missing_or_invalid_mount_fails_without_fallback(self):
        invalid = Path(self.temp.name) / "invalid.pem"
        invalid.write_text("not a certificate")
        for candidate in [invalid, invalid.with_name("missing.pem")]:
            result = self.run_helper(NOVNC_CERT_FILE=str(candidate), NOVNC_DEV_TLS="1")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, "")

    def test_expired_certificate_fails(self):
        source = self.generate()
        key = subprocess.check_output(["openssl", "pkey", "-in", str(source)])
        expired_cert = subprocess.check_output([
            "openssl", "x509", "-in", str(source), "-signkey", str(source),
            "-days", "0"], stderr=subprocess.DEVNULL)
        expired = Path(self.temp.name) / "expired.pem"
        expired.write_bytes(key + expired_cert)
        result = self.run_helper(NOVNC_CERT_FILE=str(expired))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertIn("invalid or expired", result.stderr)

    def test_mismatched_key_and_certificate_fail(self):
        first, second = self.generate(), self.generate()
        key = subprocess.check_output(["openssl", "pkey", "-in", str(first)])
        cert = subprocess.check_output(["openssl", "x509", "-in", str(second)])
        mismatch = Path(self.temp.name) / "mismatch.pem"
        mismatch.write_bytes(key + cert)
        result = self.run_helper(NOVNC_CERT_FILE=str(mismatch))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertIn("do not match", result.stderr)


if __name__ == "__main__":
    unittest.main()
