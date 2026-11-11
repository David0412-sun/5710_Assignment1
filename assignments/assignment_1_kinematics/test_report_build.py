"""Regression checks for source provenance and Windows TeX selection."""
import hashlib
from pathlib import Path
import unittest
from unittest.mock import patch
from uuid import uuid4

from assignments.assignment_1_kinematics import build_report


class SourceProvenanceTests(unittest.TestCase):
    def setUp(self):
        # Keep temporary sources inside the writable project on restricted Windows.
        test_root = build_report.ROOT / ".tmp" / "report-build-tests"
        test_root.mkdir(parents=True, exist_ok=True)
        # Python 3.13's private tempfile DACL can exclude a restricted process
        # token; a normal project directory inherits its writable permissions.
        self.root = test_root / uuid4().hex
        self.root.mkdir()
        self.addCleanup(self.root.rmdir)
        self.source = self.root / "source.py"
        self.addCleanup(self.source.unlink, missing_ok=True)
        self.lf_text = b"value = 1\nprint(value)\n"
        self.source.write_bytes(self.lf_text.replace(b"\n", b"\r\n"))
        self.environment = {
            "source_sha256": {"source.py": hashlib.sha256(self.lf_text).hexdigest()}
        }
        self.root_patch = patch.object(build_report, "ROOT", self.root)
        self.sources_patch = patch.object(build_report, "REQUIRED_SOURCES", ("source.py",))
        self.root_patch.start()
        self.sources_patch.start()
        self.addCleanup(self.root_patch.stop)
        self.addCleanup(self.sources_patch.stop)

    def test_legacy_lf_measurements_survive_windows_checkout(self):
        verification = build_report.verify_source_hashes(self.environment)
        self.assertEqual(verification["scheme"], "legacy-sha256-raw-or-lf")
        self.assertEqual(verification["crlf_normalized_sources"], ["source.py"])

    def test_declared_lf_scheme_handles_both_newline_styles(self):
        self.environment["source_hash_scheme"] = "sha256-lf-v1"
        build_report.verify_source_hashes(self.environment)
        self.source.write_bytes(self.lf_text)
        verification = build_report.verify_source_hashes(self.environment)
        self.assertEqual(verification["crlf_normalized_sources"], [])

    def test_declared_raw_scheme_requires_exact_bytes(self):
        self.environment["source_hash_scheme"] = "sha256-raw-v1"
        with self.assertRaisesRegex(ValueError, "Source changed"):
            build_report.verify_source_hashes(self.environment)
        self.environment["source_sha256"]["source.py"] = build_report.sha256(self.source)
        build_report.verify_source_hashes(self.environment)

    def test_changed_code_is_rejected_under_every_scheme(self):
        self.source.write_bytes(b"value = 2\r\nprint(value)\r\n")
        for scheme in ("legacy-sha256-raw-or-lf", "sha256-lf-v1", "sha256-raw-v1"):
            with self.subTest(scheme=scheme):
                self.environment["source_hash_scheme"] = scheme
                with self.assertRaisesRegex(ValueError, "Source changed"):
                    build_report.verify_source_hashes(self.environment)

    def test_unknown_hash_scheme_is_rejected(self):
        self.environment["source_hash_scheme"] = "unknown"
        with self.assertRaisesRegex(ValueError, "Unsupported source hash scheme"):
            build_report.verify_source_hashes(self.environment)


class WindowsEngineSelectionTests(unittest.TestCase):
    def test_auto_uses_native_pdflatex_when_latexmk_is_also_installed(self):
        available = {"pdflatex": "C:/MiKTeX/pdflatex.exe",
                     "latexmk": "C:/MiKTeX/latexmk.exe"}
        with patch.object(build_report.sys, "platform", "win32"), patch.object(
                build_report, "executable_path", side_effect=available.get):
            self.assertEqual(build_report.find_engine("auto"),
                             ("pdflatex", available["pdflatex"]))
            self.assertEqual(build_report.find_engine("latexmk"),
                             ("latexmk", available["latexmk"]))


if __name__ == "__main__":
    unittest.main()
