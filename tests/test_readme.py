import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import update_readme as u  # noqa: E402


def test_readme_has_markers_diagrams_and_balanced_fences():
    doc = (ROOT / "README.md").read_text()
    for name in ("experiments", "verification"):
        assert f"<!-- BEGIN:{name} -->" in doc and f"<!-- END:{name} -->" in doc
    assert doc.count("```") % 2 == 0
    assert doc.count("```mermaid") == 2


def test_render_injects_results_and_is_idempotent():
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "results").mkdir()
        (root / "results" / "metrics.md").write_text("| a | b |\n|---|---|\n| 1 | 2 |\n")
        (root / "results" / "metrics.json").write_text(json.dumps(
            {"meta": {"git_commit": "abc1234", "python": "3.11.18", "machine": "arm64", "wall_time_s": 3.5}}))
        (root / "results" / "verification.md").write_text("| v |\n|---|\n| 9 |\n")
        (root / "results" / "verification.json").write_text(json.dumps(
            {"meta": {"git_commit": "abc1234", "cpu": "Apple M4", "compiler": "clang",
                      "compile_flags": "-O2", "python": "3.11.18"}}))
        doc = (ROOT / "README.md").read_text()
        out = u.render(doc, root)
        assert "| 1 | 2 |" in out and "| 9 |" in out
        assert "abc1234" in out and "Apple M4" in out
        assert u.render(out, root) == out           # running it twice changes nothing


def test_render_reports_missing_results_instead_of_inventing_numbers():
    with tempfile.TemporaryDirectory() as d:
        out = u.render((ROOT / "README.md").read_text(), Path(d))
        assert out.count("Not generated yet") == 2
