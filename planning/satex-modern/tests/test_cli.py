"""Exercise the user-facing commands and saved-artifact lifecycle."""

import json

from satex.cli import main


def test_demo_solve_verify_roundtrip(tmp_path):
    demo = tmp_path / "demo"
    assert main(["demo", "--scenario", "reach-avoid", "--output", str(demo)]) == 0
    result = json.loads((demo / "result.json").read_text())
    assert result["status"] == "sat"
    assert result["validation"]["valid"]
    assert (demo / "trajectory.png").stat().st_size > 1000
    html = (demo / "trajectory.html").read_text()
    assert "<canvas" in html
    assert "__DATA__" not in html
    assert "__TITLE__" not in html
    assert main(["verify", str(demo)]) == 0
    assert json.loads((demo / "verification.json").read_text())["valid"]

    solved = tmp_path / "solved"
    assert (
        main(["solve", str(demo / "problem.json"), "--horizon", "8", "--no-visuals", "--output", str(solved)])
        == 0
    )
    assert main(["verify", str(solved)]) == 0
    assert not (solved / "trajectory.html").exists()


def test_unsat_rerun_invalidates_old_views_and_verification(tmp_path):
    output = tmp_path / "attempt"
    assert main(["demo", "--no-visuals", "--output", str(output)]) == 0
    assert main(["verify", str(output)]) == 0
    for name in ("trajectory.png", "trajectory.html"):
        (output / name).write_text("old successful trajectory")
    (output / "notes.txt").write_text("user notes")
    assert main(["demo", "--horizon", "1", "--output", str(output)]) == 2
    assert json.loads((output / "result.json").read_text())["status"] == "unsat"
    for name in ("trajectory.png", "trajectory.html", "verification.json"):
        assert not (output / name).exists()
    assert (output / "notes.txt").read_text() == "user notes"


def test_no_visuals_rerun_removes_old_generated_views(tmp_path):
    output = tmp_path / "attempt"
    output.mkdir()
    for name in ("trajectory.png", "trajectory.html", "verification.json"):
        (output / name).write_text("outdated")
    assert main(["demo", "--no-visuals", "--output", str(output)]) == 0
    for name in ("trajectory.png", "trajectory.html", "verification.json"):
        assert not (output / name).exists()


def test_three_dimensional_visualization_serializes_geometry(tmp_path):
    output = tmp_path / "3d"
    assert main(["demo", "--scenario", "3d", "--output", str(output)]) == 0
    html = (output / "trajectory.html").read_text()
    payload = html.split('<script type="application/json" id="data">', 1)[1].split("</script>", 1)[0]
    data = json.loads(payload)
    assert data["dimension"] == 3
    assert data["geometry"][0]["edges"]
    assert (output / "trajectory.png").stat().st_size > 1000
    assert main(["verify", str(output)]) == 0


def test_verify_rejects_modified_saved_trajectory(tmp_path):
    output = tmp_path / "attempt"
    assert main(["demo", "--no-visuals", "--output", str(output)]) == 0
    path = output / "result.json"
    result = json.loads(path.read_text())
    result["states"]["r1"][1][0] += 10
    path.write_text(json.dumps(result))
    assert main(["verify", str(output)]) == 1
    report = json.loads((output / "verification.json").read_text())
    assert not report["valid"]
    assert any("dynamics" in error for error in report["errors"])


def test_invalid_input_returns_error_without_output(tmp_path, capsys):
    source = tmp_path / "bad.json"
    source.write_text("{}")
    output = tmp_path / "output"
    assert main(["solve", str(source), "--output", str(output)]) == 1
    assert "satex:" in capsys.readouterr().err
    assert not output.exists()
