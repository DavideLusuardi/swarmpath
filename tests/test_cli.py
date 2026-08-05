import csv
import importlib.util
from pathlib import Path

from conftest import MAPS
from swarmpath.cli import main


def test_solve_from_scen(capsys):
    code = main(["solve", "--map", str(MAPS / "corridor-swap.map"), "--scen", str(MAPS / "corridor-swap.scen"), "-k", "2"])
    out = capsys.readouterr().out
    assert code == 0
    assert "SOC       12" in out and "valid     False" in out
    assert "vertex agents 0,1 at (3, 1)" in out


def test_solve_random_and_save_gif(tmp_path):
    gif = tmp_path / "out.gif"
    code = main(["solve", "--map", str(MAPS / "bottleneck.map"), "--random", "-k", "3", "--save", str(gif)])
    assert code == 0 and gif.stat().st_size > 0


def test_bench_writes_csv(tmp_path):
    spec = importlib.util.spec_from_file_location("bench_run", Path(__file__).parents[1] / "bench" / "run.py")
    bench = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bench)

    assert bench.agent_counts("1:4:2") == [1, 3]
    out = tmp_path / "results.csv"
    bench.run(str(MAPS / "bottleneck.map"), str(MAPS / "bottleneck.scen"), ["independent"], [1, 4], 5.0, str(out))
    rows = list(csv.DictReader(out.open()))
    assert [r["agents"] for r in rows] == ["1", "4"]
    assert rows[0]["valid"] == "True" and rows[0]["soc"] == "6"
