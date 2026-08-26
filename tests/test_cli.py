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


def test_solve_with_eecbs(capsys):
    args = ["--map", str(MAPS / "corridor-swap.map"), "--scen", str(MAPS / "corridor-swap.scen"), "-k", "2"]
    code = main(["solve", *args, "--algo", "eecbs", "--w", "1.2", "--no-bypass"])
    out = capsys.readouterr().out
    assert code == 0
    assert "solved    True" in out and "conflicts 0" in out and "valid     True" in out


def test_solve_with_lns2(capsys):
    args = ["--map", str(MAPS / "bottleneck.map"), "--scen", str(MAPS / "bottleneck.scen"), "-k", "4"]
    code = main(["solve", *args, "--algo", "lns2", "--neighborhood", "2"])
    out = capsys.readouterr().out
    assert code == 0
    assert "solved    True" in out and "valid     True" in out


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
    bench.run(str(MAPS / "bottleneck.map"), [str(MAPS / "bottleneck.scen")], ["independent", "eecbs:w=1.5"], [1, 4], 5.0, str(out))
    rows = list(csv.DictReader(out.open()))
    assert [(r["algo"], r["agents"]) for r in rows] == [("independent", "1"), ("independent", "4"), ("eecbs:w=1.5", "1"), ("eecbs:w=1.5", "4")]
    assert rows[0]["valid"] == "True" and rows[0]["soc"] == "6"
    assert rows[3]["solved"] == rows[3]["valid"] == "True"


def test_bench_parse_algo():
    spec = importlib.util.spec_from_file_location("bench_run", Path(__file__).parents[1] / "bench" / "run.py")
    bench = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bench)
    assert bench.parse_algo("independent") == ("independent", {})
    assert bench.parse_algo("eecbs:w=1.2:bypass=0") == ("eecbs", {"w": 1.2, "bypass": 0.0})


def test_plot_writes_pngs_and_tables(tmp_path):
    modules = {}
    for name in ["run", "plot"]:
        spec = importlib.util.spec_from_file_location(f"bench_{name}", Path(__file__).parents[1] / "bench" / f"{name}.py")
        modules[name] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(modules[name])

    out = tmp_path / "results.csv"
    modules["run"].run(str(MAPS / "bottleneck.map"), [str(MAPS / "bottleneck.scen")], ["independent", "lns2"], [2, 4], 5.0, str(out))
    text = modules["plot"].summarize(modules["plot"].load([str(out)]), tmp_path)
    assert (tmp_path / "success-bottleneck.png").stat().st_size > 0
    assert (tmp_path / "runtime-bottleneck.png").stat().st_size > 0
    assert "| lns2 | 1/1 | 1/1 |" in text and "SOC / independent SOC" in text
