"""Plot the CSVs written by bench/run.py and print summary tables in markdown.

    python bench/plot.py docs/results/*.csv --out docs/

For every map, it saves ``success-<map>.png`` (solved instances against agents) and
``runtime-<map>.png`` (median runtime of the solved ones), and prints four tables: success,
median runtime, SOC over the ``independent`` SOC of the same instance (a lower bound on the
optimal SOC, so this is an upper bound on the real suboptimality), and the collisions of the
``independent`` plan (what the solvers have to remove).
"""

import argparse
import csv
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # files only, no windows
import matplotlib.pyplot as plt  # noqa: E402


def load(paths: list[str]) -> list[dict]:
    rows = []
    for path in paths:
        with open(path, newline="") as f:
            rows.extend(csv.DictReader(f))
    return rows


def table(title: str, counts: list[int], lines: dict[str, list[str]]) -> str:
    """A markdown table: one row per algorithm, one column per agent count."""
    out = [f"**{title}**", "", "| agents | " + " | ".join(str(k) for k in counts) + " |"]
    out.append("|---" * (len(counts) + 1) + "|")
    for algo, cells in lines.items():
        out.append(f"| {algo} | " + " | ".join(cells) + " |")
    return "\n".join(out) + "\n"


def summarize(rows: list[dict], out: Path) -> str:
    """Plots for the rows of one map; returns its tables."""
    name = rows[0]["map"]
    counts = sorted({int(r["agents"]) for r in rows})
    algos = list(dict.fromkeys(r["algo"] for r in rows))  # in order of first appearance
    lower_bound = {(r["scen"], r["agents"]): int(r["soc"]) for r in rows if r["algo"] == "independent"}

    success, runtime, quality, collisions = {}, {}, {}, {}
    fig_s, ax_s = plt.subplots(figsize=(6, 4))
    fig_r, ax_r = plt.subplots(figsize=(6, 4))
    for algo in algos:
        if algo == "independent":
            cells = []
            for k in counts:
                runs = [r for r in rows if r["algo"] == algo and int(r["agents"]) == k]
                cells.append(f"{statistics.mean(int(r['conflicts']) for r in runs):.0f}" if runs else "")
            collisions[algo] = cells
            continue
        solved_counts, medians = [], []
        success[algo], runtime[algo], quality[algo] = [], [], []
        for k in counts:
            runs = [r for r in rows if r["algo"] == algo and int(r["agents"]) == k]
            solved = [r for r in runs if r["solved"] == "True"]
            ratios = [int(r["soc"]) / lower_bound[(r["scen"], r["agents"])] for r in solved if (r["scen"], r["agents"]) in lower_bound]
            median = statistics.median(float(r["runtime"]) for r in solved) if solved else None
            success[algo].append(f"{len(solved)}/{len(runs)}" if runs else "")
            runtime[algo].append(f"{median:.1f}" if median is not None else "–")
            quality[algo].append(f"{statistics.mean(ratios):.3f}" if ratios else "–")
            solved_counts.append(len(solved) / len(runs) if runs else None)
            medians.append(median)
        ax_s.plot(counts, solved_counts, marker="o", label=algo)
        ax_r.plot(counts, medians, marker="o", label=algo)

    ax_s.set(title=f"{name}: success rate", xlabel="agents", ylabel="solved", ylim=(-0.05, 1.05))
    ax_r.set(title=f"{name}: median runtime of solved runs", xlabel="agents", ylabel="seconds", yscale="log")
    for fig, ax, kind in [(fig_s, ax_s, "success"), (fig_r, ax_r, "runtime")]:
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out / f"{kind}-{name}.png", dpi=120)
        plt.close(fig)

    text = f"### {name}\n\n" + table("Solved instances", counts, success)
    text += "\n" + table("Median runtime of solved runs (s)", counts, runtime)
    if lower_bound:
        text += "\n" + table("SOC / independent SOC (mean over solved runs)", counts, quality)
        text += "\n" + table("Collisions in the independent plan (mean)", counts, collisions)
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="CSV files from bench/run.py")
    parser.add_argument("--out", default=".", help="directory for the PNG files")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = load(args.csv)
    for name in dict.fromkeys(r["map"] for r in rows):
        print(summarize([r for r in rows if r["map"] == name], out))


if __name__ == "__main__":
    main()
