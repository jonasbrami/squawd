"""Turn per-run knee results into the capacity-frontier table (Markdown) and,
when matplotlib is available, a heatmap PNG.
"""


def build_frontier_table(rows: list[dict]) -> dict:
    table: dict = {}
    for r in rows:
        table.setdefault(r["backend"], {})[r["resolution"]] = {
            "n": r["knee_n"], "limit": r["limiting"]}
    return table


def render_markdown(table: dict, backends: list[str], resolutions: list[str]) -> str:
    header = "| resolution | " + " | ".join(backends) + " |"
    sep = "|" + "---|" * (len(backends) + 1)
    lines = [header, sep]
    for res in resolutions:
        cells = []
        for b in backends:
            cell = table.get(b, {}).get(res)
            cells.append(f"{cell['n']} ({cell['limit']})" if cell else "—")
        lines.append(f"| {res} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def render_heatmap(table: dict, backends: list[str], resolutions: list[str], path: str) -> bool:
    """Write a max-N heatmap PNG. Returns False if matplotlib is unavailable."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    grid = [[(table.get(b, {}).get(res) or {}).get("n", 0) for b in backends]
            for res in resolutions]
    fig, ax = plt.subplots(figsize=(1.6 * len(backends) + 2, 0.7 * len(resolutions) + 2))
    im = ax.imshow(grid, cmap="viridis", aspect="auto")
    ax.set_xticks(range(len(backends)), backends)
    ax.set_yticks(range(len(resolutions)), resolutions)
    for y, res in enumerate(resolutions):
        for x, b in enumerate(backends):
            cell = table.get(b, {}).get(res)
            ax.text(x, y, "—" if not cell else f"{cell['n']}\n{cell['limit']}",
                    ha="center", va="center", color="w", fontsize=8)
    ax.set_title("Max sustainable drones")
    fig.colorbar(im, ax=ax, label="drones")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return True
