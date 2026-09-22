#!/usr/bin/env python3
"""Shared figure styling for every rendered artifact.

One visual language across the matplotlib figures and the two HTML pages:
neutral paper ground, hairline rules, monospace tabular numerics, and colour
reserved for agents. The categorical palettes below pass all six checks of the
dataviz palette validator (lightness band, chroma floor, CVD separation,
normal-vision floor, contrast) against their respective surfaces.
"""

# Ink and surface tokens; mirrored by the CSS custom properties in tools/*.html.
BG = "#fffffe"
FIELD = "#fcfcfa"
INK = "#16160f"
MUTED = "#6e6e65"
FAINT = "#8d8d84"
RULE = "#dcdcd5"
RULE_2 = "#a9a9a0"
GRID = "#e2e2db"

# Scene furniture stays greyscale so colour only ever means "agent".
ROAD = "#f2f2ed"
CURB = "#e6e6df"
OBSTACLE = "#e3e3dc"
OBSTACLE_LINE = "#8d8d84"

# Categorical series, assigned in fixed order and never cycled.
SERIES = ["#1268a3", "#c2551a", "#0f9d74", "#a83f86",
          "#8a6a00", "#0088a0", "#5b4bc4", "#a32a35"]
SERIES_DARK = ["#3f8fc4", "#d4703a", "#0aa98d", "#b458ab",
               "#9c7c1a", "#109aae", "#7a68d8", "#b84a54"]

MONO = ["DejaVu Sans Mono", "monospace"]


def fill(color, alpha="26"):
    """Series colour as a light fill; the solid colour stays on the outline."""
    return color + alpha


def use():
    """Apply the shared rcParams. Call before creating any figure."""
    import matplotlib
    matplotlib.rcParams.update({
        "figure.facecolor": BG,
        "figure.edgecolor": BG,
        "savefig.facecolor": BG,
        "axes.facecolor": FIELD,
        "axes.edgecolor": RULE_2,
        "axes.linewidth": 0.8,
        "axes.labelcolor": MUTED,
        "axes.labelsize": 9,
        "axes.titlesize": 11,
        "axes.titlecolor": INK,
        "axes.titlelocation": "left",
        "axes.titleweight": "regular",
        "axes.titlepad": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": FAINT,
        "ytick.color": FAINT,
        "xtick.labelcolor": MUTED,
        "ytick.labelcolor": MUTED,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "xtick.major.width": 0.8,
        "ytick.major.width": 0.8,
        "xtick.major.size": 3.5,
        "ytick.major.size": 3.5,
        "font.size": 9.5,
        "text.color": INK,
        "legend.frameon": False,
        "legend.fontsize": 8.5,
        "legend.labelcolor": MUTED,
        "figure.dpi": 100,
    })


def swatch(fig, x, y, color, size=0.009):
    """A small filled square in figure coordinates, for direct labelling.

    Identity rides on the mark so the adjacent text can stay in ink tokens.
    """
    from matplotlib.patches import Rectangle
    aspect = fig.get_figwidth() / fig.get_figheight()
    fig.patches.append(Rectangle((x, y), size, size * aspect, transform=fig.transFigure,
                                 facecolor=color, edgecolor="none", zorder=10))
