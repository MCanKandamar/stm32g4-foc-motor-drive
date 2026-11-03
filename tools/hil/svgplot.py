"""Tiny SVG plotting / diagram toolkit (vector figures, selectable text).

Units are CSS px; a full text-width figure is W = 600 px (~159 mm).
Palette: validated categorical slots 1-5 (light mode) of the dataviz skill.
"""
import html
import math
import re

INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
S = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948']
FILL_SOFT = ['#e8f1fc', '#fdeee6', '#e6f6f0', '#fdf3dc', '#fbebf1']
FONT = "Calibri, 'Segoe UI', sans-serif"
MFONT = "'Cambria Math', Cambria, serif"
W = 600


def esc(s):
    return html.escape(str(s), quote=True)


def fmt_tick(v, step):
    if abs(v) < 1e-12:
        v = 0.0
    nd = 0
    if step < 1:
        nd = max(0, int(math.ceil(-math.log10(step) - 1e-9)))
        if abs(round(step * 10 ** nd) - step * 10 ** nd) > 1e-6:
            nd += 1
    s = f'{v:.{nd}f}'.replace('-', '−')
    return s


def nice_ticks(lo, hi, n=5):
    span = hi - lo
    raw = span / max(n, 1)
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    start = math.ceil(lo / step - 1e-9) * step
    ticks = []
    v = start
    while v <= hi + 1e-9 * step:
        ticks.append(round(v, 12))
        v += step
    return ticks, step


RICH_RE = re.compile(r'(_\{[^}]*\}|\^\{[^}]*\}|_\w(?!\w)|\^\w(?!\w))')


def rich(text, size):
    """'e_α' / 'x^{2}' -> SVG tspans with sub/superscripts."""
    parts = RICH_RE.split(text)
    out = ''
    small = size * 0.75
    for p in parts:
        if not p:
            continue
        if p.startswith('_') or p.startswith('^'):
            body = p[2:-1] if p[1] == '{' else p[1:]
            shift = '-0.25em' if p[0] == '_' else '0.45em'
            out += (f'<tspan font-size="{small:.1f}" baseline-shift="{shift}">{esc(body)}</tspan>'
                    f'<tspan font-size="{size:.1f}"></tspan>')
        else:
            out += esc(p)
    return out


def text(x, y, s, size=11, anchor='start', color=INK, weight='normal', italic=False,
         font=FONT, rotate=None, baseline='auto'):
    tr = f' transform="rotate({rotate} {x:.1f} {y:.1f})"' if rotate is not None else ''
    st = ' font-style="italic"' if italic else ''
    db = f' dominant-baseline="{baseline}"' if baseline != 'auto' else ''
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-family="{font}" font-size="{size}" '
            f'fill="{color}" text-anchor="{anchor}" font-weight="{weight}"{st}{db}{tr}>'
            f'{rich(s, size)}</text>')


def polyline(pts, color, width=1.6, dash=None, opacity=1.0, join='round', fill='none'):
    d = ' '.join(f'{x:.2f},{y:.2f}' for x, y in pts)
    da = f' stroke-dasharray="{dash}"' if dash else ''
    op = f' stroke-opacity="{opacity}"' if opacity < 1 else ''
    return (f'<polyline points="{d}" fill="{fill}" stroke="{color}" stroke-width="{width}"'
            f' stroke-linejoin="{join}" stroke-linecap="round"{da}{op}/>')


def decimate(xs, ys, max_pts=900):
    n = len(xs)
    if n <= max_pts:
        return xs, ys
    # min/max decimation keeps the envelope of oscillating traces
    bucket = int(math.ceil(n / (max_pts / 2)))
    ox, oy = [], []
    for i in range(0, n, bucket):
        seg = list(range(i, min(i + bucket, n)))
        jmin = min(seg, key=lambda j: ys[j])
        jmax = max(seg, key=lambda j: ys[j])
        for j in sorted({jmin, jmax}):
            ox.append(xs[j]); oy.append(ys[j])
    return ox, oy


class Axes:
    _uid = 0

    def __init__(self, fig, x, y, w, h, xlim, ylim, xlabel='', ylabel='', xticks=None,
                 yticks=None, title=None, grid=True, xtick_fmt=None, ytick_fmt=None,
                 equal=False, frame=False):
        Axes._uid += 1
        self.fig, self.x, self.y, self.w, self.h = fig, x, y, w, h
        self.xlim, self.ylim = xlim, ylim
        self.xlabel, self.ylabel, self.title = xlabel, ylabel, title
        self.xticks, self.yticks = xticks, yticks
        self.grid, self.frame = grid, frame
        self.xtick_fmt, self.ytick_fmt = xtick_fmt, ytick_fmt
        self.items = []
        self.legend_items = []
        self.clip_id = f'{fig.name}-clip{Axes._uid}'

    def X(self, v):
        a, b = self.xlim
        return self.x + (v - a) / (b - a) * self.w

    def Y(self, v):
        a, b = self.ylim
        return self.y + self.h - (v - a) / (b - a) * self.h

    def plot(self, xs, ys, color=S[0], width=1.6, dash=None, label=None, opacity=1.0,
             max_pts=900, step=False):
        if step:
            sx, sy = [], []
            for i in range(len(xs)):
                if i > 0:
                    sx.append(xs[i]); sy.append(ys[i - 1])
                sx.append(xs[i]); sy.append(ys[i])
            xs, ys = sx, sy
            max_pts = max(max_pts, len(xs))
        segs, cur = [], ([], [])
        for a, b in zip(xs, ys):
            if b != b:                      # NaN -> gap
                if cur[0]:
                    segs.append(cur)
                cur = ([], [])
            else:
                cur[0].append(a); cur[1].append(b)
        if cur[0]:
            segs.append(cur)
        for sx, sy in segs:
            sx, sy = decimate(sx, sy, max_pts)
            pts = [(self.X(a), self.Y(b)) for a, b in zip(sx, sy)]
            self.items.append(('clip', polyline(pts, color, width, dash, opacity)))
        if label:
            self.legend_items.append((label, color, dash, width))

    def hline(self, v, color=MUTED, width=1.0, dash='4 3', label=None):
        self.items.append(('clip', polyline([(self.x, self.Y(v)), (self.x + self.w, self.Y(v))], color, width, dash)))
        if label:
            self.legend_items.append((label, color, dash, width))

    def vline(self, v, color=MUTED, width=1.0, dash='4 3'):
        self.items.append(('clip', polyline([(self.X(v), self.y), (self.X(v), self.y + self.h)], color, width, dash)))

    def band(self, x0, x1, y0, y1, color, opacity=0.18):
        X0, X1 = sorted((self.X(x0), self.X(x1)))
        Y0, Y1 = sorted((self.Y(y0), self.Y(y1)))
        self.items.append(('clip', f'<rect x="{X0:.1f}" y="{Y0:.1f}" width="{X1 - X0:.1f}" '
                                   f'height="{Y1 - Y0:.1f}" fill="{color}" fill-opacity="{opacity}"/>'))

    def fill_between(self, xs, y0s, y1s, color, opacity=0.15):
        pts = [(self.X(a), self.Y(b)) for a, b in zip(xs, y1s)]
        pts += [(self.X(a), self.Y(b)) for a, b in reversed(list(zip(xs, y0s)))]
        d = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.items.append(('clip', f'<polygon points="{d}" fill="{color}" fill-opacity="{opacity}"/>'))

    def dots(self, xs, ys, color=S[0], r=3.0, label=None):
        s = ''.join(f'<circle cx="{self.X(a):.1f}" cy="{self.Y(b):.1f}" r="{r}" fill="{color}" '
                    f'stroke="#fcfcfb" stroke-width="1"/>' for a, b in zip(xs, ys))
        self.items.append(('clip', s))
        if label:
            self.legend_items.append((label, color, None, 0))

    def text(self, xv, yv, s, size=10.5, anchor='start', color=INK2, dx=0, dy=0, italic=False,
             weight='normal', font=FONT):
        self.items.append(('free', text(self.X(xv) + dx, self.Y(yv) + dy, s, size, anchor, color,
                                        weight, italic, font)))

    def arrow(self, x0, y0, x1, y1, color=INK2, width=1.0):
        self.items.append(('free', f'<line x1="{self.X(x0):.1f}" y1="{self.Y(y0):.1f}" '
                                   f'x2="{self.X(x1):.1f}" y2="{self.Y(y1):.1f}" stroke="{color}" '
                                   f'stroke-width="{width}" marker-end="url(#{self.fig.name}-arr)"/>'))

    def raw(self, s, clip=True):
        self.items.append(('clip' if clip else 'free', s))

    def legend(self, loc='ur', ncol=1, size=10.5, bg=True):
        items = self.legend_items
        if not items:
            return
        rowh = size + 5
        colw = max(len(lbl) for lbl, *_ in items) * size * 0.5 + 34
        nrow = math.ceil(len(items) / ncol)
        bw, bh = colw * ncol + 6, nrow * rowh + 6
        if loc == 'ur':
            x0, y0 = self.x + self.w - bw - 4, self.y + 4
        elif loc == 'ul':
            x0, y0 = self.x + 4, self.y + 4
        elif loc == 'lr':
            x0, y0 = self.x + self.w - bw - 4, self.y + self.h - bh - 4
        elif loc == 'll':
            x0, y0 = self.x + 4, self.y + self.h - bh - 4
        elif loc == 'above':
            x0, y0 = self.x, self.y - bh - 2
        else:
            x0, y0 = loc
        s = ''
        if bg:
            s += (f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{bw:.1f}" height="{bh:.1f}" rx="3" '
                  f'fill="#ffffff" fill-opacity="0.92" stroke="{GRID}" stroke-width="0.8"/>')
        for k, (lbl, color, dash, width) in enumerate(items):
            r, c = k % nrow, k // nrow
            lx = x0 + 6 + c * colw
            ly = y0 + 3 + r * rowh + rowh / 2
            if width:
                s += polyline([(lx, ly), (lx + 20, ly)], color, max(width, 1.8), dash)
            else:
                s += f'<circle cx="{lx + 10:.1f}" cy="{ly:.1f}" r="3.5" fill="{color}"/>'
            s += text(lx + 26, ly + size * 0.35, lbl, size, color=INK2)
        self.items.append(('free', s))

    def render(self):
        out = [f'<clipPath id="{self.clip_id}"><rect x="{self.x:.1f}" y="{self.y:.1f}" '
               f'width="{self.w:.1f}" height="{self.h:.1f}"/></clipPath>']
        xt = self.xticks if self.xticks is not None else nice_ticks(*self.xlim)[0]
        yt = self.yticks if self.yticks is not None else nice_ticks(*self.ylim)[0]
        xstep = (xt[1] - xt[0]) if len(xt) > 1 else 1
        ystep = (yt[1] - yt[0]) if len(yt) > 1 else 1
        g = []
        if self.grid:
            for v in yt:
                Yv = self.Y(v)
                g.append(f'<line x1="{self.x:.1f}" y1="{Yv:.1f}" x2="{self.x + self.w:.1f}" y2="{Yv:.1f}" '
                         f'stroke="{GRID}" stroke-width="0.8"/>')
            for v in xt:
                Xv = self.X(v)
                g.append(f'<line x1="{Xv:.1f}" y1="{self.y:.1f}" x2="{Xv:.1f}" y2="{self.y + self.h:.1f}" '
                         f'stroke="{GRID}" stroke-width="0.6"/>')
        out += g
        if self.frame:
            out.append(f'<rect x="{self.x:.1f}" y="{self.y:.1f}" width="{self.w:.1f}" height="{self.h:.1f}" '
                       f'fill="none" stroke="{AXIS}" stroke-width="0.9"/>')
        # zero / bottom axis
        out.append(f'<line x1="{self.x:.1f}" y1="{self.y + self.h:.1f}" x2="{self.x + self.w:.1f}" '
                   f'y2="{self.y + self.h:.1f}" stroke="{AXIS}" stroke-width="1"/>')
        out.append(f'<line x1="{self.x:.1f}" y1="{self.y:.1f}" x2="{self.x:.1f}" '
                   f'y2="{self.y + self.h:.1f}" stroke="{AXIS}" stroke-width="1"/>')
        if self.ylim[0] < 0 < self.ylim[1]:
            out.append(f'<line x1="{self.x:.1f}" y1="{self.Y(0):.1f}" x2="{self.x + self.w:.1f}" '
                       f'y2="{self.Y(0):.1f}" stroke="{AXIS}" stroke-width="0.9"/>')
        for v in xt:
            lbl = self.xtick_fmt(v) if self.xtick_fmt else fmt_tick(v, xstep)
            out.append(text(self.X(v), self.y + self.h + 13, lbl, 10, 'middle', INK2))
        for v in yt:
            lbl = self.ytick_fmt(v) if self.ytick_fmt else fmt_tick(v, ystep)
            out.append(text(self.x - 5, self.Y(v) + 3.5, lbl, 10, 'end', INK2))
        if self.xlabel:
            out.append(text(self.x + self.w / 2, self.y + self.h + 29, self.xlabel, 11, 'middle', INK))
        if self.ylabel:
            out.append(text(self.x - 40, self.y + self.h / 2, self.ylabel, 11, 'middle', INK, rotate=-90))
        if self.title:
            out.append(text(self.x, self.y - 7, self.title, 11.5, 'start', INK, weight='bold'))
        clip = [s for kind, s in self.items if kind == 'clip']
        free = [s for kind, s in self.items if kind == 'free']
        out.append(f'<g clip-path="url(#{self.clip_id})">' + ''.join(clip) + '</g>')
        out += free
        return '\n'.join(out)


class Fig:
    def __init__(self, name, w=W, h=260):
        self.name, self.w, self.h = name, w, h
        self.axes_list = []
        self.items = []

    def axes(self, *a, **k):
        ax = Axes(self, *a, **k)
        self.axes_list.append(ax)
        return ax

    def add(self, s):
        self.items.append(s)

    def svg(self):
        defs = (f'<defs><marker id="{self.name}-arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
                f'markerHeight="7" orient="auto-start-reverse"><path d="M0,1 L10,5 L0,9 z" fill="{INK2}"/></marker>'
                f'<marker id="{self.name}-arrb" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
                f'markerHeight="7" orient="auto-start-reverse"><path d="M0,1 L10,5 L0,9 z" fill="{S[0]}"/></marker>'
                f'</defs>')
        body = '\n'.join(ax.render() for ax in self.axes_list) + '\n' + '\n'.join(self.items)
        return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" '
                f'width="{self.w}" height="{self.h}" class="fig-svg" role="img">{defs}{body}</svg>')


# ---------------------------------------------------------------------------
# Diagram primitives
# ---------------------------------------------------------------------------
class Diagram(Fig):
    def box(self, x, y, w, h, label, fill='#f4f7fb', stroke=AXIS, size=11, bold=False, rx=4,
            sub=None, color=INK, dash=None):
        da = f' stroke-dasharray="{dash}"' if dash else ''
        s = (f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" '
             f'fill="{fill}" stroke="{stroke}" stroke-width="1.1"{da}/>')
        lines = label.split('\n')
        n = len(lines) + (1 if sub else 0)
        y0 = y + h / 2 - (n - 1) * (size + 2) / 2 + size * 0.35
        for i, ln in enumerate(lines):
            s += text(x + w / 2, y0 + i * (size + 2), ln, size, 'middle', color,
                      'bold' if bold else 'normal')
        if sub:
            s += text(x + w / 2, y0 + len(lines) * (size + 2), sub, size - 1.5, 'middle', INK2)
        self.items.append(s)
        return (x, y, w, h)

    def circle_sum(self, cx, cy, r=9, signs=None):
        s = (f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="#ffffff" stroke="{INK2}" stroke-width="1.1"/>'
             f'<line x1="{cx - r * 0.7:.1f}" y1="{cy - r * 0.7:.1f}" x2="{cx + r * 0.7:.1f}" y2="{cy + r * 0.7:.1f}" stroke="{INK2}" stroke-width="0.8"/>'
             f'<line x1="{cx - r * 0.7:.1f}" y1="{cy + r * 0.7:.1f}" x2="{cx + r * 0.7:.1f}" y2="{cy - r * 0.7:.1f}" stroke="{INK2}" stroke-width="0.8"/>')
        if signs:
            for (dx, dy, sg) in signs:
                s += text(cx + dx, cy + dy, sg, 10, 'middle', INK)
        self.items.append(s)

    def arrow(self, pts, color=INK2, width=1.2, label=None, lpos=None, dash=None, head=True,
              lsize=10.5, lanchor='middle', lcolor=None):
        d = 'M' + ' L'.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        da = f' stroke-dasharray="{dash}"' if dash else ''
        mk = f' marker-end="url(#{self.name}-arr)"' if head else ''
        s = f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{da}{mk}/>'
        if label:
            if lpos is None:
                (x0, y0), (x1, y1) = pts[0], pts[1]
                lpos = ((x0 + x1) / 2, (y0 + y1) / 2 - 5)
            s += text(lpos[0], lpos[1], label, lsize, lanchor, lcolor or INK)
        self.items.append(s)

    def label(self, x, y, s, size=10.5, anchor='middle', color=INK, weight='normal', italic=False,
              font=FONT):
        self.items.append(text(x, y, s, size, anchor, color, weight, italic, font))

    def dot(self, x, y, r=2.5, color=INK2):
        self.items.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{color}"/>')

    def line(self, pts, color=INK2, width=1.0, dash=None):
        self.items.append(polyline(pts, color, width, dash, join='miter'))

    def rect(self, x, y, w, h, fill='none', stroke=AXIS, width=1.0, rx=0, dash=None, opacity=1.0):
        da = f' stroke-dasharray="{dash}"' if dash else ''
        self.items.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" '
                          f'fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="{width}"{da}/>')

    def raw(self, s):
        self.items.append(s)
