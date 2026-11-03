"""Draw the README plots (SVG) from the newest HIL logs.

Usage: python tools/hil/plot_readme.py
Reads tools/hil/logs/ (step_hall_1000_1300_*, step_sensorless_1000_1300_*, startup_1000_*, angle_1000_*, waves_1000_*)
and writes docs/img/speed_step.svg, sensorless_startup.svg, angle_compare.svg and emf_duties.svg.
"""
import csv
import math
from pathlib import Path

import svgplot as sp

HERE = Path(__file__).parent
LOGS = HERE / 'logs'
IMG = HERE.parent.parent / 'docs' / 'img'
POLE_PAIRS = 4
STATES = {2: 'ALIGN', 3: 'OPEN LOOP', 4: 'CLOSED LOOP'}
GREY = '#7d7c76'


def newest(pattern):
    files = sorted(LOGS.glob(pattern))
    if not files:
        raise SystemExit(f"no log matches {pattern}")
    return files[-1]


def load(path):
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def save(fig, name):
    svg = fig.svg()
    i = svg.index('</defs>') + len('</defs>')
    svg = svg[:i] + '<rect width="100%" height="100%" fill="#ffffff"/>' + svg[i:]   # readable in dark mode
    (IMG / name).write_text(svg, encoding='utf-8')
    print("wrote", IMG / name)


def speed_step():
    runs = [('Hall', load(newest('step_hall_1000_1300_*.csv')), sp.S[0]),
            ('Sensorless (SMO)', load(newest('step_sensorless_1000_1300_*.csv')), sp.S[1])]
    fig = sp.Fig('speed-step', 640, 420)
    a1 = fig.axes(70, 34, 540, 200, (-50, 500), (950, 1400), ylabel='speed [rpm]',
                  title='Speed step 1000 → 1300 rpm (no load)')
    a2 = fig.axes(70, 280, 540, 90, (-50, 500), (-0.5, 1.2), xlabel='time from the step [ms]',
                  ylabel='iq ref [A]', yticks=[0, 0.5, 1.0])
    for i, (name, d, color) in enumerate(runs):
        i0 = next(k for k, r in enumerate(d['ref']) if r > 1150.0)
        t = [(v - d['t_s'][i0]) * 1000.0 for v in d['t_s']]
        if i == 0:
            a1.plot(t, d['ref'], GREY, 1.4, dash='5 3', label='reference')
        a1.plot(t, d['fb'], color, 1.8, label=name)
        a2.plot(t, d['iq_ref'], color, 1.4)
    a1.legend('lr')
    save(fig, 'speed_step.svg')


def sensorless_startup():
    d = load(newest('startup_1000_*.csv'))
    t, st = d['t_s'], [int(round(v)) for v in d['state']]
    ol = [w / POLE_PAIRS * 60.0 / (2.0 * math.pi) if s == 3 else math.nan for w, s in zip(d['ol_speed'], st)]
    ref = [r if s == 4 else math.nan for r, s in zip(d['ref'], st)]
    fig = sp.Fig('startup', 640, 420)
    a1 = fig.axes(70, 34, 540, 200, (0, t[-1]), (-300, 1200), ylabel='speed [rpm]',
                  title='Sensorless start-up 0 → 1000 rpm (no load)')
    a2 = fig.axes(70, 280, 540, 90, (0, t[-1]), (-0.5, 2.5), xlabel='time [s]', ylabel='iq ref [A]',
                  yticks=[0, 1, 2])
    k = 0
    while k < len(st):                          # shade each start-up state
        j = k
        while j + 1 < len(st) and st[j + 1] == st[k]:
            j += 1
        if st[k] in STATES:
            for ax, (lo, hi) in ((a1, (-300, 1200)), (a2, (-0.5, 2.5))):
                ax.band(t[k], t[j], lo, hi, sp.FILL_SOFT[st[k] - 2], 1.0)
            a1.text((t[k] + t[j]) / 2, 1120, STATES[st[k]], 10, 'middle', sp.INK2, weight='bold')
        k = j + 1
    a1.plot(t, ref, GREY, 1.4, dash='5 3', label='speed reference')
    a1.plot(t, ol, sp.S[2], 1.8, label='open-loop field speed')
    a1.plot(t, d['hall'], sp.S[0], 1.4, label='Hall speed (comparison only)')
    a1.plot(t, d['smo'], sp.S[1], 1.8, label='SMO speed')
    a1.legend('lr')
    a2.plot(t, d['iq_ref'], sp.S[3], 1.6)
    save(fig, 'sensorless_startup.svg')


def angle_compare():
    d = load(newest('angle_1000_*.csv'))
    t = [(v - d['t_s'][0]) * 1000.0 for v in d['t_s']]
    deg = lambda xs: [math.degrees(v) % 360.0 for v in xs]

    def gaps(xs):                               # break the line at each 360° → 0° wrap
        out = [xs[0]]
        for a, b in zip(xs, xs[1:]):
            out.append(math.nan if abs(b - a) > 180.0 else b)
        return out

    err = [math.degrees(math.remainder(s + o - h, 2.0 * math.pi))
           for h, s, o in zip(d['hall'], d['smo'], d['offset'])]
    fig = sp.Fig('angle', 640, 420)
    a1 = fig.axes(70, 34, 540, 200, (0, 45), (0, 460), ylabel='electrical angle [°]',
                  yticks=[0, 90, 180, 270, 360], title='Rotor angle at 1000 rpm, sensorless control')
    a2 = fig.axes(70, 280, 540, 90, (0, t[-1]), (-6, 6), xlabel='time [ms]',
                  ylabel='SMO − Hall [°]', yticks=[-5, 0, 5])
    a1.plot(t, gaps(deg(d['hall'])), sp.S[0], 3.2, label='Hall PLL angle (reference)', opacity=0.45)
    a1.plot(t, gaps(deg(d['smo'])), sp.S[1], 1.4, label='SMO angle (used by the FOC)')
    a1.legend('ul')
    a2.plot(t, err, sp.S[1], 1.2)
    save(fig, 'angle_compare.svg')


def emf_duties():
    d = load(newest('waves_1000_*.csv'))
    t = [(v - d['t_s'][0]) * 1000.0 for v in d['t_s']]
    fig = sp.Fig('waves', 640, 420)
    a1 = fig.axes(70, 34, 540, 170, (0, 45), (-6, 6), ylabel='back-EMF [V]', yticks=[-4, -2, 0, 2, 4],
                  title='SMO back-EMF and PWM duty cycles at 1000 rpm, sensorless control')
    a2 = fig.axes(70, 250, 540, 120, (0, 45), (0.3, 0.8), xlabel='time [ms]', ylabel='duty cycle',
                  yticks=[0.3, 0.4, 0.5, 0.6, 0.7])
    a1.plot(t, d['z_alpha'], sp.S[0], 1.6, label='e_α estimate')
    a1.plot(t, d['z_beta'], sp.S[1], 1.6, label='e_β estimate')
    a1.legend('ur', ncol=2)
    for k, color in (('duty_a', sp.S[0]), ('duty_b', sp.S[1]), ('duty_c', sp.S[2])):
        a2.plot(t, d[k], color, 1.5, label=k.replace('duty_', 'phase ') + '  ')   # spaces: room for the wider glyphs
    a2.legend('ur', ncol=3)
    save(fig, 'emf_duties.svg')


if __name__ == '__main__':
    IMG.mkdir(exist_ok=True)
    speed_step()
    sensorless_startup()
    angle_compare()
    emf_duties()
