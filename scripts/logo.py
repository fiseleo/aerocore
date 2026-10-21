"""AeroCore 0.41: the logo (neoforge/src/main/resources/aerocore_logo.png, the mod list's, 512 x 256) and the release
icon (512 x 512, under %USERPROFILE%/.mcmod/release/aerocore, not in the project). The picture is physics: the
streamlines of potential flow past a Joukowski airfoil at 6 degrees, its circulation set by the Kutta condition,
over a Stokes wave. Needs numpy, matplotlib and Pillow.

    py -3 scripts/logo.py
"""
import io
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
LOGO = os.path.join(HERE, '..', 'neoforge', 'src', 'main', 'resources', 'aerocore_logo.png')
RELEASE = os.path.join(os.path.expanduser('~'), '.mcmod', 'release', 'aerocore')
FONT = 'C:/Windows/Fonts/segoeuib.ttf'
NAVY_TOP, NAVY_BOTTOM = (10, 22, 40), (16, 46, 78)


def flow(width, height, x0, x1, y0, y1):
    """The stream function on a grid of the z-plane, the airfoil's outline, and which grid points are inside it."""
    c, zeta0, alpha = 1.0, complex(-0.09, 0.09), np.radians(6)
    r = abs(c - zeta0)
    x, y = np.meshgrid(np.linspace(x0, x1, width), np.linspace(y0, y1, height))
    z = x + 1j * y
    root = np.sqrt(z * z - 4 * c * c)
    a, b = (z + root) / 2, (z - root) / 2
    zeta = np.where(np.abs(a - zeta0) >= np.abs(b - zeta0), a, b)
    # Kutta: no velocity at the trailing edge (zeta = c)
    s = c - zeta0
    gamma = (-2j * np.pi * s * (np.exp(-1j * alpha) - r * r * np.exp(1j * alpha) / (s * s))).real
    w = (zeta - zeta0) * np.exp(-1j * alpha) + r * r * np.exp(1j * alpha) / (zeta - zeta0) + 1j * gamma / (2 * np.pi) * np.log(zeta - zeta0)
    inside = np.abs(zeta - zeta0) < r * 1.0005
    theta = np.linspace(0, 2 * np.pi, 400)
    circle = zeta0 + r * np.exp(1j * theta)
    outline = circle + c * c / circle
    return w.imag, inside, outline


def picture(width, height, x0, x1, y0, y1, wave_y):
    """The flow and the wave on a navy gradient, as an RGBA image."""
    dpi = 100
    fig = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.axis('off')
    grad = np.linspace(0, 1, 256)[:, None]
    top, bottom = np.array(NAVY_TOP) / 255, np.array(NAVY_BOTTOM) / 255
    ax.imshow(top * (1 - grad[..., None]) + bottom * grad[..., None], extent=(x0, x1, y0, y1), aspect='auto', zorder=0)
    psi, inside, outline = flow(width * 2, height * 2, x0, x1, y0, y1)
    psi = np.ma.array(psi, mask=inside)
    levels = np.linspace(-1.6, 1.6, 23)
    ax.contour(np.linspace(x0, x1, width * 2), np.linspace(y0, y1, height * 2), psi, levels=levels,
               colors=[(0.45, 0.85, 1.0, 0.85)], linewidths=1.1, negative_linestyles='solid', zorder=1)
    ax.fill(outline.real, outline.imag, color=(0.96, 0.97, 1.0), zorder=2)
    # a second-order Stokes wave: sharper crests, flatter troughs
    xs = np.linspace(x0, x1, 600)
    k, amp = 2 * np.pi / 1.9, 0.10
    ys = wave_y + amp * np.cos(k * xs) + 0.5 * k * amp * amp * np.cos(2 * k * xs)
    ax.fill_between(xs, y0, ys, color=(0.10, 0.36, 0.62), zorder=3)
    ax.plot(xs, ys, color=(0.62, 0.86, 1.0), linewidth=1.6, zorder=4)
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=dpi)
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert('RGBA')


def text(im, words, size, xy, fill=(255, 255, 255, 255)):
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT, size)
    d.text(xy, words, font=f, fill=fill)


def main():
    logo = picture(512, 256, -2.6, 2.6, -1.25, 1.35, -0.92)
    shade = Image.new('RGBA', logo.size, (0, 0, 0, 0))
    ImageDraw.Draw(shade).rectangle((0, 0, 512, 64), fill=(6, 14, 28, 150))
    logo = Image.alpha_composite(logo, shade)
    text(logo, 'AeroCore', 40, (16, 6))
    text(logo, 'physics for vehicle mods', 16, (206, 26), (170, 220, 255, 255))
    os.makedirs(os.path.dirname(LOGO), exist_ok=True)
    logo.convert('RGB').save(LOGO, optimize=True)
    icon = picture(512, 512, -2.2, 2.2, -2.0, 2.4, -1.45)
    os.makedirs(RELEASE, exist_ok=True)
    icon.convert('RGB').save(os.path.join(RELEASE, 'icon.png'), optimize=True)
    print('logo', LOGO, 'icon', os.path.join(RELEASE, 'icon.png'))


if __name__ == '__main__':
    main()
