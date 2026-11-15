"""Amplitude-invariant Clarke/Park transforms (3-phase power = 1.5 * (vd*id + vq*iq))."""
import math

SQRT3 = math.sqrt(3.0)
INV_SQRT3 = 1.0 / SQRT3


def clarke(a: float, b: float, c: float) -> tuple[float, float]:
    """abc -> alpha/beta. Insensitive to any common-mode offset in a, b, c."""
    return (2.0 * a - b - c) / 3.0, (b - c) * INV_SQRT3


def inv_clarke(alpha: float, beta: float) -> tuple[float, float, float]:
    h, s = 0.5 * alpha, 0.5 * SQRT3 * beta
    return alpha, -h + s, -h - s


def park(alpha: float, beta: float, theta: float) -> tuple[float, float]:
    """alpha/beta -> d/q (d axis at electrical angle theta from alpha)."""
    c, s = math.cos(theta), math.sin(theta)
    return alpha * c + beta * s, -alpha * s + beta * c


def inv_park(d: float, q: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return d * c - q * s, d * s + q * c
