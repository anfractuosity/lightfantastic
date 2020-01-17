#!/usr/bin/python3

from typing import NamedTuple
from math import floor

class Rect(NamedTuple):
    x1: int
    y1: int
    x2: int
    y2: int

# From:
# http://mauveweb.co.uk/posts/2011/05/introduction-to-spatial-hashes.html

class SpatialHash(object):

    def __init__(self, cell_size=10.0):
        self.cell_size = float(cell_size)
        self.d = {}

    def _add(self, cell_coord, o):
        """Add the object o to the cell at cell_coord."""
        try:
            self.d.setdefault(cell_coord, set()).add(o)
        except KeyError:
            self.d[cell_coord] = set((o,))

    def _cells_for_rect(self, r):
        """Return a set of the cells into which r extends."""
        cells = set()
        cy = floor(r.y1 / self.cell_size)
        while (cy * self.cell_size) <= r.y2:
            cx = floor(r.x1 / self.cell_size)
            while (cx * self.cell_size) <= r.x2:
                cells.add((int(cx), int(cy)))
                cx += 1.0
            cy += 1.0
        return cells

    def add_rect(self, r, obj):
        """Add an object obj with bounds r."""
        cells = self._cells_for_rect(r)
        for c in cells:
            self._add(c, obj)

    def potential_collisions(self, r, obj):
        """Get a set of all objects that potentially intersect obj."""
        cells = self._cells_for_rect(r)
        potentials = set()
        for c in cells:
            potentials.update(self.d.get(c, set()))
        potentials.discard(obj)  # obj cannot intersect itself
        return potentials

