"""State of charge of a Li-ion pack from its voltage."""

import numpy as np

# Open-circuit voltage of one Li-ion cell (V) against its charge (%), for 18650 cells like the
# Ampace JP30. Under load the voltage sags, so the estimate reads low while the robot walks.
CELL_VOLTAGE = (3.00, 3.30, 3.45, 3.55, 3.65, 3.75, 3.85, 3.95, 4.05, 4.20)
CELL_CHARGE = (0.0, 5.0, 10.0, 20.0, 35.0, 50.0, 65.0, 80.0, 90.0, 100.0)


def charge(pack_voltage: float, cells: int) -> float:
    """Charge in [0, 1] of a pack of `cells` cells in series."""
    return float(np.interp(pack_voltage / cells, CELL_VOLTAGE, CELL_CHARGE)) / 100.0
