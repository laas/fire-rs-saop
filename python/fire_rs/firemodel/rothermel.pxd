

cdef class RothermelResult:
    cdef readonly double ros  # Rate of Spread [m/s
    cdef readonly double wind_factor
    cdef readonly double slope_factor
    # Equivalent slope gives the strength of the wind that would give a similar effect to the one of the slope [Lopes 02]
    cdef readonly double equivalent_slope