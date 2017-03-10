

cdef int DYNAMIC_FUEL_MODEL = 0
cdef int STATIC_FUEL_MODEL = 1

cdef class FuelModel:
    cdef readonly int is_dynamic  # as in C: 1 is true, 0 is false
    cdef readonly double[5] loads
    cdef readonly double[5] savs
    cdef readonly double depth
    cdef readonly double mx_dead
    cdef readonly double[5] heat_contents

    
cdef class MoistureScenario:
    cdef readonly double[5] moistures
