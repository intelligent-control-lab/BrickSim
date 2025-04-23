import ctypes
import numpy as np
from omni.physx.bindings._physx import ContactEventHeaderVector, ContactDataVector

_CTYPE2NP = {
    ctypes.c_uint8  : np.uint8,
    ctypes.c_int8   : np.int8,
    ctypes.c_uint16 : np.uint16,
    ctypes.c_int16  : np.int16,
    ctypes.c_uint32 : np.uint32,
    ctypes.c_int32  : np.int32,
    ctypes.c_uint64 : np.uint64,
    ctypes.c_int64  : np.int64,
    ctypes.c_float  : np.float32,
    ctypes.c_double : np.float64,
}

def _cfield_to_dtype(name, ctype):
    if ctype in _CTYPE2NP:
        return (name, _CTYPE2NP[ctype])

    # ctype is a fixed‑length array: e.g. <class '__main__.c_float_Array_3'>
    if issubclass(ctype, ctypes.Array) and ctype._type_ in _CTYPE2NP:
        base = _CTYPE2NP[ctype._type_]
        return (name, base, (ctype._length_,))

    raise TypeError(f"field {name!r}: unsupported ctypes type {ctype}")

def _cstruct_to_numpy_dtype(ctype_struct):
    return np.dtype([_cfield_to_dtype(n, t) for n, t in ctype_struct._fields_],
                    align=False)

def _vec_to_numpy(vec, dtype, size):
    ptr_sz = ctypes.sizeof(ctypes.c_void_p)
    base = ctypes.c_void_p.from_address(id(vec[0]) + 2*ptr_sz).value
    byte_buf = (ctypes.c_ubyte * (len(vec)*size)).from_address(base)
    return np.frombuffer(byte_buf, dtype=dtype, count=len(vec))

_Float3 = ctypes.c_float * 3

class _ContactEventHeader_C(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("type",                        ctypes.c_uint32),
        ("_pad",                        ctypes.c_uint32),
        ("stage_id",                    ctypes.c_uint64),
        ("actor0",                      ctypes.c_uint64),
        ("actor1",                      ctypes.c_uint64),
        ("collider0",                   ctypes.c_uint64),
        ("collider1",                   ctypes.c_uint64),
        ("contact_data_offset",         ctypes.c_uint32),
        ("num_contact_data",            ctypes.c_uint32),
        ("friction_anchors_offset",     ctypes.c_uint32),
        ("num_friction_anchors_data",   ctypes.c_uint32),
        ("proto_index0",                ctypes.c_uint32),
        ("proto_index1",                ctypes.c_uint32),
    ]
_ContactEventHeader_SIZE = 72
assert ctypes.sizeof(_ContactEventHeader_C) == _ContactEventHeader_SIZE
_ContactEventHeader_DTYPE = _cstruct_to_numpy_dtype(_ContactEventHeader_C)

class _ContactData_C(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("position",                    _Float3),
        ("normal",                      _Float3),
        ("impulse",                     _Float3),
        ("separation",                  ctypes.c_float),
        ("face_index0",                 ctypes.c_uint32),
        ("face_index1",                 ctypes.c_uint32),
        ("material0",                   ctypes.c_uint64),
        ("material1",                   ctypes.c_uint64),
    ]
_ContactData_SIZE = 64
assert ctypes.sizeof(_ContactData_C) == _ContactData_SIZE
_ContactData_DTYPE  = _cstruct_to_numpy_dtype(_ContactData_C)

def buffer_from_ContactEventHeaderVector(vec: ContactEventHeaderVector) -> np.ndarray:
    return _vec_to_numpy(vec, _ContactEventHeader_DTYPE, _ContactEventHeader_SIZE)

def buffer_from_ContactDataVector(vec: ContactDataVector) -> np.ndarray:
    return _vec_to_numpy(vec, _ContactData_DTYPE, _ContactData_SIZE)
