import ctypes
import os
import sys

# crude loader: prefer local build tree
root = os.path.abspath(os.path.join(__file__, "../../.."))
lib_candidates = [
    os.path.join(root, "build", "libape_c.so"),
    os.path.join(root, "build", "ape_c.dll"),
    os.path.join(root, "build", "libape_c.dylib"),
]

_lib = None
for p in lib_candidates:
    if os.path.exists(p):
        _lib = ctypes.CDLL(p)
        break
if _lib is None:
    _lib = ctypes.CDLL("ape_c")

class Vec3(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class RigidBodyDesc(ctypes.Structure):
    _fields_ = [("position", Vec3), ("velocity", Vec3), ("mass", ctypes.c_float)]

_lib.ape_version_major.restype = ctypes.c_uint
_lib.ape_version_minor.restype = ctypes.c_uint
_lib.ape_version_patch.restype = ctypes.c_uint

class World(ctypes.Structure):
    pass

_lib.ape_world_create.restype = ctypes.POINTER(World)
_lib.ape_world_destroy.argtypes = [ctypes.POINTER(World)]
_lib.ape_world_create_rigidbody.argtypes = [ctypes.POINTER(World), RigidBodyDesc]
_lib.ape_world_create_rigidbody.restype = ctypes.c_uint
_lib.ape_world_step.argtypes = [ctypes.POINTER(World), ctypes.c_float]
_lib.ape_world_get_position.argtypes = [ctypes.POINTER(World), ctypes.c_uint]
_lib.ape_world_get_position.restype = Vec3


def version():
    return (_lib.ape_version_major(), _lib.ape_version_minor(), _lib.ape_version_patch())


def smoke():
    w = _lib.ape_world_create()
    d = RigidBodyDesc(position=Vec3(0, 5, 0), velocity=Vec3(0, 0, 0), mass=1.0)
    rid = _lib.ape_world_create_rigidbody(w, d)
    for _ in range(60):
        _lib.ape_world_step(w, ctypes.c_float(1.0/60.0))
    p = _lib.ape_world_get_position(w, rid)
    _lib.ape_world_destroy(w)
    return p.y

if __name__ == "__main__":
    print("version:", ".".join(map(str, version())))
    print("smoke y:", smoke())
