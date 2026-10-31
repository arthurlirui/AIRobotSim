"""
Camera optical models for robot perception simulation.

Every camera type (monochrome/RGB monocular, stereo, iToF depth, thermal
long-wave infrared, near-infrared) shares the same pinhole projection
geometry but differs in **what it measures** and **its noise model**:

=====================  ==========================  ==========================
Camera                 Measures                    Dominant noise
=====================  ==========================  ==========================
MonocularCamera        RGB color                   photon shot + read noise
StereoCamera           disparity → depth           stereo matching error
ITofCamera             per-pixel phase depth       phase ambiguity + multi-
                                                   path + shot noise
ThermalCamera          LWIR 8–14 µm radiance       temperature sensitivity
                                                   (NETD) + fixed-pattern
NearInfraredCamera     NIR 0.75–1.4 µm intensity    active-illumination
                                                   falloff + shot noise
=====================  ==========================  ==========================

All models render against the :class:`~core.gaussian_splat_env.
GaussianSplatEnv` via its ``ray_cast`` query: the scene is a set of
Gaussian ellipsoids, so "rendering" = casting one ray per pixel and taking
the first hit distance + splat color/radiance.  This is deliberately a
**geometry-faithful, appearance-approximate** model — good enough to study
sensor fusion, comm-constrained perception and RL observation models,
while the photorealistic rendering itself is handled by the Three.js /
Splatshop front end.
"""

from sensors.base import (
    CameraIntrinsics, CameraExtrinsics, CameraModel, CameraFrame,
)
from sensors.monocular import MonocularCamera
from sensors.stereo import StereoCamera
from sensors.itof import ITofCamera
from sensors.thermal import ThermalCamera
from sensors.nir import NearInfraredCamera

__all__ = [
    "CameraIntrinsics", "CameraExtrinsics", "CameraModel", "CameraFrame",
    "MonocularCamera", "StereoCamera", "ITofCamera",
    "ThermalCamera", "NearInfraredCamera",
]
