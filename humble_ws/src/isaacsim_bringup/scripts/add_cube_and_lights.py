#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Add a sample cube, lights, and camera to the currently opened Isaac Sim stage."""

from __future__ import annotations

import carb
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

WORLD_PATH = Sdf.Path("/World")
SAMPLE_ROOT_PATH = WORLD_PATH.AppendChild("BringupSample")
CUBE_PATH = SAMPLE_ROOT_PATH.AppendChild("Cube")
DOME_LIGHT_PATH = SAMPLE_ROOT_PATH.AppendChild("DomeLight")
KEY_LIGHT_PATH = SAMPLE_ROOT_PATH.AppendChild("KeyLight")
FILL_LIGHT_PATH = SAMPLE_ROOT_PATH.AppendChild("FillLight")
CAMERA_PATH = SAMPLE_ROOT_PATH.AppendChild("Camera")


def _set_xform(
    prim: UsdGeom.Xformable,
    translate: tuple[float, float, float],
    rotate: tuple[float, float, float],
) -> None:
    xform_api = UsdGeom.XformCommonAPI(prim)
    xform_api.SetTranslate(Gf.Vec3d(*translate))
    xform_api.SetRotate(Gf.Vec3f(*rotate), UsdGeom.XformCommonAPI.RotationOrderXYZ)


def _create_cube(stage: Usd.Stage) -> None:
    cube = UsdGeom.Cube.Define(stage, CUBE_PATH)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.45, 1.0)])
    _set_xform(cube, translate=(0.0, 0.0, 0.5), rotate=(0.0, 0.0, 0.0))


def _create_lights(stage: Usd.Stage) -> None:
    dome_light = UsdLux.DomeLight.Define(stage, DOME_LIGHT_PATH)
    dome_light.CreateIntensityAttr(350.0)
    dome_light.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))

    key_light = UsdLux.SphereLight.Define(stage, KEY_LIGHT_PATH)
    key_light.CreateIntensityAttr(18000.0)
    key_light.CreateRadiusAttr(1.0)
    key_light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.86))
    _set_xform(key_light, translate=(3.5, -4.0, 5.0), rotate=(0.0, 0.0, 0.0))

    fill_light = UsdLux.DistantLight.Define(stage, FILL_LIGHT_PATH)
    fill_light.CreateIntensityAttr(500.0)
    fill_light.CreateAngleAttr(0.6)
    fill_light.CreateColorAttr(Gf.Vec3f(0.7, 0.8, 1.0))
    _set_xform(fill_light, translate=(0.0, 0.0, 0.0), rotate=(-45.0, 20.0, 20.0))


def _create_camera(stage: Usd.Stage) -> None:
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    camera.CreateFocalLengthAttr(28.0)
    _set_xform(camera, translate=(3.0, -5.0, 2.5), rotate=(70.0, 0.0, 32.0))

    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = str(CAMERA_PATH)
    except Exception as exc:
        carb.log_warn(f"Could not set active viewport camera: {exc}")


def main() -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("No USD stage is open. Launch with gui:=<stage.usd> or open a stage before running this script.")
        return

    if not stage.GetPrimAtPath(WORLD_PATH):
        UsdGeom.Xform.Define(stage, WORLD_PATH)
    UsdGeom.Xform.Define(stage, SAMPLE_ROOT_PATH)
    _create_cube(stage)
    _create_lights(stage)
    _create_camera(stage)

    carb.log_info(f"Added bringup sample cube, lights, and camera under {SAMPLE_ROOT_PATH}.")


main()
