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

"""Open a USD stage and run optional startup Python inside Isaac Sim."""

import argparse
import asyncio
import os
import runpy
import sys
import traceback

import carb
import omni.client
import omni.kit.app
import omni.kit.async_engine
import omni.timeline
import omni.usd


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        type=str,
        default="",
        help="The path to USD stage.",
    )
    parser.add_argument(
        "--python-script",
        type=str,
        default="",
        help="The path to a Python script to run after Isaac Sim starts.",
    )
    parser.add_argument(
        "--start-on-play",
        action="store_true",
        help="If present, start playing after the USD stage is loaded.",
    )

    try:
        options = parser.parse_args()
    except Exception as exc:
        carb.log_error(str(exc))
        return

    if not options.path and not options.python_script:
        carb.log_warn("No USD stage or Python script was provided.")
        return

    omni.kit.async_engine.run_coroutine(
        _run_startup_actions_async(options.path, options.start_on_play, options.python_script)
    )


def _run_python_script(python_script: str) -> bool:
    if not python_script:
        return True

    script_path = os.path.abspath(os.path.expanduser(python_script))
    if not os.path.isfile(script_path):
        carb.log_error(f"Python script path does not exist or is not a file: {script_path}")
        return False

    previous_argv = sys.argv[:]
    sys.argv = [script_path]
    try:
        carb.log_info(f"Running Python script: {script_path}")
        runpy.run_path(script_path, run_name="__main__")
        return True
    except Exception as exc:
        carb.log_error(f"Failed to run Python script {script_path}: {exc}")
        carb.log_error(traceback.format_exc())
        return False
    finally:
        sys.argv = previous_argv


async def _run_startup_actions_async(path: str, start_on_play: bool, python_script: str) -> None:
    if path:
        await open_stage_async(path, start_on_play, python_script)
        return

    if start_on_play:
        carb.log_warn("play_sim_on_start is ignored when no USD stage path is provided.")

    await omni.kit.app.get_app().next_update_async()
    _run_python_script(python_script)


async def open_stage_async(path: str, start_on_play: bool, python_script: str) -> None:
    timeline_interface = None
    if start_on_play:
        timeline_interface = omni.timeline.get_timeline_interface()

    async def _open_stage_internal(stage_path: str) -> None:
        layers = None
        is_stage_with_session = False
        try:
            import omni.kit.usd.layers as layers

            live_session_name = layers.get_live_session_name_from_shared_link(stage_path)
            is_stage_with_session = live_session_name is not None
        except Exception:
            pass

        if is_stage_with_session and layers is not None:
            # Try to open the stage with specified live session.
            success, error = await layers.get_live_syncing().open_stage_with_live_session_async(stage_path)
        else:
            # Otherwise, use normal stage open.
            success, error = await omni.usd.get_context().open_stage_async(stage_path)

        if not success:
            carb.log_error(f"Failed to open stage {stage_path}: {error}.")
            return

        if not _run_python_script(python_script):
            if timeline_interface is not None:
                carb.log_error("Skipping timeline play because the startup Python script failed.")
            return

        if timeline_interface is not None:
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            timeline_interface.play()
            carb.log_info("Stage loaded and simulation is playing.")

    result, _ = await omni.client.stat_async(path)
    if result == omni.client.Result.OK:
        await _open_stage_internal(path)
        return

    broken_url = omni.client.break_url(path)
    if broken_url.scheme == "omniverse":
        # Attempt to connect to nucleus server before opening stage.
        try:
            from omni.kit.widget.nucleus_connector import get_nucleus_connector

            nucleus_connector = get_nucleus_connector()
        except Exception:
            carb.log_warn("Open stage: Could not import Nucleus connector.")
            return

        server_url = omni.client.make_url(scheme="omniverse", host=broken_url.host)
        nucleus_connector.connect(
            broken_url.host,
            server_url,
            on_success_fn=lambda *_: asyncio.ensure_future(_open_stage_internal(path)),
            on_failed_fn=lambda *_: carb.log_error(f"Open stage: Failed to connect to server {server_url}."),
        )
    else:
        carb.log_warn(f"Open stage: Could not open non-existent url {path}.")


main()
