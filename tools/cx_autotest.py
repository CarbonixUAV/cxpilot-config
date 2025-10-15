"""
Fly Carbonix aircraft in SITL
"""
import os
import sys
import shutil
import argparse
import functools
from typing import Any, Optional
from pathlib import Path
from pymavlink import mavutil

import sitl_tools
from paths import CXPILOT_ROOT, CXPILOT_CORE_ROOT, CXPILOT_CONFIG_ROOT

sys.path.insert(0, str(CXPILOT_CORE_ROOT / "Tools" / "autotest"))
from pysim import util  # noqa: E402
from quadplane import AutoTestQuadPlane  # noqa: E402
from vehicle_test_suite import AutoTestTimeoutException, NotAchievedException, Test, TestSuite  # noqa: E402

PLANE_BINARY = CXPILOT_CORE_ROOT / "build" / "sitl" / "bin" / "arduplane"


class AutoTestCarbonix(AutoTestQuadPlane):
    """Base class for Carbonix SITL autotests"""
    @classmethod
    @functools.lru_cache(maxsize=1)
    def get_frames(cls) -> dict[str, dict[str, Any]]:
        """
        Get the valid frames for this subclass.
        Returns:
            dict: Dictionary of frame names and their details.
        """
        raise NotImplementedError("This method should be overridden in subclasses")

    def __init__(self, binary, **kwargs):
        super().__init__(binary, **kwargs)
        if self.logs_dir is None:
            self.logs_dir = self.buildlogs_dirpath()

        if not isinstance(self.frame, str):
            raise TypeError(f"Frame must be a string, got {type(self.frame).__name__}")
        if self.frame not in self.get_frames():
            raise ValueError(
                f"Invalid frame: {self.frame} for AutoTestHeadless. Valid frames are: {list(self.get_frames().keys())}"
            )

        defaults = self.model_defaults_filepath(self.frame)
        assert len(defaults) == 1, f"Expected one defaults file for {self.frame}, got {defaults}"
        defaults = defaults[0]
        sitl_tools.write_defaults_file(
            self.frame,
            defaults_out=Path(defaults),
            strip=True,
        )
        self.install_frame_scripts()

    def default_parameter_list(self):
        return super().default_parameter_list() | {
            "ARMING_MIS_ITEMS": 0,      # disable mission check
            "BRD_SAFETY_DEFLT": 0,      # disable safety switch
            "FENCE_AUTOENABLE": 0,      # disable fences
            "FENCE_ENABLE": 0,
            "FS_GCS_ENABL": 0,          # disable GCS failsafe
            "TERRAIN_FOLLOW": 0,        # disable terrain follow (causes prearm fail; terrain requests require extra steps)
        }

    def model_defaults_filepath(self, model):
        return [str(CXPILOT_CORE_ROOT / "build" / "sitl" / f"{model}.parm")]

    def get_model(self, frame):
        model = self.get_frames()[frame].get('model', '')
        if ":" in model and model.endswith('.json'):
            # Convert to absolute path
            model, model_json = model.split(':', 1)
            # ArduPilot needs a relative path to the model JSON
            # (os.path.relpath, unlike Path().relative_to(...) works correctly
            # when the target is outside the current working directory)
            model_json = os.path.relpath(
                CXPILOT_CONFIG_ROOT / model_json,
                Path.cwd()
            )
            model = f"{model}:{model_json}"
        return model

    def install_frame_scripts(self):
        '''installs all scripts specified for the frame in vehicleinfo'''
        dest_root = Path("scripts").resolve()
        shutil.rmtree(dest_root, ignore_errors=True)  # Clean up existing scripts first
        sitl_tools.copy_scripts(
            str(self.frame),
            dest_root=dest_root,
            symlink=True,
        )

    def assert_no_text(self, *args, **kwargs):
        '''Assert that a text message does not come in within a timeout'''
        try:
            text = self.wait_text(*args, **kwargs)
        except AutoTestTimeoutException:
            return
        raise AssertionError(f"Text '{text}' appeared")

    def assert_receive_named_value_float(self, name, timeout=10):
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get NAMED_VALUE_FLOAT %s" % name)
            m = self.assert_receive_message('NAMED_VALUE_FLOAT', timeout=timeout)
            if m.name != name:
                continue
            return m

    def wait_not_ready_to_arm(self, timeout=5):
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK, True, True, False, timeout=timeout)

    def wait_for_engine_temp(self, idx=1, is_cht=True, temp_min=-280, temp_max=6000, timeout=10):
        """
        Waits until the desired engine temperature is reached.

        Args:
            idx (int): The index of the cylinder to check.
            is_cht (bool): If True, checks Cylinder Head Temperature (CHT), otherwise checks Exhaust Gas Temperature (EGT).
            temp_min (int): Minimum temperature to wait for.
            temp_max (int): Maximum temperature to wait for.
            timeout (int): Maximum time to wait in seconds.
        """

        if idx == 1 and is_cht:
            def get_temp():
                return self.assert_receive_message('EFI_STATUS', timeout=timeout).cylinder_head_temperature
        elif idx == 2 and is_cht:
            def get_temp():
                return self.assert_receive_named_value_float('CHT2', timeout=timeout).value
        elif idx == 1 and not is_cht:
            def get_temp():
                return self.assert_receive_message('EFI_STATUS', timeout=timeout).exhaust_gas_temperature
        elif idx == 2 and not is_cht:
            def get_temp():
                return self.assert_receive_named_value_float('EGT2', timeout=timeout).value
        else:
            raise ValueError(f'Invalid CHT index {idx}, must be 1 or 2')

        def validator(cht, _):
            return temp_min <= cht <= temp_max
        self.wait_and_maintain(
            value_name=f'CHT{idx}' if is_cht else f'EGT{idx}',
            target=(temp_min + temp_max) / 2,
            current_value_getter=get_temp,
            accuracy=(temp_max - temp_min),
            validator=validator,
            timeout=timeout,
        )

    def disabled_tests(self):
        return dict()


class AutoTestHeadless(AutoTestCarbonix):
    """
    Carbonix SITL autotest for headless operation.
    """
    @classmethod
    @functools.lru_cache(maxsize=1)
    def get_frames(cls):
        return {k: v for k, v in sitl_tools.get_frames().items() if not v.get('external', False)}

    def log_name(self):  # pyright: ignore[reportIncompatibleMethodOverride]
        return f"{self.frame}"

    def CX_BIT(self):
        '''Test Carbonix's Built-in-Test (BIT) script'''

        def TestESCTelemetry(index):
            '''Test a single ESC'''
            index = int(index)
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.wait_ready_to_arm()

            self.assert_no_text('^CX_BIT:.*', regex=True, check_context=True)

            # Fail the ESC telemetry for the specified index
            self.progress(f'Failing ESC telemetry for ESC {index}')
            self.set_parameter('SIM_ESC_TLM_FAIL', 1 << index)

            # Wait for the prearm failure and the error message
            self.wait_not_ready_to_arm()
            lost_text = f'CX_BIT: ESC {index + 1} Telemetry Lost'
            self.wait_text(lost_text, check_context=True)
            self.progress("'" + lost_text + "':" + ' Success!')

            # Check that the prearm disable parameter works
            self.progress('Checking prearm disable parameter')
            self.set_parameter('BIT_PREARM_DIS', 0b1)
            self.wait_ready_to_arm()
            self.set_parameter('BIT_PREARM_DIS', 0)
            self.wait_not_ready_to_arm()

            # Clear the failure
            self.progress(f'Clearing ESC telemetry failure for ESC {index}')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ESC_TLM_FAIL', 0)
            recovered_text = f'CX_BIT: ESC {index + 1} Telemetry Recovered'
            self.wait_text(recovered_text, check_context=True)
            # Confirm we didn't get lost/recovered/lost/recovered during that time
            self.assert_no_text(lost_text, timeout=1, regex=True, check_context=True)

            # Engage the safety switch and trigger a telemetry failure, and
            # confirm that we don't get the telemetry lost message
            self.progress(f'Engaging safety switch and failing ESC telemetry for ESC {index}')
            self.context_clear_collection('STATUSTEXT')
            self.set_safetyswitch_on()
            self.set_parameter('SIM_ESC_TLM_FAIL', 1 << index)
            self.assert_no_text('^CX_BIT:.*', regex=True, check_context=True)
            self.set_parameter('SIM_ESC_TLM_FAIL', 0)
            self.set_safetyswitch_off()

            # And one more time, confirm no error messages are present
            self.context_clear_collection('STATUSTEXT')
            self.assert_no_text('^CX_BIT:.*', regex=True, check_context=True)
            self.context_pop()

        def TestMotorFail(esc_index, servo_index, is_pusher=False):
            '''Test a single VTOL motor failure'''
            esc_index = int(esc_index)
            servo_index = int(servo_index)
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.wait_ready_to_arm()

            self.arm_vehicle()
            if is_pusher:
                self.change_mode('MANUAL')
                self.set_rc(3, 1500)
            else:
                self.change_mode('QSTABILIZE')

            # Confirm no error messages are present
            self.assert_no_text('CX_BIT.*', regex=True, check_context=True)

            # Fail the ESC telemetry for the specified index
            self.progress(f'Failing Motor {esc_index}')
            self.set_parameter('SIM_ENGINE_FAIL', 1 << servo_index)

            # Wait for the error message
            lost_text = f'CX_BIT: ESC {esc_index + 1} RPM Drop'
            self.wait_text(lost_text, check_context=True)
            self.progress("'" + lost_text + "':" + ' Success!')

            # Clear the failure
            self.progress(f'Fixing Motor {esc_index}')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ENGINE_FAIL', 0)
            recovered_text = f'CX_BIT: ESC {esc_index + 1} RPM Recovered'
            self.wait_text(recovered_text, check_context=True)
            # Confirm we didn't get lost/recovered/lost/recovered during that time
            self.assert_no_text(lost_text, timeout=1, regex=True, check_context=True)

            # And one more time, confirm no error messages are present
            self.context_clear_collection('STATUSTEXT')
            self.assert_no_text('CX_BIT.*', regex=True, check_context=True)
            self.disarm_vehicle()
            self.context_pop()

        def TestGPSPrearm(index):
            '''Test GPS prearm checks'''
            index = int(index)
            self.context_push()

            # Get the parameter names for the GPSs
            if index == 0:
                this_numsats = 'SIM_GPS_NUMSATS'
                other_numsats = 'SIM_GPS2_NUMSATS'
                this_fixtype = 'SIM_GPS_FIXTYPE'
            elif index == 1:
                this_numsats = 'SIM_GPS2_NUMSATS'
                other_numsats = 'SIM_GPS_NUMSATS'
                this_fixtype = 'SIM_GPS2_FIXTYPE'
            else:
                raise ValueError('Only 2 GPSs are supported')
            self.set_parameters({this_numsats: 30, other_numsats: 30})
            self.wait_ready_to_arm()

            # Reduce the number of satellites to trigger a prearm failure
            self.progress(f'Reducing number of satellites for GPS {index}')
            self.set_parameter(this_numsats, 6)
            self.wait_not_ready_to_arm()

            # Check that the prearm disable parameter works
            self.progress('Checking prearm disable parameter')
            self.set_parameter('BIT_PREARM_DIS', 0b10)
            self.wait_ready_to_arm()
            self.set_parameter('BIT_PREARM_DIS', 0)
            self.wait_not_ready_to_arm()

            # Restore the number of satellites
            self.progress(f'Restoring number of satellites for GPS {index}')
            self.set_parameter(this_numsats, 30)
            self.wait_ready_to_arm()

            # Cause a large satellite count difference between the two GPSs
            self.progress('Causing large satellite count difference')
            self.set_parameters({this_numsats: 30, other_numsats: 50})
            self.wait_not_ready_to_arm()

            # Check that our script does not complain about low satellite count
            # when there is insufficient fix (the existing GPS prearm check
            # should be the one to complain about lack of fix)
            self.progress('Checking low satellite count with no fix')
            self.set_parameter(this_fixtype, 1)  # No fix
            # Disable ArduPilot's GPS prearm check to use wait_ready_to_arm
            # (23 bits, except for bits 0 and 3)
            self.set_parameter('ARMING_CHECK', 0x7FFFF6)
            self.wait_ready_to_arm()

            # Restore everything
            self.context_pop()

        def TestEngineWarnings():
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.wait_ready_to_arm()

            self.progress('Starting engine')
            self.set_rc(3, 1000)
            self.change_mode('MANUAL')
            self.set_safetyswitch_off()
            self.run_cmd_int(
                command=mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL,
                p1=1,  # Start the engine
            )
            self.wait_rpm(1, 1000, 9000, timeout=10)
            self.progress('Engine started successfully')
            self.wait_text('Engine cold', check_context=True)

            self.progress('Waiting for engine warmup')
            self.wait_for_engine_temp(idx=1, temp_min=120, temp_max=300, timeout=600)
            self.wait_for_engine_temp(idx=2, temp_min=120, temp_max=300, timeout=600)
            self.wait_text('Engine needs runup to', check_context=True)

            self.progress('Engine runup')
            self.set_rc(3, 2000)
            self.wait_rpm(1, 6500, 8000, timeout=10)
            self.set_rc(3, 1000)
            self.wait_rpm(1, 2000, 4000, timeout=10)
            self.wait_ready_to_arm()

            self.progress('Overheat engine')
            self.context_clear_collection('STATUSTEXT')
            self.set_rc(3, 2000)
            self.wait_for_engine_temp(idx=1, temp_min=280, temp_max=600, timeout=240)
            self.wait_not_ready_to_arm()
            self.wait_text('Engine hot', check_context=True)
            self.set_rc(3, 1000)
            self.wait_for_engine_temp(idx=1, temp_min=100, temp_max=280, timeout=240)
            self.wait_ready_to_arm()

            self.progress('EGT overheat')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ICE_EGT1_INC', 900)
            self.set_parameter('SIM_ICE_EGT2_INC', 900)
            self.set_rc(3, 1500)
            self.wait_for_engine_temp(is_cht=False, temp_min=740, temp_max=1000, timeout=600)
            self.wait_not_ready_to_arm()
            self.wait_text('Engine hot', check_context=True)
            self.set_parameter('SIM_ICE_EGT1_INC', 0)
            self.set_parameter('SIM_ICE_EGT2_INC', 0)
            self.set_rc(3, 1000)
            self.wait_for_engine_temp(is_cht=False, temp_min=100, temp_max=700, timeout=600)
            self.wait_ready_to_arm()

            self.progress('Large CHT difference')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ICE_CHT1_INC', -300)
            self.set_rc(3, 1500)
            self.wait_not_ready_to_arm(timeout=600)
            self.wait_text('CHT difference', check_context=True)
            self.set_rc(3, 1000)
            self.set_parameter('SIM_ICE_CHT1_INC', 0)
            self.wait_ready_to_arm(timeout=600)

            self.progress('Large EGT difference')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ICE_EGT1_INC', -300)
            self.set_rc(3, 1500)
            self.wait_not_ready_to_arm(timeout=600)
            self.wait_text('EGT difference', check_context=True)
            self.set_rc(3, 1000)
            self.set_parameter('SIM_ICE_EGT1_INC', 0)
            self.wait_ready_to_arm(timeout=600)

            # Restore everything
            self.context_pop()

        # Count the number of ESCs
        frame_class = self.get_parameter('Q_FRAME_CLASS')
        if frame_class == 1:  # Quad
            num_vtols = 4
        elif frame_class == 4:  # OctaQuad
            num_vtols = 8
        else:
            raise ValueError(f'Unsupported frame class {frame_class}')
        has_engine = self.get_parameter('ICE_ENABLE')

        # Find the servo assignments for the ESCs
        vtol_servos = {}
        pusher_servo = None
        for i in range(1, 32):
            try:
                assignment = self.get_parameter(f'SERVO{i}_FUNCTION')
                if 33 <= assignment <= 40:
                    vtol_servos[assignment - 33] = i - 1
                elif assignment == 70:
                    pusher_servo = i - 1
            except NotAchievedException:
                break
        assert len(vtol_servos) == num_vtols
        assert pusher_servo is not None

        self.start_subtest('Test ESC telemetry warnings')
        for i in range(num_vtols):
            TestESCTelemetry(i)
        if not has_engine:
            TestESCTelemetry(num_vtols)  # Test the pusher

        self.start_subtest('Test VTOL motor failures')
        for i in range(num_vtols):
            TestMotorFail(i, vtol_servos[i])
        if not has_engine:
            TestMotorFail(num_vtols, pusher_servo, is_pusher=True)

        self.start_subtest('Test GPS')
        for i in range(2):
            TestGPSPrearm(i)

        self.start_subtest('Test engine warnings')
        if has_engine:
            TestEngineWarnings()

    def tests(self):
        return [
            self.CX_BIT,
        ]


STEPS = {
    'test.Headless': AutoTestHeadless,
}


def _run_one(tester_cls: type[TestSuite], frame: str, subtest: Optional[str]) -> tuple[bool, TestSuite]:
    tester = tester_cls(str(PLANE_BINARY), frame=frame)
    if subtest:
        tests = []
        for t in tester.tests():
            if not isinstance(t, Test):
                t = Test(t)
            if t.name == subtest:
                tests = [t]
                break
        if not tests:
            raise ValueError(f"Subtest not found: {subtest}")
        result = tester.autotest(
            tests=tests,
            allow_skips=False,
            step_name=f"test.{tester_cls.__name__}.{frame}.{subtest}",
        )
    else:
        result = tester.autotest(None, step_name=f"test.{tester_cls.__name__}.{frame}")
    return result, tester


def main():
    parser = argparse.ArgumentParser(description="Run Carbonix SITL autotests")
    parser.add_argument('--build', action='store_true', help="Build the SITL binary before running tests")
    parser.add_argument('--no-clean', action='store_true', help="Do not run waf with --clean")
    parser.add_argument('tests', nargs='*', default=None,
                        help="Specs: test.<Class>[.<Subtest>][.<frame>]")
    args = parser.parse_args()

    os.chdir(CXPILOT_CORE_ROOT)

    buildlogs_dir = CXPILOT_ROOT.parent / "buildlogs"
    buildlogs_dir.mkdir(exist_ok=True)
    os.environ['BUILDLOGS'] = str(buildlogs_dir)

    if args.build:
        try:
            PLANE_BINARY.unlink(missing_ok=True)  # Path
        except Exception:
            pass
        util.build_SITL('bin/arduplane', clean=(not args.no_clean))
        if not PLANE_BINARY.exists():
            raise RuntimeError(f"Failed to build {PLANE_BINARY}")

    specs = list(STEPS.keys()) if not args.tests else args.tests

    failed_test_labels = []
    for spec in specs:
        parts = spec.split('.')
        if len(parts) < 2 or parts[0] != 'test':
            raise ValueError(f"Bad spec: {spec}")
        cls_name = parts[1]
        key = f"test.{cls_name}"
        tester_cls = STEPS.get(key)
        if tester_cls is None:
            raise ValueError(f"Unknown test class: {cls_name}")
        rest = parts[2:]
        frames_all = list(tester_cls.get_frames().keys())
        frame = rest[-1] if rest and rest[-1] in frames_all else None
        subtest = '.'.join(rest[:-1] if frame else rest) or None
        frames = [frame] if frame else frames_all
        for f in frames:
            util.run_cmd('/bin/rm -f logs/*.BIN logs/LASTLOG.TXT')
            ok, tester = _run_one(tester_cls, f, subtest)
            test_name = f"{cls_name}" + (f".{subtest}" if subtest else "")
            label = f"{test_name} on {f}"
            print(f">>>>>>> {'PASSED' if ok else 'FAILED'}: {label}.")
            if not ok:
                tester.check_logs(f"{test_name}.{f}")
                failed_test_labels.append(label)

    if failed_test_labels:
        N = len(failed_test_labels)
        if N == 1:
            prefix = "1 test failed:"
        else:
            prefix = f"{N} tests failed:"
        raise RuntimeError(
            f"{prefix}\n" + "\n".join(failed_test_labels) + "\n"
        )
    else:
        print("All tests passed successfully!")


if __name__ == "__main__":
    main()
