import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ground_segment_check', ROOT/'scripts/check_air_ground_pick_demo.py')
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


class GroundSegmentCheckTests(unittest.TestCase):
    def events(self):
        events = [dict(state=s, ros_time=float(i)) for i, s in enumerate(check.EXPECTED_STATES[8:])]
        events[1]['ground_travel'] = .8
        events[-1].update(tcp_lift=.15, grasp_confirmed=True)
        return events

    def test_opt_in_ground_sequence_has_no_fabricated_flight(self):
        summary = check.status_sequence_summary(self.events(), 3., .1, ground_only=True)
        self.assertEqual(summary['takeoff_count'], 0)
        self.assertEqual(summary['landing_count'], 0)
        with self.assertRaises(check.DemoCheckError):
            check.status_sequence_summary(self.events(), 3., .1)

    def test_ground_retains_lift_and_confirmation_checks(self):
        for field, value in (('tcp_lift', .05), ('grasp_confirmed', False)):
            events = self.events()
            events[-1][field] = value
            with self.assertRaises(check.DemoCheckError):
                check.status_sequence_summary(events, 3., .1, ground_only=True)

    def test_physical_lift_still_requires_real_target_motion(self):
        with self.assertRaises(check.DemoCheckError):
            check.target_lift_summary(.0575, .06, .1)


if __name__ == '__main__': unittest.main()
