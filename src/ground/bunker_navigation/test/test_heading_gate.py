#!/usr/bin/env python
from __future__ import division

import math
import unittest

from bunker_navigation.heading_gate import GoalLifecycle, HeadingGate


class HeadingGateTest(unittest.TestCase):
    def gate(self):
        return HeadingGate(math.radians(100), math.radians(30), 0.5,
                           1.5, 0.25, 0.8, 15.0, 0.5)

    def test_distant_rear_goal_aligns_in_shortest_direction(self):
        gate = self.gate()
        gate.set_goal((-1.0, 0.2), (0.0, 0.0, 0.0), 1.0)
        command = gate.command((0.0, 0.0, 0.0), (-0.2, 0.1), 1.1, 1.1)
        self.assertEqual(gate.state, HeadingGate.ALIGNING)
        self.assertEqual(command[0], 0.0)
        self.assertGreater(command[1], 0.0)
        self.assertLessEqual(command[1], 0.8)

    def test_release_below_30_degrees_passes_raw_reverse(self):
        gate = self.gate()
        gate.set_goal((-1.0, 0.2), (0.0, 0.0, 0.0), 1.0)
        command = gate.command((0.0, 0.0, math.radians(160)),
                               (-0.2, -0.1), 2.0, 2.0)
        self.assertEqual(gate.state, HeadingGate.PASS_THROUGH)
        self.assertEqual(command, (-0.2, -0.1))

    def test_release_is_latched_across_noisy_30_degree_boundary(self):
        gate = self.gate()
        gate.set_goal((-1.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        states = []
        for index, yaw_degrees in enumerate((149.8, 150.2, 149.9, 150.1)):
            gate.command((0.0, 0.0, math.radians(yaw_degrees)),
                         (-0.12, 0.07), 2.0 + index * 0.1,
                         2.0 + index * 0.1)
            states.append(gate.state)
        self.assertEqual(states[0], HeadingGate.ALIGNING)
        self.assertEqual(states[1:], [HeadingGate.PASS_THROUGH] * 3)

    def test_near_rear_goal_bypasses_and_retains_reverse(self):
        gate = self.gate()
        gate.set_goal((-0.4, 0.0), (0.0, 0.0, 0.0), 1.0)
        self.assertEqual(gate.state, HeadingGate.PASS_THROUGH)
        self.assertEqual(gate.command((0.0, 0.0, 0.0), (-0.15, 0.2),
                                      1.1, 1.1), (-0.15, 0.2))

    def test_stale_raw_command_and_timeout_stop(self):
        gate = self.gate()
        gate.set_goal((-1.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        self.assertEqual(gate.command((0.0, 0.0, 0.0), (-0.2, 0.0),
                                      2.0, 1.0), (0.0, 0.0))
        self.assertEqual(gate.command((0.0, 0.0, 0.0), (-0.2, 0.0),
                                      16.1, 16.1), (0.0, 0.0))
        self.assertEqual(gate.state, HeadingGate.FAILED)

    def test_blocked_is_latched_zero_until_goal_lifecycle_reset(self):
        gate = self.gate()
        gate.set_goal((-1.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        gate.block()
        self.assertEqual(gate.state, HeadingGate.BLOCKED)
        self.assertEqual(gate.command((0.0, 0.0, 0.0), (-0.2, 0.8),
                                      20.0, 20.0), (0.0, 0.0))
        self.assertEqual(gate.state, HeadingGate.BLOCKED)
        gate.set_goal((1.0, 0.0), (0.0, 0.0, 0.0), 21.0)
        self.assertEqual(gate.state, HeadingGate.PASS_THROUGH)

    def test_clear_returns_idle_zero(self):
        gate = self.gate()
        gate.set_goal((-1.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        gate.clear()
        self.assertEqual(gate.state, HeadingGate.IDLE)
        self.assertEqual(gate.command((0, 0, 0), (0.2, 0), 1.1, 1.1),
                         (0.0, 0.0))


class GoalLifecycleTest(unittest.TestCase):
    def test_new_goal_atomically_replaces_active_goal(self):
        lifecycle = GoalLifecycle()
        lifecycle.start('old', 10.0)
        lifecycle.start('new', 20.0)
        self.assertEqual(lifecycle.active_goal_id, 'new')
        self.assertEqual(lifecycle.active_goal_stamp, 20.0)

    def test_cancel_all_or_matching_goal_clears_but_unrelated_cancel_does_not(self):
        lifecycle = GoalLifecycle()
        lifecycle.start('active', 10.0)
        self.assertFalse(lifecycle.cancel('other', 0.0))
        self.assertEqual(lifecycle.active_goal_id, 'active')
        self.assertTrue(lifecycle.cancel('', 0.0))
        self.assertIsNone(lifecycle.active_goal_id)

    def test_cancel_before_stamp_obeys_actionlib_semantics(self):
        lifecycle = GoalLifecycle()
        lifecycle.start('active', 10.0)
        self.assertFalse(lifecycle.cancel('', 9.0))
        self.assertTrue(lifecycle.cancel('', 10.0))

    def test_stale_preempt_result_cannot_clear_replacement_goal(self):
        lifecycle = GoalLifecycle()
        lifecycle.start('old', 10.0)
        lifecycle.start('new', 20.0)
        self.assertFalse(lifecycle.finish('old'))
        self.assertEqual(lifecycle.active_goal_id, 'new')

    def test_current_preempt_abort_or_success_result_clears(self):
        for terminal_status in ('PREEMPTED', 'ABORTED', 'SUCCEEDED'):
            lifecycle = GoalLifecycle()
            lifecycle.start('active', 10.0)
            self.assertTrue(lifecycle.finish('active'), terminal_status)
            self.assertIsNone(lifecycle.active_goal_id)


if __name__ == '__main__':
    unittest.main()
