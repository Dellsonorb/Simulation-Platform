#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest
from tf2_msgs.msg import TFMessage


EXPECTED_PARENT = "ground/base_link"
EXPECTED_CHILD = "ground/lidar_2d_link"
EXPECTED_AUTHORITY = "/ground/robot_state_publisher"
FORBIDDEN_FRAMES = {
    "base_link", "lidar_2d_link",
    "ground/ground/base_link", "ground/ground/lidar_2d_link",
}


class TfPrefixContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("tf_prefix_contract_test", anonymous=True)

    def test_fixed_edge_has_one_prefixed_authority_and_no_dynamic_copy(self):
        lock = threading.Lock()
        static_records = []
        dynamic_records = []

        def capture(target):
            def callback(message):
                authority = (message._connection_header or {}).get(
                    "callerid", "")
                with lock:
                    for transform in message.transforms:
                        target.append((
                            transform.header.frame_id,
                            transform.child_frame_id,
                            authority,
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w,
                        ))
            return callback

        static_subscriber = rospy.Subscriber(
            "/tf_static", TFMessage, capture(static_records), queue_size=10)
        dynamic_subscriber = rospy.Subscriber(
            "/tf", TFMessage, capture(dynamic_records), queue_size=10)
        self.addCleanup(static_subscriber.unregister)
        self.addCleanup(dynamic_subscriber.unregister)

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if any(record[:2] == (EXPECTED_PARENT, EXPECTED_CHILD)
                       for record in static_records):
                    break
            time.sleep(0.05)
        time.sleep(0.5)

        with lock:
            static_snapshot = tuple(static_records)
            dynamic_snapshot = tuple(dynamic_records)
        target_records = tuple(
            record for record in static_snapshot
            if record[:2] == (EXPECTED_PARENT, EXPECTED_CHILD))
        self.assertTrue(target_records, static_snapshot)
        self.assertEqual(
            {(EXPECTED_PARENT, EXPECTED_CHILD)},
            {record[:2] for record in static_snapshot},
        )
        self.assertEqual(
            {EXPECTED_AUTHORITY}, {record[2] for record in target_records})
        for record in target_records:
            self.assertAlmostEqual(-0.30, record[3], places=9)
            self.assertAlmostEqual(0.0, record[4], places=9)
            self.assertAlmostEqual(0.25, record[5], places=9)
            self.assertEqual((0.0, 0.0, 0.0, 1.0), record[6:10])
        for record in static_snapshot + dynamic_snapshot:
            self.assertFalse(FORBIDDEN_FRAMES & set(record[:2]), record)
        self.assertFalse(any(
            record[:2] == (EXPECTED_PARENT, EXPECTED_CHILD)
            for record in dynamic_snapshot), dynamic_snapshot)


if __name__ == "__main__":
    rostest.rosrun(
        "bunker_sim_runtime", "tf_prefix_contract", TfPrefixContractTest)
