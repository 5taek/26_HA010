import unittest
from unittest.mock import Mock, patch

from google.api_core.exceptions import ServiceUnavailable
from google.auth.exceptions import RefreshError

import gps_location_publisher as publisher


class PublisherTests(unittest.TestCase):
    def test_expected_sdk_errors_do_not_stop_publishing(self):
        for error in (ServiceUnavailable('offline'), RefreshError('auth'), OSError('network')):
            ref = Mock()
            ref.set.side_effect = error
            self.assertFalse(publisher.publish_location(ref, {'latitude': 1, 'longitude': 2}))
            self.assertEqual(ref.set.call_args.kwargs['timeout'], 10)

    def test_outage_recovery_reads_fresh_fixes_and_resets_backoff(self):
        ref = Mock()
        ref.set.side_effect = [ServiceUnavailable('offline')] * 7 + [None, ServiceUnavailable('offline')]
        fixes = [{'latitude': i, 'longitude': 2} for i in range(9)]
        waits = []

        def sleep(delay):
            waits.append(delay)
            if len(waits) == 9:
                raise KeyboardInterrupt

        with patch.object(publisher, 'read_gpsd_location', side_effect=fixes), \
                patch.object(publisher.time, 'sleep', side_effect=sleep):
            with self.assertRaises(KeyboardInterrupt):
                publisher.run_publisher(ref, 'localhost', 2947, 2, 10)
        self.assertEqual(waits, [1, 2, 4, 8, 16, 30, 30, 10, 1])
        self.assertEqual([c.args[0]['latestLocation']['latitude'] for c in ref.set.call_args_list], list(range(9)))

    def test_missing_fix_does_not_publish_or_refresh_timestamp(self):
        ref = Mock()
        with patch.object(publisher, 'read_gpsd_location', return_value=None), \
                patch.object(publisher.time, 'sleep', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                publisher.run_publisher(ref, 'localhost', 2947, 2, 10)
        ref.set.assert_not_called()


if __name__ == '__main__':
    unittest.main()
