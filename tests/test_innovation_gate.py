import math
import unittest

import numpy as np

from controls import Coord, EKFEstimator, ReaderState, RFIDConfig, RFIDMeasurement, TagState


class InnovationGateTests(unittest.TestCase):
    def test_rejection_retains_prediction_mean_and_covariance(self):
        estimator = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(4, 0), 1, 0),
                                 innovationGateMode="reject")
        estimator.predictState(0.1)
        state, covariance = estimator.state.copy(), estimator.covariance.copy()
        estimator.update(RFIDMeasurement(-55, True, 2, math.pi), ReaderState(), 0.1, predict=False)
        np.testing.assert_array_equal(estimator.state, state)
        np.testing.assert_array_equal(estimator.covariance, covariance)
        diagnostics = estimator.diagnostics
        self.assertFalse(diagnostics.measurementAccepted)
        self.assertTrue(diagnostics.innovationOutlier)
        self.assertEqual(diagnostics.measurementRejectionReason, "innovation_gate")
        self.assertGreater(diagnostics.normalizedInnovationSquared, diagnostics.innovationGateThreshold)

    def test_monitor_and_off_have_identical_updates(self):
        estimators = [EKFEstimator(RFIDConfig(), initialState=TagState(Coord(4, 0)),
                                  innovationGateMode=mode) for mode in ("monitor", "off")]
        for estimator in estimators:
            estimator.update(RFIDMeasurement(-55, True, 2, math.pi), ReaderState(), 0.1, predict=False)
        np.testing.assert_array_equal(estimators[0].state, estimators[1].state)
        np.testing.assert_array_equal(estimators[0].covariance, estimators[1].covariance)
        self.assertTrue(estimators[0].diagnostics.innovationOutlier)
        self.assertTrue(estimators[0].diagnostics.measurementAccepted)
        self.assertIsNone(estimators[1].diagnostics.innovationOutlier)

    def test_nis_matches_direct_calculation_and_wraps_bearing(self):
        estimator = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(-5, 0.05)),
                                 innovationGateMode="reject")
        measurement = RFIDMeasurement(-40 - 20 * math.log10(5), True, 2, -math.pi + 0.01)
        expected, jacobian = estimator._measurementModel(estimator.state, ReaderState())
        difference = measurement.bearing - expected[1]
        innovation = np.array([measurement.Pr - expected[0], math.atan2(math.sin(difference), math.cos(difference))])
        covariance = jacobian @ estimator.covariance @ jacobian.T + np.diag([4, 0.01])
        nis = float(innovation @ np.linalg.solve(covariance, innovation))
        estimator.update(measurement, ReaderState(), 0.1, predict=False)
        self.assertAlmostEqual(estimator.diagnostics.normalizedInnovationSquared, nis)
        self.assertTrue(estimator.diagnostics.measurementAccepted)
        self.assertFalse(estimator.diagnostics.innovationOutlier)

    def test_diagnostics_reset_on_missing_or_invalid_measurements(self):
        estimator = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(4, 0)), innovationGateMode="reject")
        estimator.update(RFIDMeasurement(-55, True, 2, math.pi), ReaderState(), 0.1)
        estimator.update(None, ReaderState(), 0.1)
        self.assertIsNone(estimator.diagnostics.measurementAccepted)
        self.assertIsNone(estimator.diagnostics.innovationOutlier)
        self.assertTrue(math.isnan(estimator.diagnostics.normalizedInnovationSquared))
        estimator.update(RFIDMeasurement(), ReaderState(), 0.1)
        self.assertIsNone(estimator.diagnostics.measurementAccepted)

    def test_bootstrap_and_singular_geometry_are_distinguished(self):
        estimator = EKFEstimator(RFIDConfig(), innovationGateMode="reject")
        estimator.update(RFIDMeasurement(-50, True, 2, 0), ReaderState(), 0.1)
        self.assertTrue(estimator.diagnostics.measurementAccepted)
        self.assertTrue(math.isnan(estimator.diagnostics.normalizedInnovationSquared))
        estimator.initializeState(TagState())
        estimator.update(RFIDMeasurement(-40, True, 2, 0), ReaderState(), 0.1, predict=False)
        self.assertFalse(estimator.diagnostics.measurementAccepted)
        self.assertEqual(estimator.diagnostics.measurementRejectionReason, "singular_geometry")

    def test_invalid_configuration(self):
        for kwargs in ({"innovationGateMode": "wrong"}, {"innovationGateThreshold": 0},
                       {"innovationGateThreshold": math.inf}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                EKFEstimator(RFIDConfig(), **kwargs)


if __name__ == "__main__":
    unittest.main()
