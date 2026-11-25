"""Extended Kalman filter for constant-velocity motion and RFID RSSI/bearing."""

from copy import deepcopy
from math import atan2, cos, hypot, log, log10, sin

import numpy as np

from .Estimator import Estimator
from .EstimatorDiagnostics import EstimatorDiagnostics
from .RFIDSensor import RFIDConfig
from .Utils import Coord, RFIDMeasurement, ReaderState, TagState, positive, wrapAngle


class EKFEstimator(Estimator[TagState, RFIDMeasurement, ReaderState]):
    DEFAULT_INNOVATION_THRESHOLD = -2 * log(0.01)  # Nominal chi-square(2), 99%.

    def __init__(
        self, config: RFIDConfig, accelerationStdDev: float = 0.5,
        initialState: TagState | None = None, initialCovariance=None,
        innovationGateThreshold: float = DEFAULT_INNOVATION_THRESHOLD,
        innovationGateMode: str = "monitor",
    ):
        positive(accelerationStdDev, "accelerationStdDev", allowZero=True)
        positive(innovationGateThreshold, "innovationGateThreshold")
        if innovationGateMode not in ("monitor", "reject", "off"):
            raise ValueError("innovationGateMode must be monitor, reject, or off")
        self.innovationGateThreshold = innovationGateThreshold
        self.innovationGateMode = innovationGateMode
        self._resetInnovation()
        self.config = deepcopy(config)
        self.accelerationStdDev = accelerationStdDev
        self.state: np.ndarray | None = None
        self.covariance: np.ndarray | None = None
        self._nextTime = 0.0
        if initialState is not None:
            self.initializeState(initialState, initialCovariance)
        elif initialCovariance is not None:
            raise ValueError("initialCovariance requires initialState")

    @property
    def initialized(self) -> bool:
        return self.state is not None

    @property
    def diagnostics(self) -> EstimatorDiagnostics:
        covariance = (float("nan"),) * 3 if not self.initialized else (
            float(self.covariance[0, 0]), float(self.covariance[0, 1]), float(self.covariance[1, 1]))
        return EstimatorDiagnostics(
            *covariance, effectiveSampleSize=float("nan"),
            innovationRSSI=self._innovationRSSI, innovationBearing=self._innovationBearing,
            normalizedInnovationSquared=self._nis,
            innovationGateThreshold=self.innovationGateThreshold if self.innovationGateMode != "off" else float("nan"),
            innovationGateMode=self.innovationGateMode, innovationOutlier=self._innovationOutlier,
            measurementAccepted=self._measurementAccepted, measurementRejectionReason=self._rejectionReason,
        )

    def _resetInnovation(self):
        self._innovationRSSI = self._innovationBearing = self._nis = float("nan")
        self._innovationOutlier = self._measurementAccepted = None
        self._rejectionReason = ""

    def initializeState(self, state: TagState, covariance=None) -> None:
        vector = np.array([state.position.x, state.position.y, state.xdot, state.ydot], dtype=float)
        matrix = np.eye(4) if covariance is None else np.array(covariance, dtype=float, copy=True)
        if not np.all(np.isfinite(vector)):
            raise ValueError("Initial state must be finite")
        if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
                or not np.allclose(matrix, matrix.T, rtol=0, atol=1e-12)):
            raise ValueError("Initial covariance must be a finite symmetric 4x4 matrix")
        eigenvalues, eigenvectors = np.linalg.eigh(matrix)
        if eigenvalues.min() < -1e-12:
            raise ValueError("Initial covariance must be positive semidefinite")
        if eigenvalues.min() < 0:
            matrix = (eigenvectors * np.maximum(eigenvalues, 0)) @ eigenvectors.T
        self.state, self.covariance = vector, matrix

    def _initializeFromMeasurement(self, measurement: RFIDMeasurement, reader: ReaderState) -> None:
        c = self.config
        exponent = (c.referenceRSSI - measurement.Pr) / (10 * c.pathLossExponent)
        if exponent <= 0:
            # Inside d0, RSSI is flat. Approximate a radial disk prior by its
            # mean and variance, rather than pretending range is known exactly.
            radius = min(c.referenceDistance, c.maxRange)
            distance, rangeVariance = 2 * radius / 3, radius ** 2 / 18
        else:
            distance = c.referenceDistance * 10 ** min(exponent, log10(c.maxRange / c.referenceDistance))
            rangeSigma = log(10) * distance * max(measurement.uncertainty, 1e-6) / (10 * c.pathLossExponent)
            rangeVariance = rangeSigma ** 2
        angle = reader.heading + measurement.bearing
        rotation = np.array([[cos(angle), -distance * sin(angle)],
                             [sin(angle), distance * cos(angle)]])
        covariance = np.eye(4)  # Initial velocity sigma = 1 m/s on each axis.
        covariance[:2, :2] = rotation @ np.diag([rangeVariance, max(c.bearingStdDev, 1e-6) ** 2]) @ rotation.T
        self.initializeState(TagState(Coord(reader.position.x + distance * cos(angle),
                                            reader.position.y + distance * sin(angle))), covariance)

    def predictState(self, dt: float) -> None:
        positive(dt, "dt")
        if not self.initialized:
            return
        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = dt
        acceleration = np.array([[0.5 * dt ** 2, 0], [0, 0.5 * dt ** 2], [dt, 0], [0, dt]])
        # Same independent, constant-over-the-tick acceleration model as the PF.
        processCovariance = self.accelerationStdDev ** 2 * acceleration @ acceleration.T
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + processCovariance
        self.covariance = (self.covariance + self.covariance.T) / 2

    def _measurementModel(self, state: np.ndarray, reader: ReaderState):
        dx, dy = state[0] - reader.position.x, state[1] - reader.position.y
        squaredRange = dx * dx + dy * dy
        distance = hypot(dx, dy)
        c = self.config
        expected = np.array([
            c.referenceRSSI - 10 * c.pathLossExponent * log10(max(distance, c.referenceDistance) / c.referenceDistance),
            wrapAngle(atan2(dy, dx) - reader.heading),
        ])
        jacobian = np.zeros((2, 4))
        if distance > c.referenceDistance:
            factor = -10 * c.pathLossExponent / (log(10) * squaredRange)
            jacobian[0, :2] = factor * np.array([dx, dy])
        if distance > 1e-8:
            jacobian[1, :2] = [-dy / squaredRange, dx / squaredRange]
        return expected, jacobian

    def _correct(self, measurement: RFIDMeasurement, reader: ReaderState) -> None:
        # Bearing is undefined at coincidence and the clamped RSSI has no local
        # range gradient there. Retain the prediction instead of dividing by 0.
        if hypot(self.state[0] - reader.position.x, self.state[1] - reader.position.y) <= 1e-8:
            self._measurementAccepted = False
            self._rejectionReason = "singular_geometry"
            return
        expected, jacobian = self._measurementModel(self.state, reader)
        innovation = np.array([measurement.Pr - expected[0], wrapAngle(measurement.bearing - expected[1])])
        noise = np.diag([max(measurement.uncertainty, 1e-6) ** 2,
                         max(self.config.bearingStdDev, 1e-6) ** 2])
        innovationCovariance = jacobian @ self.covariance @ jacobian.T + noise
        self._innovationRSSI, self._innovationBearing = map(float, innovation)
        self._nis = float(innovation @ np.linalg.solve(innovationCovariance, innovation))
        if self.innovationGateMode != "off":
            self._innovationOutlier = self._nis > self.innovationGateThreshold
        if self.innovationGateMode == "reject" and self._innovationOutlier:
            # Retain the predicted mean AND covariance: a rejected measurement
            # supplies no information and must not shrink the uncertainty.
            self._measurementAccepted = False
            self._rejectionReason = "innovation_gate"
            return
        self._measurementAccepted = True
        gain = np.linalg.solve(innovationCovariance, jacobian @ self.covariance).T
        self.state = self.state + gain @ innovation
        residual = np.eye(4) - gain @ jacobian
        # Joseph form preserves covariance positive semidefiniteness numerically.
        self.covariance = residual @ self.covariance @ residual.T + gain @ noise @ gain.T
        self.covariance = (self.covariance + self.covariance.T) / 2

    def calculateStateEstimate(self) -> TagState:
        if not self.initialized:
            raise RuntimeError("Estimator needs an initial prior or a valid RFID measurement")
        x, y, vx, vy = map(float, self.state)
        return TagState(Coord(x, y), vx, vy)

    def update(
        self, measurement: RFIDMeasurement | None, reader: ReaderState, dt: float,
        *, time: float | None = None, predict: bool = True,
    ) -> TagState | None:
        positive(dt, "dt")
        time = self._nextTime if time is None else time
        positive(time, "time", allowZero=True)
        self._resetInnovation()
        valid = measurement is not None and measurement.isValid
        if valid:
            if not np.all(np.isfinite([measurement.Pr, measurement.bearing, measurement.uncertainty])) or measurement.uncertainty < 0:
                raise ValueError("Valid measurements require finite RSSI/bearing and nonnegative uncertainty")
        if not self.initialized:
            if valid:
                self._initializeFromMeasurement(measurement, reader)
                self._measurementAccepted = True  # Bootstrap has no prior to test.
        else:
            if predict:
                self.predictState(dt)
            if valid:
                self._correct(measurement, reader)
        self._nextTime = time + dt
        return self.calculateStateEstimate() if self.initialized else None
