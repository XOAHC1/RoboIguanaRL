import math


class Test:
    def __init__(self):
        self.a = 0.116
        self.b = 0.09
        self.c = 0.172
        self.d = 0.2

        # =========================================================
        # Trajectory parameters
        # =========================================================
        self.dStep = 0.15
        self.gC = 0.04
        self.gP = 0.03
        self.h = 0.18

        self.spineRange = 20.0
        self.tailRange = 20.0

        # =========================================================
        # Convergence Parameters
        # =========================================================
        self.convergence = 0.1
        self.TimeStep = 0.01

        # =========================================================
        # CPG PARAMETERS
        # =========================================================
        self.initialPhases = [
            0.0,
            math.pi,
            math.pi,
            0.0,
            math.pi / 2.0,
            3.0 * math.pi / 2.0,
        ]

        self.initialAmplitudes = [2.740051, 2.740051, 2.740051, 2.740051, 2.0, 2.0]
        self.initialOrientationOffset = [0.2914568, -2.850136, -0.2914568, 2.850136]

        # Initial foot positions (translated from original project)
        # Format: (x, y, z)
        self.FootPositionFL = (0.075, -0.18, 0.25)
        self.FootPositionFR = (-0.075, -0.18, -0.25)
        self.FootPositionRL = (-0.075, -0.18, 0.25)
        self.FootPositionRR = (0.075, -0.18, -0.25)

        self.FootPositions = [self.FootPositionFL, self.FootPositionFR, self.FootPositionRL, self.FootPositionRR]


    def get_foot_position(self, phase, amplitude, orientation_offset):
        x = -self.dStep * (amplitude - 1.0) * math.cos(phase) * math.cos(orientation_offset)
        y = -self.h + (self.gC if math.sin(phase) > 0.0 else self.gP) * math.sin(phase)
        z = -self.dStep * (amplitude - 1.0) * math.cos(phase) * math.sin(orientation_offset)

        print(
            f"Foot Position - Phase: {phase}, Amplitude: {amplitude}, OrientationOffset: {orientation_offset}, Position: ({x}, {y}, {z})"
        )
        return x, y, z

    def inverse_foot_position(self, phase, position, tolerance=1e-6):
        x, y, z = position
        cos_phase = math.cos(phase)
        if abs(cos_phase) < 1e-9:
            raise ValueError("Cannot invert when cos(phase) is zero.")

        radius = math.hypot(x, z)
        amplitude = 1.0 + radius / (self.dStep * abs(cos_phase))
        orientation_offset = math.atan2(z, x)

        if cos_phase > 0.0:
            orientation_offset -= math.pi

        orientation_offset = (orientation_offset + math.pi) % (2.0 * math.pi) - math.pi

        expected_y = -self.h + (self.gC if math.sin(phase) > 0.0 else self.gP) * math.sin(phase)
        if abs(y - expected_y) > tolerance:
            raise ValueError(
                f"Given y={y} does not match expected foot height {expected_y} for phase={phase}."
            )

        return amplitude, orientation_offset

    


T = Test()

for i in range(4):
    # T.get_foot_position(T.initialPhases[i], T.initialAmplitudes[i], T.initialOrientationOffset[i])

    print(f"{T.inverse_foot_position(T.initialPhases[i], T.FootPositions[i])}")
