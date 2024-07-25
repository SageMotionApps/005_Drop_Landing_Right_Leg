import numpy as np
import math


class Quaternion:
    def __init__(self, w=None, x=None, y=None, z=None):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def updateFromRawData(self, data=None):
        self.w = data["Quat1"]
        self.x = data["Quat2"]
        self.y = data["Quat3"]
        self.z = data["Quat4"]

    def __str__(self):
        return f"Quaternion({self.w}, {self.x}, {self.y}, {self.z})"

    def calculateTrunkSwayAngle(self):
        """
        This will calculate the Rotation Z->Y->Z for a node that that is switch pointing up.
        Note: This is about the z-axis (axis out of IMU board)
        """
        # case zyz, Rotation sequence: Z->Y->Z
        t0 = 2 * (self.y * self.z + self.w * self.x)
        t1 = -2 * (self.x * self.z - self.w * self.y)

        sway = np.arctan2(t0, t1)  # in radians
        trunk_sway = (
            sway * 180 / 3.14159 - 90
        )  # in deg and adjusted for attaching to the back
        trunk_sway = -trunk_sway  # adjust for sign convention

        return trunk_sway

    def calculateTrunkFlexionAngle(self):
        """
        This will calculate the Rotation Z->Y->X for a node that that is switch pointing up.
        Note: This is about the x-axis (axis along short edge of IMU board)
        """

        # Calculate the Trunk Forward Angle, about the x-axis (axis along short edge of IMU board)

        # case zyx, Rotation sequence: Z->Y->X, res0 about x-axis
        r31 = 2 * (self.y * self.z + self.w * self.x)
        r32 = self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z
        res0 = np.arctan2(r31, r32)  # in radians
        res0 = (
            res0 * 180 / 3.14159 - 90
        )  # in deg and adjusted for alignment on the back
        trunk_flexion = -res0  # adjust for sign convention

        return trunk_flexion

    def calculate_FrontAngle(self, leftORright="Right"):
        # Calculate the Shank or Thigh Forward Angle, about the z-axis (axis out of IMU board)

        # case zyz, Rotation sequence: Z->Y->Z, res2 about z-axis (1st rotation)
        r31 = 2 * (self.y * self.z + self.w * self.x)
        r32 = -2 * (self.x * self.z - self.w * self.y)
        res2 = np.arctan2(r31, r32)  # in radians
        res2 = (
            res2 * 180 / 3.14159 - 90
        )  # in deg and adjusted for attaching to the back
        Angle = -res2  # adjust for sign convention

        if leftORright == "left":
            Angle = -Angle

        return Angle

    def calculate_SideAngle(self, leftORright="Right"):
        # Calculate the Shank or Thigh Side Angle, about the x-axis (axis along short edge of IMU board)

        # case zyx, Rotation sequence: Z->Y->X, res0 about x-axis
        r31 = 2 * (self.y * self.z + self.w * self.x)
        r32 = self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z
        res0 = np.arctan2(r31, r32)  # in radians
        res0 = (
            res0 * 180 / 3.14159 - 90
        )  # in deg and adjusted for alignment on the back
        Angle = -res0  # adjust for sign convention

        if leftORright == "left":
            Angle = +Angle

        return Angle
