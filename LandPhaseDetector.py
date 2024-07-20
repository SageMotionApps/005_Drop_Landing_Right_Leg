import numpy as np
import math


class LandPhaseDetector:
    def __init__(self, landing_acc_threshold, min_landphase_dur, datarate):
        self.landing_acc_threshold = landing_acc_threshold
        self.min_landphase_dur = min_landphase_dur
        self.datarate = datarate
        self.landphase = "pre_land"
        self.iterations_since_landphase_start = 0
        self.iterations_since_postlandphase_start = 0
        self.KFA_arr = []

    def update(self, trunk_acc_magnitude, KFA):

        if self.landphase == "pre_land":

            if trunk_acc_magnitude > self.landing_acc_threshold:
                self.landphase = "land"

        elif self.landphase == "land":
            self.KFA_arr.append(KFA)
            self.iterations_since_landphase_start += 1
            time_since_landphase_start = (
                self.iterations_since_landphase_start / self.datarate
            )

            if (
                time_since_landphase_start > self.min_landphase_dur
                and self.KFA_arr[-1] < self.KFA_arr[-2]
            ):
                self.landphase = "post_land"
                self.iterations_since_landphase_start = 0
                self.KFA_arr = []

        elif self.landphase == "post_land":
            self.iterations_since_postlandphase_start += 1

        return self.landphase

    def landphaseLookup(self):

        LANDPHASE_LOOKUP_TABLE = {
            "pre_land": 0,
            "land": 1,
            "post_land": 2,
        }
        return LANDPHASE_LOOKUP_TABLE[self.landphase]
