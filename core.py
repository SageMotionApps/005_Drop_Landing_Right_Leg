from sage.base_app import BaseApp
import numpy as np

if __name__ == "__main__":
    from Quaternion import Quaternion
    from LandPhaseDetector import LandPhaseDetector
else:
    from .Quaternion import Quaternion
    from .LandPhaseDetector import LandPhaseDetector

class Core(BaseApp):

    ###########################################################
    # INITIALIZE APP
    ###########################################################
    def __init__(self, my_sage):
        BaseApp.__init__(self, my_sage, __file__)

        # (m/s^2), accel threshhold on trunk sensor for initial contact
        landing_acc_threshold = 30

        # (sec), minimum duration of land phase in seconds
        min_landphase_dur = 0.100

        self.datarate = self.info["datarate"]
        self.min_threshold = float(self.config["min_threshold"])
        self.max_threshold = float(self.config["max_threshold"])
        self.feedback_delay = float(self.config["feedback_delay"])

        self.iteration = 0
        self.trunk_quat = Quaternion()
        self.thigh_quat = Quaternion()
        self.shank_quat = Quaternion()
        self.landphase = "pre_land"  # default land phase

        self.land_phase_detector = LandPhaseDetector(
            landing_acc_threshold, min_landphase_dur, self.datarate
        )

        self.NodeNum_shank = self.info["sensors"].index("shank")
        self.NodeNum_thigh = self.info["sensors"].index("thigh")
        self.NodeNum_trunk = self.info["sensors"].index("trunk")
        self.NodeNum_feedback_min = self.info["feedback"].index("min feedback")
        self.NodeNum_feedback_max = self.info["feedback"].index("max feedback")

        self.TSA_trial_start = 0
        self.TFA_trial_start = 0
        self.shank_FrontAngle_trial_start = 0
        self.thigh_FrontAngle_trial_start = 0

        self.KFA_initial_contact = 0
        self.KAA_initial_contact = 0
        self.TSA_initial_contact = 0
        self.TFA_initial_contact = 0

        self.feedback_min = 0
        self.feedback_max = 0

        self.alreadyGivenFeedback = False

    ###########################################################
    # CHECK NODE CONNECTIONS
    ###########################################################
    def check_status(self):
        sensors_count = self.get_sensors_count()
        feedback_count = self.get_feedback_count()
        # logging.debug("config pulse length {}".format(self.info["pulse_length"]))
        err_msg = ""
        if sensors_count < len(self.info["sensors"]):
            err_msg += "App requires {} sensors but only {} are connected".format(
                len(self.info["sensors"]), sensors_count
            )
        if self.config["feedback_enabled"] and feedback_count < len(
            self.info["feedback"]
        ):
            err_msg += "App require {} feedback but only {} are connected".format(
                len(self.info["feedback"]), feedback_count
            )
        if err_msg != "":
            return False, err_msg
        return True, "Now running DropLand app"

    #############################################################
    # UPON STARTING THE APP
    # If you have anything that needs to happen before the app starts
    # collecting data, you can uncomment the following lines
    # and add the code in there. This function will be called before the
    # run_in_loop() function below.
    #############################################################
    # def on_start_event(self, start_time):
    #     print("In On Start Event: {start_time}")

    ###########################################################
    # RUN APP IN LOOP
    ###########################################################
    def run_in_loop(self):
        # Get next data packet
        data = self.my_sage.get_next_data()

        # Get Quaternion Data
        self.trunk_quat.updateFromRawData(data=data[self.NodeNum_trunk])
        self.thigh_quat.updateFromRawData(data=data[self.NodeNum_thigh])
        self.shank_quat.updateFromRawData(data=data[self.NodeNum_shank])
        landphase_last = self.landphase

        # Calculate unadjusted angles
        TSA_unadjusted = self.trunk_quat.calculateTrunkSwayAngle()
        TFA_unadjusted = self.trunk_quat.calculateTrunkFlexionAngle()

        shank_FrontAngle_unadjusted = self.shank_quat.calculate_FrontAngle("right")
        thigh_FrontAngle_unadjusted = self.thigh_quat.calculate_FrontAngle("right")

        shank_SideAngle_unadjusted = self.shank_quat.calculate_SideAngle("right")
        thigh_SideAngle_unadjusted = self.thigh_quat.calculate_SideAngle("right")

        # Store initial angles from standing calibration on 1st iteration
        if self.iteration == 0:
            self.TSA_trial_start = TSA_unadjusted
            self.TFA_trial_start = TFA_unadjusted

            self.shank_FrontAngle_trial_start = shank_FrontAngle_unadjusted
            self.thigh_FrontAngle_trial_start = thigh_FrontAngle_unadjusted

            self.shank_SideAngle_trial_start = shank_SideAngle_unadjusted
            self.thigh_SideAngle_trial_start = thigh_SideAngle_unadjusted

        # Compute angles relative to standing calibration values
        TSA = TSA_unadjusted - self.TSA_trial_start
        TFA = TFA_unadjusted - self.TFA_trial_start

        shank_FrontAngle = (
            shank_FrontAngle_unadjusted - self.shank_FrontAngle_trial_start
        )
        thigh_FrontAngle = (
            thigh_FrontAngle_unadjusted - self.thigh_FrontAngle_trial_start
        )

        shank_SideAngle = shank_SideAngle_unadjusted - self.shank_SideAngle_trial_start
        thigh_SideAngle = thigh_SideAngle_unadjusted - self.thigh_SideAngle_trial_start

        # Calculate Knee Angles based human alignment of sensors on thigh and shank
        KFA = shank_FrontAngle - thigh_FrontAngle
        KAA = shank_SideAngle - thigh_SideAngle

        Accel_data = [data[self.NodeNum_trunk][f"Accel{axis}"] for axis in "XYZ"]
        AccelMag = np.linalg.norm(Accel_data)
        self.landphase = self.land_phase_detector.update(AccelMag, KFA)
        # At land, store initial contact values
        # RIGHT LEG
        if landphase_last == "pre_land" and self.landphase == "land":
            self.KFA_initial_contact = KFA
            self.KAA_initial_contact = KAA
            self.TSA_initial_contact = TSA  # Set TSA and TFA initial contact values based on right leg initial contact
            self.TFA_initial_contact = TFA

        # Turn feedback nodes on/off
        if self.config["feedback_enabled"] and not self.alreadyGivenFeedback:
            self.give_feedback()

        # Increment and save data
        self.iteration += 1
        time_now = self.iteration / self.datarate  # time in seconds

        my_data = {
            "time": [time_now],
            "landphase": [self.land_phase_detector.landphaseLookup()],
            # landphase and landphase_left are the same because they are both based on the trunk node acceleration
            "TSA": [TSA],
            "TFA": [TFA],
            "KFA": [KFA],
            "KAA": [KAA],
            "min_threshold": [self.min_threshold],
            "max_threshold": [self.max_threshold],
            "min_feedback_state": [self.feedback_min],
            "max_feedback_state": [self.feedback_max],
        }

        self.my_sage.save_data(data, my_data)
        self.my_sage.send_stream_data(data, my_data)
        return True

        #############################################################
        # UPON STOPPING THE APP
        # If you have anything that needs to happen after the app stops,
        # you can uncomment the following lines and add the code in there.
        # This function will be called after the data file is saved and
        # can be read back in for reporting purposes if needed.
        #############################################################
        # def on_stop_event(self, stop_time):
        #     print(f"In On Stop Event: {stop_time}")

    def give_feedback(self):
        lookup_table = {
            "Right Knee Flexion Angle": self.KFA_initial_contact,
            "Right Knee Adduction Angle": self.KAA_initial_contact,
            "Trunk Side Angle": self.TSA_initial_contact,
            "Trunk Front Angle": self.TFA_initial_contact,
        }
        # RIGHT LEG landing phases for right leg and trunk angles
        iterations_since_postlandphase_start = (
            self.land_phase_detector.iterations_since_postlandphase_start
        )
        time_since_postlandphase_start = (
            iterations_since_postlandphase_start / self.datarate
        )

        if (
            self.landphase == "post_land"
            and time_since_postlandphase_start >= self.feedback_delay
        ):

            # Select appropriate initial contact angle value
            angleVal = lookup_table[self.config["whichFeedbackMeasurement"]]

            # Determine if the threshold has been crossed and store the state.
            self.feedback_min = int(angleVal < self.min_threshold)
            self.feedback_max = int(angleVal > self.max_threshold)

            # Turn on/off feedback based on the state.
            self.toggle_feedback(
                self.NodeNum_feedback_min,
                self.info["pulse_length"],
                feedback_state=self.feedback_min,
            )
            self.toggle_feedback(
                self.NodeNum_feedback_max,
                self.info["pulse_length"],
                feedback_state=self.feedback_max,
            )

            # This flag is used in the run_in_loop() so that the user doesn't receive continous feedback,
            # only after performing the activity.
            self.alreadyGivenFeedback = True

    def toggle_feedback(self, feedbackNode=0, duration=1, feedback_state=False):
        if feedback_state:
            self.my_sage.feedback_on(feedbackNode, duration)
        else:
            self.my_sage.feedback_off(feedbackNode)
