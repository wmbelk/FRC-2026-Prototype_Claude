from ntcore import NetworkTableInstance

class AHHHHHHHHH:
    def __init__(self):
        self._inst = NetworkTableInstance.getDefault()

        self._state_table = self._inst.getTable("CameraState")
        self._test = self._state_table.getBooleanTopic("Test").publish()



        self._test.set(False)