import commands2

from subsystems.intake import IntakeSubsystem


class IntakeCommand(commands2.Command):
    def __init__(self, intake_subsystem: IntakeSubsystem):
        super().__init__()
        self._intake = intake_subsystem
        self.addRequirements(intake_subsystem)

    def initialize(self):
        self._intake.deploy()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._intake.undeploy()
