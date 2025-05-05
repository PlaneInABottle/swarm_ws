from enum import Enum, auto
class InternalState(Enum):
    """ Defines the internal operational state of the CoreDroneController """
    INIT = auto()         # Initial state before configuration
    IDLE_ON_GROUND = auto()  # Configured, ready, but not armed/flying
    ARMING = auto()          # Arm command sent, waiting for confirmation
    IDLE_HOVERING = auto()   # Armed, flying, holding position, no action active
    EXECUTING_ACTION = auto()# An action goal is being actively processed
    # Consider adding: LANDING_AUTO, ERROR
