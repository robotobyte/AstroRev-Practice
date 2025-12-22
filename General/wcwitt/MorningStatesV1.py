# Skeleton code for sample state machine...
# -----------------------------------------

import sys
from enum import Enum

# Define state symbols...
class State(Enum):
    SLEEPING            = 0
    SNOOZING            = 1
    AWAKE_IN_BED        = 2
    GETTING_UP          = 3
    SHOWERING           = 4
    GETTING_DRESSED     = 5
    EATING_BREAKFAST    = 6
    TRAVELING_TO_SCHOOL = 7

# Initialize state variable...
currentState = State.SLEEPING

# Initialize all input/event variables...
alarm = True
sleepy = False
runningLate = False
stinky = False
hungry = True

while True :

    # Read sensors to update alarm, sleepy, etc...
    # alarm = ...
    # sleepy = ...
    # ...

    # Let next state default to current state...
    nextState = currentState

    # Perform current-to-next state transitions...
    match currentState :
        case State.SLEEPING :
            # do sleeping stuff (snoring, dreaming)...
            if alarm :
                nextState = State.AWAKE_IN_BED
        case State.SNOOZING :
            if alarm :
                nextState = State.AWAKE_IN_BED
        case State.AWAKE_IN_BED :
            if sleepy and not runningLate :
                # hit snooze button...
                nextState = State.SNOOZING
            else :
                nextState = State.GETTING_UP
        case State.GETTING_UP :
            if stinky :
                nextState = State.SHOWERING
            else :
                nextState = State.GETTING_DRESSED
        case State.SHOWERING :
            # do showering stuff...
            nextState = State.GETTING_DRESSED
        case State.GETTING_DRESSED :
            # do getting dressed stuff...
            if hungry :
                nextState = State.EATING_BREAKFAST
            else :
                nextState = State.TRAVELING_TO_SCHOOL
        case State.EATING_BREAKFAST :
            # do breakfast eating stuff...
            nextState = State.TRAVELING_TO_SCHOOL
        case State.TRAVELING_TO_SCHOOL :
            # do traveling to school stuff...
            sys.exit()  # dummy line; really needs if to manage next state transition

    print ( f"{currentState.name} -> {nextState.name}" )

    # Update state for next iteration...
    currentState = nextState