from abc import ABC, abstractmethod

class PropagateStates(ABC):
 
    @abstractmethod
    def propagate_state(self, ti, tf, state_init, control):
        """Interface for propagating state
        - state_init:   state object
        - t_vec         time series. t_vec[0] is t_init
        """
        pass
 