from scipy.integrate import solve_ivp

class PropagationUtil:
    """Propagation Utitlity Functions
    """

    @staticmethod
    def propagateOnce(fun, dt, state, control):
        """Implements numerical propagation of general EOM.

        Equation of Motion (EOM) is given by function f where
            xdot = fun(t, x)
        where 
          t = time
          x = state

        dt is the duration of the propagation
        """
        solutionOrbit = solve_ivp(fun,[0, dt],state)
        return solutionOrbit.y[:,-1]
    
    @staticmethod
    def propagate(fun, t_vec, state):

        state_series = [state] 
        num_time = len(t_vec)
        for i in range(num_time-1):
            tbegin = t_vec[i]
            tend   = t_vec[i+1]
            sol = solve_ivp(fun,[tbegin, tend],state)
            state = sol.y[:,-1]
            state_series.append(state)

        return state_series
                
