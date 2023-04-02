
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities.simulationProgessBar import SimulationProgressBar
from Basilisk.utilities import macros

# Custom implementation of the SimulationBaseClass
# By splitting ExecuteSimulation into three components, we isolate a single step of Basilisk simulation
class IncrementalSimulationClass(SimulationBaseClass.SimBaseClass):


    def __init__(self) -> None:
        super().__init__()
    
    # Split ExecuteSimulation into three components
    # - ExecuteSimulation_pre
    # - ExecuteSimulation_propagate
    # - ExecuteSimulation_post
    def ExecuteSimulation_pre(self):

        self.initializeEventChecks()

        self.nextStopTime = self.TotalSim.NextTaskTime
        self.nextPriority = -1
        self.pyProcPresent = False
        if len(self.pyProcList) > 0:
            self.nextPriority = self.pyProcList[0].pyProcPriority
            self.pyProcPresent = True
            self.nextStopTime = self.pyProcList[0].nextCallTime()
        self.progressBar = SimulationProgressBar(self.StopTime, self.showProgressBar)
        
    def ExecuteSimulation_propagate(self, stop_time_sec):

        stop_time_ns = macros.sec2nano(stop_time_sec)
        
        assert(stop_time_ns <= self.StopTime)

        while self.TotalSim.NextTaskTime <= stop_time_ns:
            if self.TotalSim.CurrentNanos >= self.nextEventTime >= 0:
                self.nextEventTime = self.checkEvents()
                self.nextEventTime = self.nextEventTime if self.nextEventTime >= self.TotalSim.NextTaskTime else self.TotalSim.NextTaskTime
            if 0 <= self.nextEventTime < self.nextStopTime:
                self.nextStopTime = self.nextEventTime
                self.nextPriority = -1
            self.TotalSim.StepUntilStop(self.nextStopTime, self.nextPriority)
            self.progressBar.update(self.TotalSim.NextTaskTime)
            self.nextPriority = -1
            self.nextStopTime = self.StopTime
            nextLogTime = self.RecordLogVars()
            procStopTimes = []
            for pyProc in self.pyProcList:
                nextCallTime = pyProc.nextCallTime()
                if nextCallTime <= self.TotalSim.CurrentNanos:
                    pyProc.executeTaskList(self.TotalSim.CurrentNanos)
                nextCallTime = pyProc.nextCallTime()
                procStopTimes.append(nextCallTime)

            if self.pyProcPresent and self.nextStopTime >= min(procStopTimes):
                self.nextStopTime = min(procStopTimes)
                self.nextPriority = self.pyProcList[procStopTimes.index(self.nextStopTime)].pyProcPriority
            if 0 <= nextLogTime < self.nextStopTime:
                self.nextStopTime = nextLogTime
                self.nextPriority = -1
            self.nextStopTime = self.nextStopTime if self.nextStopTime >= self.TotalSim.NextTaskTime else self.TotalSim.NextTaskTime

    def ExecuteSimulation_post(self):
        self.progressBar.markComplete()
        self.progressBar.close()
