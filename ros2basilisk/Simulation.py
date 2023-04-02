"""Basilisk Simulation Node with reaction wheels."""

# python imports
import numpy as np
import matplotlib.pyplot as plt
import os

# basilisk imports
from Basilisk.architecture import messaging
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import vizSupport
from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import spacecraft
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk import __path__

# relative imports
from IncrementalSimulationClass import IncrementalSimulationClass
from RosBskTask import RosBskTask

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


class Simulation():
    
    def __init__(self, bsk_config, dir_out, timeStep, simTime) -> None:

        self.timeStep = timeStep
        self.simTime = simTime
        self.bskSim, self.bsk_task = self.create_sim_class(bsk_config)
        self.num_sc_zfill = 4 # assume num_sc <= 9999
        self.dir_out = dir_out
        
        # create dir if not exist
        if not os.path.exists(self.dir_out):
            os.makedirs(self.dir_out)
            
        self.bskSim.ExecuteSimulation_pre()

        pass

    def create_sim_class(self, bsk_config):
        """Create Simulation Base Class
        """

        bskSim = IncrementalSimulationClass()

        sc_configs = bsk_config["sc_configs"]
        agent_list = sorted(sc_configs.keys())
        
        num_sc = len(agent_list)

        # --- Dynamics --- 
        dynProcessName = "dynProcess"
        dynTaskName = "dynTask"
        dynProcess = bskSim.CreateNewProcess(dynProcessName)
        dynTimeStep = macros.sec2nano(self.timeStep)
        dynProcess.addTask(bskSim.CreateNewTask(dynTaskName, dynTimeStep))

        # --- Simulation gravity ---
        gravFactory = simIncludeGravBody.gravBodyFactory()
        gravBodies = gravFactory.createBodies(['earth'])
        gravBodies['earth'].isCentralBody = True
        mu = gravFactory.gravBodies['earth'].mu
        
        # --- ROS Interface ---
        pyTaskName = "pyTask"
        pyProcessName = "pyProcess"
        pyModulesProcess = bskSim.CreateNewPythonProcess(pyProcessName, -1)
        pyModulesProcess.createPythonTask(pyTaskName, dynTimeStep, True, 10)
        ros_bridge = RosBskTask("ros_interface", agent_list, True, 100)
        pyModulesProcess.addModelToTask(pyTaskName, ros_bridge)
        
        scObjects = {}
        self.log_sc = {}
        self.log_onTime = {}
        self.log_torque = {}
        agent2thrEffector ={}
        agent2rwsEffector = {}
        for agent_name in agent_list:
            
            # --- Spacecraft properties ---
            scObjects[agent_name] = spacecraft.Spacecraft()
            scObjects[agent_name].ModelTag = "scObject_" + agent_name
            scObjects[agent_name].hub.mHub = sc_configs[agent_name]["mHub"]
            scObjects[agent_name].hub.r_BcB_B = [[sc_configs[agent_name]["r_BcB_B"][0]],
                                                 [sc_configs[agent_name]["r_BcB_B"][1]],
                                                 [sc_configs[agent_name]["r_BcB_B"][2]]]
            scObjects[agent_name].hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(sc_configs[agent_name]['IHubPntBc_B'])
            
            bskSim.AddModelToTask(dynTaskName, scObjects[agent_name], None, 2)
            
            # --- Gravity ---
            scObjects[agent_name].gravField.gravBodies = spacecraft.GravBodyVector(
                list(gravFactory.gravBodies.values()))
            
            # --- Thrusters ---
            agent2thrEffector[agent_name] = thrusterDynamicEffector.ThrusterDynamicEffector()
            bskSim.AddModelToTask(dynTaskName, agent2thrEffector[agent_name], None, 3)
            
            thrusters_config = sc_configs[agent_name]["thrusters"] # list of thrusters
            thFactory = simIncludeThruster.thrusterFactory()
            for thruster in thrusters_config:
                thFactory.create(
                    thrusterType = thruster["thrusterType"],
                    r_B = thruster['r_B'],
                    tHat_B = thruster['tHat_B'],
                    useMinPulseTime=False, 
                    MinOnTime=0.0 )
            thFactory.addToSpacecraft(scObjects[agent_name].ModelTag, agent2thrEffector[agent_name], scObjects[agent_name])
                
            ###########################################################
            # Reaction Wheels
            # Reference: 
            #  - scendarioAttitudeFeedbackRW.py in Basilisk examples.
            ###########################################################
            
            # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
            rwFactory = simIncludeRW.rwFactory()
            varRWModel = messaging.BalancedWheels
            rws_config = sc_configs[agent_name]["reaction_wheels"] # list of thrusters
            for rw in rws_config:    
                rwFactory.create(rw['type'], rw['gsHat_B'], maxMomentum=rw['maxMomentum'], Omega=rw['Omega'], RWModel=varRWModel)
            
            # create RW object container and tie to spacecraft object
            agent2rwsEffector[agent_name] = reactionWheelStateEffector.ReactionWheelStateEffector()
            rwFactory.addToSpacecraft(scObjects[agent_name].ModelTag, agent2rwsEffector[agent_name], scObjects[agent_name])

            # add RW object array to the simulation process.  This is required for the UpdateState() method
            # to be called which logs the RW states
            bskSim.AddModelToTask(dynTaskName, agent2rwsEffector[agent_name], None, 3)
            
            # --- ROS Bridge ---
            ros_bridge.state_bsk_readers[agent_name].subscribeTo(scObjects[agent_name].scStateOutMsg)
            agent2thrEffector[agent_name].cmdsInMsg.subscribeTo(ros_bridge.thrust_msgs_bsk[agent_name])
            agent2rwsEffector[agent_name].rwMotorCmdInMsg.subscribeTo(ros_bridge.rw_torque_msgs_bsk[agent_name])
            
            # Initialize orbital parameters
            orbit_config =  sc_configs[agent_name]["orbit"]
            if orbit_config["initMethod"] == "orbital_elements":
                self.init_orbit_from_oe(scObjects[agent_name], orbit_config, mu)
            else:
                raise ValueError("please sepcify correct orbit.initMethod")
            scObjects[agent_name].hub.sigma_BNInit = [[sc_configs[agent_name]["sigma_BNInit"][0]],
                                                      [sc_configs[agent_name]["sigma_BNInit"][1]],
                                                      [sc_configs[agent_name]["sigma_BNInit"][2]]]
            scObjects[agent_name].hub.omega_BN_BInit = [[sc_configs[agent_name]["omega_BN_BInit"][0]],
                                                        [sc_configs[agent_name]["omega_BN_BInit"][1]],
                                                        [sc_configs[agent_name]["omega_BN_BInit"][2]]]
            
            # --- Log ----
            samplingTime = macros.sec2nano(self.timeStep)
            self.log_sc[agent_name] = scObjects[agent_name].scStateOutMsg.recorder(samplingTime)
            self.log_onTime[agent_name] = ros_bridge.thrust_msgs_bsk[agent_name].recorder(samplingTime)
            self.log_torque[agent_name] = ros_bridge.torque_msgs_bsk[agent_name].recorder(samplingTime)
            
            bskSim.AddModelToTask(dynTaskName, self.log_sc[agent_name])
            bskSim.AddModelToTask(dynTaskName, self.log_onTime[agent_name])
            bskSim.AddModelToTask(dynTaskName, self.log_torque[agent_name])

        # Turn on visualization and save the recorded data to a file.
        # Uncomment livestream=True to see the visualization in real time.
        viz = vizSupport.enableUnityVisualization(bskSim, dynTaskName, list(scObjects.values()),
                                                thrEffectorList=[ [v] for v in agent2thrEffector.values() ],
                                                saveFile=fileName,
                                                rwEffectorList=[ v for v in agent2rwsEffector.values() ],
                                                # liveStream=True
                                                )
        vizSupport.setActuatorGuiSetting(viz, viewThrusterPanel=True, viewThrusterHUD=True, viewRWPanel=True, viewRWHUD=True, spacecraftName="scObject2")

        # initialize simulation 
        simulationTime = macros.sec2nano(self.simTime)
        bskSim.InitializeSimulation()
        bskSim.ConfigureStopTime(simulationTime)

        # return the simulation object and the interface
        return bskSim, ros_bridge
    
    def init_orbit_from_oe(self, scObject, orbit_config, mu):
        
        oe = orbitalMotion.ClassicElements()
        oe.a = orbit_config["a"]
        oe.e = orbit_config["e"]
        oe.i = orbit_config["i"]
        oe.Omega = orbit_config["Omega"]
        oe.omega = orbit_config["omega"]
        mean_anom = orbit_config["M"]
        ecc_anom = orbitalMotion.M2E(mean_anom, oe.e)
        oe.f = orbitalMotion.E2f(ecc_anom, oe.e)
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        scObject.hub.r_CN_NInit = rN  # m
        scObject.hub.v_CN_NInit = vN  # m/s
        

    def run_one_step(self, stop_time_sec):
        # Runs the simulation for one step
        self.bskSim.ExecuteSimulation_propagate(stop_time_sec)

    def post_process(self):
        # After the simulation is done, this method is called to save and plot the data
        self.bskSim.ExecuteSimulation_post()

        dir_out = self.dir_out


        for agent_name in self.log_sc.keys():
            self.log_sc[agent_name].r_BN_N
            pos = self.log_sc[agent_name].r_BN_N
            vel = self.log_sc[agent_name].v_BN_N
            att = self.log_sc[agent_name].sigma_BN
            omega = self.log_sc[agent_name].omega_BN_B
            timeData = self.log_sc[agent_name].times() * macros.NANO2SEC
            onTime = self.log_onTime[agent_name].OnTimeRequest
            torque = self.log_torque[agent_name].torqueRequestBody
                
            # write to file.
            np.savetxt(f"{dir_out}/" + agent_name + "_pos.csv", pos, delimiter=',')
            np.savetxt(f"{dir_out}/" + agent_name + "_vel.csv", vel, delimiter=',')
            np.savetxt(f"{dir_out}/" + agent_name + "_att.csv", att, delimiter=',')
            np.savetxt(f"{dir_out}/" + agent_name + "_omega.csv", omega, delimiter=',')
            np.savetxt(f"{dir_out}/" + agent_name + "_timedata.csv", timeData, delimiter=',')
            np.savetxt(f"{dir_out}/" + agent_name + "_thrustOnTime.csv", onTime, delimiter=',')
            np.savetxt(f"{dir_out}/" + agent_name + "_torque.csv", torque, delimiter=',')


           # attitude of spacecraft two
            plt.figure(num="MRP "+agent_name)
            plt.plot(timeData, att[:, 0], label="x")
            plt.plot(timeData, att[:, 1], label="y")
            plt.plot(timeData, att[:, 2], label="z")
            plt.legend()
            plt.xlabel("time (s)")
            plt.ylabel("MRP")
        
        plt.show()
        return
