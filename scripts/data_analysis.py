"""
Used to check if the data is actually correct
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../algorithm_base')))

# sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from helper_fns.RelativeOrbitalMechanics import RelativeOrbitalMechanics


def load_csv(path):
    f = open(path, 'rb')
    data = np.loadtxt(f, delimiter=",")
    f.close()
    return data

def run(data_dir):
    
    agent_names = ["agent0", "agent1"]
    agent_ref= "agent0"
    
    fig_dir = data_dir + "figures/"
    data_dir = data_dir + "bsk_data_out/"

    if not os.path.exists(fig_dir):
        os.makedirs(fig_dir)

    agent2pos = {}
    agent2vel = {}
    agent2att = {}
    agent2time = {}
    for agent in agent_names:
        agent2pos[agent] = load_csv(data_dir + agent + '_pos.csv')
        agent2vel[agent] = load_csv(data_dir + agent + '_vel.csv')
        agent2att[agent] = load_csv(data_dir + agent + '_att.csv')
        agent2time[agent] = load_csv(data_dir + agent + '_timedata.csv')
        
        
    # convert to pos_L_traj.
    agent2posL = {}
    agent2velL = {}
    for agent in agent_names:
        
        pos_L_traj = []
        vel_L_traj = []
        for i in range(len(agent2pos[agent])):
                
            pos_l_l, vel_l_l = RelativeOrbitalMechanics.abs2rel_v2(
                # deputy (sc2)
                agent2pos[agent][i][0:3],
                agent2vel[agent][i][0:3],
                # reference (sc1)
                agent2pos[agent_ref][i][0:3],
                agent2vel[agent_ref][i][0:3],
            )
            pos_L_traj.append(pos_l_l)
            vel_L_traj.append(vel_l_l)

        agent2posL[agent] = np.array(pos_L_traj)
        agent2velL[agent] = np.array(vel_L_traj)


    plt.figure(num='Relative position in LVLH')
    for agent in agent2posL.keys():
        plt.plot(agent2time[agent], agent2posL[agent][:, 0], label="x")
        plt.plot(agent2time[agent], agent2posL[agent][:, 1], label="y")
        plt.plot(agent2time[agent], agent2posL[agent][:, 2], label="z")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("SC positions in LVLH")
    plt.savefig(fig_dir + "rel_pos.png")

    plt.figure(num='Relative velicities in LVLH')
    for agent in agent2velL.keys():
        plt.plot(agent2time[agent], agent2velL[agent][:, 0], label="x")
        plt.plot(agent2time[agent], agent2velL[agent][:, 1], label="y")
        plt.plot(agent2time[agent], agent2velL[agent][:, 2], label="z")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("SC velocities LVLH")
    plt.savefig(fig_dir + "rel_vel.png")

    plt.figure(num='Attitude MRP')
    for agent in agent2velL.keys():
        plt.plot(agent2time[agent], agent2att[agent][:, 0], label="x")
        plt.plot(agent2time[agent], agent2att[agent][:, 1], label="y")
        plt.plot(agent2time[agent], agent2att[agent][:, 2], label="z")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("MRP")
    plt.savefig(fig_dir + "mrp.png")

    # plt.show() # not available when this script run from docker
    
    plt.close("all")

if __name__ == "__main__":
    
    data_dir = '/home/ubuntu/example_ws/data/runs/default_run/'
    run(data_dir)

