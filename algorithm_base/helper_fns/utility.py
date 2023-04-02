#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import yaml
import numpy as np


def rectifyDict(thisDict):
    """Recursively convert the output to json-dump compatible form
    """
    for key in thisDict:
        if isinstance(thisDict[key],dict):
            rectifyDict(thisDict[key]) # recursively call this function
        else:
            if isinstance(thisDict[key],list):
                thisDict[key] = np.array(thisDict[key])
            if isinstance(thisDict[key],float):
                continue
            elif isinstance(thisDict[key],np.ndarray):
                thisDict[key] = np.vstack(thisDict[key])
            else:
                print('Output: Im not sure what type this is')
                continue
                

def jsonDump(dictOfDict, dirPath, indent=2):
    """Safe json dump
    """
    with open(dirPath, 'w') as f:
        json.dump(dictOfDict, f, indent= indent)


def jsonLoad(json_path):
    """Safe json load
    """
    with open(json_path) as f:
        dictOfDict = json.load(f)

    return dictOfDict


def yamlLoad(yaml_path):
    with open(yaml_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)    
    return config


def stamp2time(stamp):
    sec_int     = stamp.sec
    nanosec_int = stamp.nanosec
    return sec_int + nanosec_int * 1.0E-9

def time2stamp(time_sec_float):
    """split the time represented as seconds (float) into seconds (int) and nanoseconds (int) """
    
    sec = int(time_sec_float)
    nanosec = int((time_sec_float-sec)*1E9 +0.5)

    return sec, nanosec