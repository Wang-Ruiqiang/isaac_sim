#!/usr/bin/env python3
import numpy as np
import pandas as pd
import time

def main(args=None):
    paths = []
    path = dict(
        observations=[],
        actions=[],
        rewards=[],
        next_observations=[],
        terminals=[],
        agent_infos=[],
        env_infos=[],
    )
    print("paths = ", paths)
    paths.append(path)
    for idx, path_dict in enumerate(paths):
        obs_dict = dict()
        next_obs_dict = dict()
        key = 0
        while key <20:
            obs_dict[key] = key
            next_obs_dict[key] = key+1
            key+=1
        path_dict['observations'].append(obs_dict)
        path_dict['rewards'].append(idx)
        path_dict['terminals'].append(idx)
        path_dict['actions'].append(idx)
        path_dict['next_observations'].append(next_obs_dict)
        path_dict['agent_infos'].append(idx)
        path_dict['env_infos'].append(idx)
        
    print("paths = ", paths)


if __name__ == "__main__":
    main()
