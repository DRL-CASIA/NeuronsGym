import numpy as np

scenarios = {
    "level1_1":{
        "goals": [[5.89, 1.17, False], [5.78, 4.0, False], [4.71, 2.76, False], [2.74, 3.69, False], [7.35, 2.26, False]],
        "enemy": [7.36, 3.91, 3.578],
        "self_pose": [1.95, 1.56, 1.53],
    },
    "level1_2":{
        "goals": [[5.29, 1.92, False], [2.19, 0.76, False], [2.43, 3.38, False], [0.42, 2.84, False], [4.67, 3.96, False]],
        "enemy": [4.1, 0.4, 2.51],
        "self_pose": [7.31, 1.6, 2.18],
    },
    "level2_1":{
        "goals": [[4.04, 4.04, False], [0.83, 1.43, False], [3.31, 4.16, False], [7.17, 3.15, False], [5.61, 3.98, False]],
        "enemy": [1.34, 3.65, -0.69],
        "self_pose": [6.59, 1.0, 4.13],
    },
    "level2_2":{
        "goals": [[5.94, 3.11, False], [2.47, 3.27, False], [5.72, 0.45, False], [7.6, 0.5, False], [0.32, 1.24, False]],
        "enemy": [4.38, 2.74, -1.186],
        "self_pose": [2.89, 0.45, 2.33],
    },
    "level3_1":{
        "goals": [[2.73, 0.82, False], [0.51, 2.74, False], [7.6, 0.5, False], [4.04, 0.4, False], [7.6, 4.0, False]],
        "enemy": [1.70, 3.96, 0.56],
        "self_pose": [4.58, 1.63, -1.13],
    },
    "level3_2":{
        "goals": [[1.27, 1.45, False], [7.6, 0.5, False], [4.04, 4.04, False], [2.65, 3.38, False], [4.04, 0.4, False]],
        "enemy": [7.39, 3.42, 3.47],
        "self_pose": [4.52, 2.35, 3.95],
    }
}