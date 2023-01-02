from ep_base_node import EPRobotBase
from simple_agent import Agent

from config import scenarios
import numpy as np
import cv2
import sys, select, termios, tty
import time
import json
import copy
import os
import random
settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())

    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)#读取终端上的交互输入
    else:
        key = '0'

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveInit(method="simple", max_vel=1.0, scenario="level1-1"):
    rootPath = "robot_logs/"
    now_time = time.strftime('%Y%m%d %H%M',time.localtime(time.time()))
    day = now_time.split(" ")[0]
    second = now_time.split(" ")[1]

    prefix = os.path.join(rootPath+day, "{}/{}-{}".format(scenario, method, max_vel))

    img_path = os.path.join(prefix, second + "/img/")
    json_path = os.path.join(prefix, second + "/json/" ) 

    if not os.path.exists(img_path):
        os.makedirs(img_path)
    
    if not os.path.exists(json_path):
        os.makedirs(json_path)
    return img_path, json_path
    
def saveSARS(img_path, json_path, image, json_log, num):
    # filename = json_path + str(num) + ".json"
    filename = os.path.join(json_path, str(num) + ".json")
    with open(filename,"w", encoding='utf-8') as f: 
        f.write(json.dumps(json_log, ensure_ascii=False  ,indent=4)) 

    cv2.imwrite(os.path.join(img_path, str(num) + ".jpg"), image)
        
env = EPRobotBase()
max_vel = 0.5
method = "simple" 
scenario = "level1_1"

eval_agent = Agent()

while True:
    try:
        key = getKey()
        if key == 's':
            break
        time.sleep(0.03)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


# eval_agent = Agent()

num_step = 200

goals = scenarios[scenario]["goals"]

obs = env.reset(goals)
img_path, json_path = saveInit(method=method, max_vel=max_vel, scenario=scenario)

# shift_x = random.uniform(-0.5, 0.5)
# shift_y = random.uniform(-0.5, 0.5)

shift_x = 0.0
shift_y = 0.0

# init obs to json start
init_action = [0.0, 0.0, 0,0, False]
init_reward = 0.0
init_done = False
init_info = [[None], [0, 0, 0, 0]]
init_start_time = time.time()

pose = np.array([obs["vector"][0][0], obs["vector"][0][1], obs["vector"][0][2]], dtype='float64').tolist()
obs["vector"][0][0] += shift_x 
obs["vector"][0][1] += shift_y 

obs_action_log = copy.deepcopy(obs)
del (obs_action_log["color_image"])
obs_action_log.update({"laser": np.array(obs["laser"], dtype='float64').tolist()})
new_vector = obs_action_log["vector"]
for i in range(len(new_vector)):
    temp = new_vector[i]
    if temp != False:
        new_vector[i] = np.array(temp, dtype='float64').tolist()
    else:
        new_vector[i] = str(temp)
obs_action_log.update({"vector": new_vector})
obs_action_log.update({"pose": pose})
obs_action_log.update({"action": np.array(init_action, dtype='float64').tolist()})
obs_action_log.update({"reward": str(init_reward)})
obs_action_log.update({"done": str(init_done)})
obs_action_log.update({"time": str(init_start_time)})
init_info[1] = np.array(init_info[1], dtype='float64').tolist()
obs_action_log.update({"info": init_info})
image = obs["color_image"]

saveSARS(img_path, json_path, image, obs_action_log, 0)


done = False
info = None
step_num = 1

while not done:
    start_time = time.time()
    print("-----------------------------------------")
    print("step ", step_num)
    print("the last points' status ", obs["vector"][9][-1])
    action = [0.0, 0.0, 0.0, False]
    action = eval_agent.agent_control(obs, done, info)

    print("Action: ", action)
    obs, reward, done, info = env.step(action)
    # obs, reward, done, info = env.auto_control()
    # action = info[0]

    pose = np.array([obs["vector"][0][0], obs["vector"][0][1], obs["vector"][0][2]], dtype='float64').tolist()
    obs["vector"][0][0] += shift_x 
    obs["vector"][0][1] += shift_y 

    obs_action_log = copy.deepcopy(obs)
    del (obs_action_log["color_image"])
    obs_action_log.update({"laser": np.array(obs["laser"], dtype='float64').tolist()})
    new_vector = obs_action_log["vector"]
    for i in range(len(new_vector)):
        temp = new_vector[i]
        if temp != False:
            new_vector[i] = np.array(temp, dtype='float64').tolist()
        else:
            new_vector[i] = str(temp)
   
    obs_action_log.update({"vector": new_vector})
    obs_action_log.update({"pose": pose})
    obs_action_log.update({"action": np.array(action, dtype='float64').tolist()})
    obs_action_log.update({"reward": str(reward)})
    obs_action_log.update({"done": str(done)})
    obs_action_log.update({"time": str(start_time)})
    info[1] = np.array(info[1], dtype='float64').tolist()
    obs_action_log.update({"info": info})
    image = obs["color_image"]

    print("======= : ", obs["vector"][-3][1])
    saveSARS(img_path, json_path, image, obs_action_log, step_num)

    
    if(obs["vector"][5][-1]):
        print("goal1 achieved")
    if(obs["vector"][6][-1]):
        print("goal2 achieved")
    if(obs["vector"][7][-1]):
        print("goal3 achieved")
    if(obs["vector"][8][-1]):
        print("goal4 achieved")
    if(obs["vector"][9][-1]):
        print("goal5 achieved")
        break

    step_num += 1
    
env.close()

