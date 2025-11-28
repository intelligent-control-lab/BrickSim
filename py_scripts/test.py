from Environment import *
from Robot import *

env = ENV("robot/SO101/robot/robot.usd",
          [-0.2, 0, 0], # base translation
          [1, 0, 0, 0], # base orientation quat
          "LeRobot") # name

print(env.robot.joint_names)
print(env.robot.joint_limits)

ts = time.time()
env.start()
while(time.time() - ts < 3):
    env.update()
    pos = env.robot.get_pos()
    print(pos)

target = np.copy(pos) #* 0
steps = np.linspace(0, 0.2, 1001)

ts = time.time()
i = 0
while(1):
    pos = env.robot.get_pos()
    
    target[0, 1] = steps[i]
    target[0, 2] = steps[i]
    env.robot.set_pos(target)
    print(pos)
    print("Diff:", abs(pos - target))
    env.update()
    i += 1
    i = min(i, 1000)

    