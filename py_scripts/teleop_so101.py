from Environment import *
from Robot import *
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader

# Setup Isaacsim environment
env = ENV("robot/SO101/robot/robot.usd",
          [-0.2, 0, 0], # base translation
          [1, 0, 0, 0], # base orientation quat
          "LeRobot") # name

print(env.robot.joint_names)
print(env.robot.joint_limits)

# Setup so101
leader_config = SO101LeaderConfig(
                port="/dev/ttyACM0",
                id="my_lerobot_leader",
            )
leader = SO101Leader(leader_config)
leader.connect()

env.start()
while(1):
    action = leader.get_action()
    leader_action = []
    for k in action.keys():
        leader_action.append(action[k])
    leader_action[-1] = leader_action[-1] - 15 # Gripper offset. Need further investigation
    leader_action = (np.array(leader_action) / 180 * np.pi).reshape((1, -1))
    env.robot.set_pos(leader_action)
    env.update()
    pos = env.robot.get_pos()
    print(pos)
    print("Diff:", abs(pos - leader_action))
