# Code adapted from https://github.com/intelligent-control-lab/StableLego

try:
    import gurobipy as gp
except ImportError:
    raise ImportError("Gurobi is not installed. Please install Gurobi to use the StableLego module.")
if gp.__version__ != "12.0.3":
    raise ImportError(f"Gurobi version 12.0.3 is required, but version {gp.__version__} is installed. Please install the correct version of Gurobi to use the StableLego module.")

import numpy as np
import time
from gurobipy import GRB
from bricksim.importers.legolization import load_default_lego_library

def _construct_world_grid(lego, world_dimension, brick_library):
    world_grid = np.zeros(world_dimension)
    for key in lego.keys():
        try:
            int(key)
        except:
            print(f"Warning: Non-integer key {key} found in lego structure. Skipping.")
            continue
        brick = lego[key]
        brick_id = str(brick["brick_id"])
        if(brick["ori"] == 0):
            h = brick_library[brick_id]["height"]
            w = brick_library[brick_id]["width"]
        else:
            w = brick_library[brick_id]["height"]
            h = brick_library[brick_id]["width"]
        brick_x = brick["x"]
        brick_y = brick["y"]
        brick_z = brick["z"] - 1
        for i in range(brick_x, brick_x + h):
            for j in range(brick_y, brick_y + w):
                world_grid[i, j, brick_z] = 1
    return world_grid
        
def _gen_key(x, y, z):
    return "X: " + str(x) + ", Y: " + str(y) + ", Z: " + str(z)

def _out_boundary(pt, brick_x, brick_y, h, w):
    x = pt[0]
    y = pt[1]
    if(x < brick_x or x >= brick_x + h or y < brick_y or y >= brick_y + w):
        return True
    return False

def run_stable_lego(
        lego_structure: dict,
        g: float = 9.8,
        T: float = 100,
        brick_unit_height: float = 0.0096,
        brick_unit_length: float = 0.0078,
        world_dim: tuple[int, int, int] = (64, 64, 64),
        alpha: float = 0.001,
        beta: float = 0.000001,
        brick_library: dict | None = None,
        print_log: bool = False,
):
    ############### Setup ###############
    if brick_library is None:
        brick_library = load_default_lego_library()
    g_ = g # N/kg
    T_ = T / 1000 * g_ # N
    brick_unit_height = brick_unit_height # mm
    brick_unit_length = brick_unit_length # mm
    for key in lego_structure.keys():
        try:
            int(key)
        except:
            print(f"Warning: Non-integer key {key} found in lego structure. Skipping.")
            continue
        lego_structure[key]["z"] += 1
    world_grid = _construct_world_grid(lego_structure, world_dim, brick_library) 
    n_bricks = len(lego_structure)
    t_start = time.time()
    
    ############### Setup Optimization ###############
    model = gp.Model("lego_stability_analysis")
    model.setParam("OutputFlag", print_log)
    model.Params.IterationLimit = 1000000
    model.setParam("MIPFocus", 1)
    big_num = 10 * n_bricks

    # Define variables
    force_sum_x_pos = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_x_pos")
    force_sum_x_neg = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_x_neg")
    force_sum_x = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_x")
    force_sum_y_pos = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_y_pos")
    force_sum_y_neg = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_y_neg")
    force_sum_y = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_y")
    force_sum_z_pos = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_z_pos")
    force_sum_z_neg = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_z_neg")
    force_sum_z = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="force_sum_z")
    torque_sum_1_pos = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="torque_sum_1_pos")
    torque_sum_1_neg = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="torque_sum_1_neg")
    torque_sum_1 = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="torque_sum_1")
    torque_sum_2_pos = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="torque_sum_2_pos")
    torque_sum_2_neg = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="torque_sum_2_neg")
    torque_sum_2 = model.addVars(n_bricks, lb=-big_num, vtype=gp.GRB.CONTINUOUS, name="torque_sum_2")
    force_abs_sum_x = model.addVars(n_bricks, vtype=gp.GRB.CONTINUOUS, name="force_abs_sum_x")
    force_abs_sum_y = model.addVars(n_bricks, vtype=gp.GRB.CONTINUOUS, name="force_abs_sum_y")
    force_abs_sum_z = model.addVars(n_bricks, vtype=gp.GRB.CONTINUOUS, name="force_abs_sum_z")
    torque_abs_sum_1 = model.addVars(n_bricks, vtype=gp.GRB.CONTINUOUS, name="torque_abs_sum_1")
    torque_abs_sum_2 = model.addVars(n_bricks, vtype=gp.GRB.CONTINUOUS, name="torque_abs_sum_2")
    brick_max_f_down = model.addVars(n_bricks, vtype=gp.GRB.CONTINUOUS, name="brick_max_f_down")

    force_dict = dict()
    sum_f_list = []
    for key in lego_structure.keys():
        try:
            int(key)
        except:
            print(f"Warning: Non-integer key {key} found in lego structure. Skipping.")
            continue
        brick = lego_structure[key]
        brick_id = str(brick["brick_id"])
        if(brick["ori"] == 0):
            h = brick_library[brick_id]["height"]
            w = brick_library[brick_id]["width"]
        else:
            w = brick_library[brick_id]["height"]
            h = brick_library[brick_id]["width"]
        brick_x = brick["x"]
        brick_y = brick["y"]
        brick_z = brick["z"] - 1
        four_pt_connections = -1
        if(min(w, h) < 2):
            four_pt_connections = 1
        else:
            four_pt_connections = 0
        for x in range(brick_x, brick_x+h):
            for y in range(brick_y, brick_y+w):
                force_key = _gen_key(x, y, brick_z)
                if(force_key not in force_dict.keys()):
                    force_dict[force_key] = dict()
                force_dict[force_key]["four_pt_connection"] = four_pt_connections
                force_dict[force_key]["brick_id"] = brick_id

                # Horizontal force from adjacent bricks
                if(_out_boundary([x-1, y], brick_x, brick_y, h, w) and x-1 >= 0 and world_grid[x-1, y, brick_z] == 1):
                    force_dict[force_key]["external_x_pos"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_external_x_pos")
                if(_out_boundary([x+1, y], brick_x, brick_y, h, w) and x+1 < world_dim[0] and world_grid[x+1, y, brick_z] == 1):
                    force_dict[force_key]["external_x_neg"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_external_x_neg")
                if(_out_boundary([x, y-1], brick_x, brick_y, h, w) and y-1 >= 0 and world_grid[x, y-1, brick_z] == 1):
                    force_dict[force_key]["external_y_pos"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_external_y_pos")
                if(_out_boundary([x, y+1], brick_x, brick_y, h, w) and y+1 < world_dim[1] and world_grid[x, y+1, brick_z] == 1):
                    force_dict[force_key]["external_y_neg"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_external_y_neg")
    
    # Construct force variables.
    for i in range(world_dim[0]):
        for j in range(world_dim[1]):
            for k in range(world_dim[2]):
                if(world_grid[i, j, k] == 0): # No brick exists
                    continue
                force_key = _gen_key(i, j, k)
                # Top knob is connected
                if(k < world_dim[2] - 1 and world_grid[i, j, k+1] != 0):
                    force_key_top = _gen_key(i, j, k+1)
                    top_four_pt_connections = force_dict[force_key_top]["four_pt_connection"]

                    # Horizontal knob presses due to connection in 4 directions
                    force_dict[force_key]["top_x_pos"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_top_x_pos")
                    force_dict[force_key]["top_x_neg"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_top_x_neg")
                    force_dict[force_key]["top_y_pos"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_top_y_pos")
                    force_dict[force_key]["top_y_neg"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_top_y_neg")
   
                    # Top is connected to a 2xX brick, 3-pt connection
                    if(top_four_pt_connections == 0):
                        force_dict[force_key]["f_up"] = model.addVars(3, vtype=gp.GRB.CONTINUOUS, name=force_key+"_f_up")
                        force_dict[force_key]["n_down"] = model.addVars(3, vtype=gp.GRB.CONTINUOUS, name=force_key+"_n_down")
                    # Top is connected to a 1xX brick, 4-pt connection
                    else:
                        force_dict[force_key]["f_up"] = model.addVars(4, vtype=gp.GRB.CONTINUOUS, name=force_key+"_f_up")
                        force_dict[force_key]["n_down"] = model.addVars(4, vtype=gp.GRB.CONTINUOUS, name=force_key+"_n_down")
                # Bottom cavity is connected
                if(k == 0 or world_grid[i, j, k-1] != 0):
                    cur_four_pt_connections = force_dict[force_key]["four_pt_connection"]

                    # Horizontal knob presses due to connection in 4 directions
                    force_dict[force_key]["bottom_x_pos"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_bottom_x_pos")
                    force_dict[force_key]["bottom_x_neg"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_bottom_x_neg")
                    force_dict[force_key]["bottom_y_pos"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_bottom_y_pos")
                    force_dict[force_key]["bottom_y_neg"] = model.addVar(vtype=gp.GRB.CONTINUOUS, name=force_key+"_bottom_y_neg")

                    # Brick is a 2xX brick, 3-pt connection
                    if(cur_four_pt_connections == 0):
                        force_dict[force_key]["f_down"] = model.addVars(3, vtype=gp.GRB.CONTINUOUS, name=force_key+"_f_down")
                        force_dict[force_key]["n_up"] = model.addVars(3, vtype=gp.GRB.CONTINUOUS, name=force_key+"_n_up")
                    # Brick is a 1xX brick, 4-pt connection
                    else:
                        force_dict[force_key]["f_down"] = model.addVars(4, vtype=gp.GRB.CONTINUOUS, name=force_key+"_f_down")
                        force_dict[force_key]["n_up"] = model.addVars(4, vtype=gp.GRB.CONTINUOUS, name=force_key+"_n_up")
    
    # Setup Constraints
    for key in lego_structure.keys():
        try:
            int(key)
        except:
            print(f"Warning: Non-integer key {key} found in lego structure. Skipping.")
            continue
        brick = lego_structure[key]
        brick_id = str(brick["brick_id"])
        if(brick["ori"] == 0):
            h = brick_library[brick_id]["height"]
            w = brick_library[brick_id]["width"]
        else:
            w = brick_library[brick_id]["height"]
            h = brick_library[brick_id]["width"]
        
        brick_weight = brick_library[brick_id]["mass"] * g_
        brick_x = brick["x"]
        brick_y = brick["y"]
        brick_z = brick["z"] - 1
        center_x = brick_x + (h - 1) / 2
        center_y = brick_y + (w - 1) / 2

        sum_x_pos_list = []
        sum_x_neg_list = []
        sum_y_pos_list = []
        sum_y_neg_list = []
        sum_z_pos_list = []
        sum_z_neg_list = []
        torque1_pos_list = []
        torque1_neg_list = []
        torque2_pos_list = []
        torque2_neg_list = []
        brick_f_down_list = []
        for i in range(brick_x, brick_x+h):
            for j in range(brick_y, brick_y+w):
                force_key = _gen_key(i, j, brick_z)

                # Horizontal presses due to adjacent bricks
                if("external_x_pos" in force_dict[force_key]):
                    model.addConstr(force_dict[force_key]["external_x_pos"] == force_dict[_gen_key(i-1, j, brick_z)]["external_x_neg"])
                    sum_x_pos_list.append(force_dict[force_key]["external_x_pos"])
                    torque2_neg_list.append(brick_unit_height / 2 * force_dict[force_key]["external_x_pos"])
                if("external_x_neg" in force_dict[force_key]):
                    model.addConstr(force_dict[force_key]["external_x_neg"] == force_dict[_gen_key(i+1, j, brick_z)]["external_x_pos"])
                    sum_x_neg_list.append(force_dict[force_key]["external_x_neg"])
                    torque2_pos_list.append(brick_unit_height / 2 * force_dict[force_key]["external_x_neg"])
                if("external_y_pos" in force_dict[force_key]):
                    model.addConstr(force_dict[force_key]["external_y_pos"] == force_dict[_gen_key(i, j-1, brick_z)]["external_y_neg"])
                    sum_y_pos_list.append(force_dict[force_key]["external_y_pos"])
                    torque1_pos_list.append(brick_unit_height / 2 * force_dict[force_key]["external_y_pos"])
                if("external_y_neg" in force_dict[force_key]):
                    model.addConstr(force_dict[force_key]["external_y_neg"] == force_dict[_gen_key(i, j+1, brick_z)]["external_y_pos"])
                    sum_y_neg_list.append(force_dict[force_key]["external_y_neg"])
                    torque1_neg_list.append(brick_unit_height / 2 * force_dict[force_key]["external_y_neg"])
                
                # Top knob is connected
                if("f_up" in force_dict[force_key]):
                    force_key_top = _gen_key(i, j, brick_z+1)
                    
                    # Horizontal knob presses
                    sum_x_pos_list.append(force_dict[force_key]["top_x_pos"])
                    sum_x_neg_list.append(force_dict[force_key]["top_x_neg"])
                    sum_y_pos_list.append(force_dict[force_key]["top_y_pos"])
                    sum_y_neg_list.append(force_dict[force_key]["top_y_neg"])
                    model.addConstr(force_dict[force_key]["top_x_pos"] == force_dict[force_key_top]["bottom_x_neg"])
                    model.addConstr(force_dict[force_key]["top_x_neg"] == force_dict[force_key_top]["bottom_x_pos"])
                    model.addConstr(force_dict[force_key]["top_y_pos"] == force_dict[force_key_top]["bottom_y_neg"])
                    model.addConstr(force_dict[force_key]["top_y_neg"] == force_dict[force_key_top]["bottom_y_pos"])

                    for k in range(len(force_dict[force_key]["f_up"])):
                        sum_z_pos_list.append(force_dict[force_key]["f_up"][k])
                        sum_z_neg_list.append(force_dict[force_key]["n_down"][k])
                        model.addConstr(force_dict[force_key]["n_down"][k] * force_dict[force_key]["f_up"][k] == 0)
                        model.addConstr(force_dict[force_key]["f_up"][k] == force_dict[force_key_top]["f_down"][k])
                        model.addConstr(force_dict[force_key]["n_down"][k] == force_dict[force_key_top]["n_up"][k])
                      
                    torque1_pos_list.append(brick_unit_height / 2 * force_dict[force_key]["top_y_neg"])
                    torque1_neg_list.append(brick_unit_height / 2 * force_dict[force_key]["top_y_pos"])
                    torque2_pos_list.append(brick_unit_height / 2 * force_dict[force_key]["top_x_pos"])
                    torque2_neg_list.append(brick_unit_height / 2 * force_dict[force_key]["top_x_neg"])
                    if(force_dict[force_key_top]["four_pt_connection"] == 1):
                        torque1_pos_list.append((j - center_y - 0.25) * brick_unit_length * force_dict[force_key]["f_up"][0])
                        torque1_neg_list.append((j - center_y - 0.25) * brick_unit_length * force_dict[force_key]["n_down"][0])
                        torque2_neg_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["f_up"][0])
                        torque2_pos_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["n_down"][0])
                        torque1_pos_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["f_up"][1])
                        torque1_neg_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["n_down"][1])
                        torque2_neg_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["f_up"][1])
                        torque2_pos_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["n_down"][1])
                        torque1_pos_list.append((j - center_y + 0.25) * brick_unit_length * force_dict[force_key]["f_up"][2])
                        torque1_neg_list.append((j - center_y + 0.25) * brick_unit_length * force_dict[force_key]["n_down"][2])
                        torque2_neg_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["f_up"][2])
                        torque2_pos_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["n_down"][2])
                        torque1_pos_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["f_up"][3])
                        torque1_neg_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["n_down"][3])
                        torque2_pos_list.append((i - center_x + 0.25) * brick_unit_length * force_dict[force_key]["n_down"][3])
                        torque2_neg_list.append((i - center_x + 0.25) * brick_unit_length * force_dict[force_key]["f_up"][3])
                    else:
                        torque1_pos_list.append((j - center_y - 0.125) * brick_unit_length * force_dict[force_key]["f_up"][0])
                        torque1_neg_list.append((j - center_y - 0.125) * brick_unit_length * force_dict[force_key]["n_down"][0])
                        torque2_pos_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["n_down"][0])
                        torque2_neg_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["f_up"][0])
                        torque1_pos_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["f_up"][1])
                        torque1_neg_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["n_down"][1])
                        torque2_pos_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["n_down"][1])
                        torque2_neg_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["f_up"][1])
                        torque1_pos_list.append((j - center_y + 0.125) * brick_unit_length * force_dict[force_key]["f_up"][2])
                        torque1_neg_list.append((j - center_y + 0.125) * brick_unit_length * force_dict[force_key]["n_down"][2])
                        torque2_pos_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["n_down"][2])
                        torque2_neg_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["f_up"][2])     
                
                # Bottom cavity is connected                  
                if("f_down" in force_dict[force_key]):
                    if(brick_z > 0):
                        force_key_bottom = _gen_key(i, j, brick_z-1)

                    # Horizontal knob presses
                    sum_x_pos_list.append(force_dict[force_key]["bottom_x_pos"])
                    sum_x_neg_list.append(force_dict[force_key]["bottom_x_neg"])
                    sum_y_pos_list.append(force_dict[force_key]["bottom_y_pos"])
                    sum_y_neg_list.append(force_dict[force_key]["bottom_y_neg"])
                    if(brick_z > 0):
                        model.addConstr(force_dict[force_key]["bottom_x_pos"] == force_dict[force_key_bottom]["top_x_neg"])
                        model.addConstr(force_dict[force_key]["bottom_x_neg"] == force_dict[force_key_bottom]["top_x_pos"])
                        model.addConstr(force_dict[force_key]["bottom_y_pos"] == force_dict[force_key_bottom]["top_y_neg"])
                        model.addConstr(force_dict[force_key]["bottom_y_neg"] == force_dict[force_key_bottom]["top_y_pos"])
                    for k in range(len(force_dict[force_key]["f_down"])):
                        sum_z_pos_list.append(force_dict[force_key]["n_up"][k])
                        sum_z_neg_list.append(force_dict[force_key]["f_down"][k])
                        sum_f_list.append(force_dict[force_key]["f_down"][k])
                        brick_f_down_list.append(force_dict[force_key]["f_down"][k])
                        model.addConstr(force_dict[force_key]["n_up"][k] * force_dict[force_key]["f_down"][k] == 0)
                        if(brick_z > 0):
                            model.addConstr(force_dict[force_key]["f_down"][k] == force_dict[force_key_bottom]["f_up"][k])
                            model.addConstr(force_dict[force_key]["n_up"][k] == force_dict[force_key_bottom]["n_down"][k])
                    
                    torque1_pos_list.append(brick_unit_height / 2 * force_dict[force_key]["bottom_y_pos"])
                    torque1_neg_list.append(brick_unit_height / 2 * force_dict[force_key]["bottom_y_neg"])
                    torque2_pos_list.append(brick_unit_height / 2 * force_dict[force_key]["bottom_x_neg"])
                    torque2_neg_list.append(brick_unit_height / 2 * force_dict[force_key]["bottom_x_pos"])
                    if(force_dict[force_key]["four_pt_connection"] == 1):
                        torque1_pos_list.append((j - center_y - 0.25) * brick_unit_length * force_dict[force_key]["n_up"][0])
                        torque1_neg_list.append((j - center_y - 0.25) * brick_unit_length * force_dict[force_key]["f_down"][0])
                        torque2_neg_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["n_up"][0])
                        torque2_pos_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["f_down"][0])
                        torque1_pos_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["n_up"][1])
                        torque1_neg_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["f_down"][1])
                        torque2_neg_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["n_up"][1])
                        torque2_pos_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["f_down"][1])
                        torque1_pos_list.append((j - center_y + 0.25) * brick_unit_length * force_dict[force_key]["n_up"][2])
                        torque1_neg_list.append((j - center_y + 0.25) * brick_unit_length * force_dict[force_key]["f_down"][2])
                        torque2_neg_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["n_up"][2])
                        torque2_pos_list.append((i - center_x) * brick_unit_length * force_dict[force_key]["f_down"][2])
                        torque1_pos_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["n_up"][3])
                        torque1_neg_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["f_down"][3])
                        torque2_neg_list.append((i - center_x + 0.25) * brick_unit_length * force_dict[force_key]["n_up"][3])
                        torque2_pos_list.append((i - center_x + 0.25) * brick_unit_length * force_dict[force_key]["f_down"][3])
                    else:
                        torque1_pos_list.append((j - center_y - 0.125) * brick_unit_length * force_dict[force_key]["n_up"][0])
                        torque1_neg_list.append((j - center_y - 0.125) * brick_unit_length * force_dict[force_key]["f_down"][0])
                        torque2_pos_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["f_down"][0])
                        torque2_neg_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["n_up"][0])
                        torque1_pos_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["n_up"][1])
                        torque1_neg_list.append((j - center_y) * brick_unit_length * force_dict[force_key]["f_down"][1])
                        torque2_pos_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["f_down"][1])
                        torque2_neg_list.append((i - center_x - 0.25) * brick_unit_length * force_dict[force_key]["n_up"][1])
                        torque1_pos_list.append((j - center_y + 0.125) * brick_unit_length * force_dict[force_key]["n_up"][2])
                        torque1_neg_list.append((j - center_y + 0.125) * brick_unit_length * force_dict[force_key]["f_down"][2])
                        torque2_pos_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["f_down"][2])
                        torque2_neg_list.append((i - center_x + 0.125) * brick_unit_length * force_dict[force_key]["n_up"][2])
                torque1_neg_list.append((j - center_y) * brick_unit_length * (brick_weight / (h * w)))
                torque2_pos_list.append((i - center_x) * brick_unit_length * (brick_weight / (h * w)))
        
        model.addConstr(force_sum_x_pos[int(key) - 1] == gp.quicksum(sum_x_pos_list[k] for k in range(len(sum_x_pos_list))))
        model.addConstr(force_sum_x_neg[int(key) - 1] == gp.quicksum(sum_x_neg_list[k] for k in range(len(sum_x_neg_list))))
        model.addConstr(force_sum_y_pos[int(key) - 1] == gp.quicksum(sum_y_pos_list[k] for k in range(len(sum_y_pos_list))))
        model.addConstr(force_sum_y_neg[int(key) - 1] == gp.quicksum(sum_y_neg_list[k] for k in range(len(sum_y_neg_list))))
        model.addConstr(force_sum_z_pos[int(key) - 1] == gp.quicksum(sum_z_pos_list[k] for k in range(len(sum_z_pos_list))))
        model.addConstr(force_sum_z_neg[int(key) - 1] == gp.quicksum(sum_z_neg_list[k] for k in range(len(sum_z_neg_list))))
        model.addConstr(force_sum_x[int(key) - 1] == force_sum_x_pos[int(key) - 1] - force_sum_x_neg[int(key) - 1])
        model.addConstr(force_sum_y[int(key) - 1] == force_sum_y_pos[int(key) - 1] - force_sum_y_neg[int(key) - 1])
        model.addConstr(force_sum_z[int(key) - 1] == force_sum_z_pos[int(key) - 1] - force_sum_z_neg[int(key) - 1] - brick_weight)
        
        model.addConstr(torque_sum_1_pos[int(key) - 1] == gp.quicksum(torque1_pos_list[k] for k in range(len(torque1_pos_list))))
        model.addConstr(torque_sum_1_neg[int(key) - 1] == gp.quicksum(torque1_neg_list[k] for k in range(len(torque1_neg_list))))
        model.addConstr(torque_sum_1[int(key) - 1] == torque_sum_1_pos[int(key) - 1] - torque_sum_1_neg[int(key) - 1])

        model.addConstr(torque_sum_2_pos[int(key) - 1] == gp.quicksum(torque2_pos_list[k] for k in range(len(torque2_pos_list))))
        model.addConstr(torque_sum_2_neg[int(key) - 1] == gp.quicksum(torque2_neg_list[k] for k in range(len(torque2_neg_list))))
        model.addConstr(torque_sum_2[int(key) - 1] == torque_sum_2_pos[int(key) - 1] - torque_sum_2_neg[int(key) - 1])

        model.addConstr(force_abs_sum_x[int(key) - 1] == gp.abs_(force_sum_x[int(key) - 1]))
        model.addConstr(force_abs_sum_y[int(key) - 1] == gp.abs_(force_sum_y[int(key) - 1]))
        model.addConstr(force_abs_sum_z[int(key) - 1] == gp.abs_(force_sum_z[int(key) - 1]))
        model.addConstr(torque_abs_sum_1[int(key) - 1] == gp.abs_(torque_sum_1[int(key) - 1]))
        model.addConstr(torque_abs_sum_2[int(key) - 1] == gp.abs_(torque_sum_2[int(key) - 1]))
        
        if(len(brick_f_down_list) > 0):
            model.addConstr(brick_max_f_down[int(key) - 1] == gp.max_(brick_f_down_list[k] for k in range(len(brick_f_down_list))))

    eq_obj = model.addVar(vtype=gp.GRB.CONTINUOUS, name="eq_obj")
    model.addConstr(eq_obj == gp.quicksum(force_abs_sum_x[i] + force_abs_sum_y[i] + force_abs_sum_z[i] + torque_abs_sum_1[i] + torque_abs_sum_2[i] 
                                          for i in range(len(force_abs_sum_z)))) # Force Equilibrium
    
    if(len(sum_f_list) > 0):
        sum_f_up = model.addVar(vtype=gp.GRB.CONTINUOUS, name="sum_f_up")
        sum_brick_max_f_down = model.addVar(vtype=gp.GRB.CONTINUOUS, name="sum_brick_max_f_down")
        model.addConstr(sum_f_up == gp.quicksum(sum_f_list[i] for i in range(len(sum_f_list))))
        model.addConstr(sum_brick_max_f_down == gp.quicksum(brick_max_f_down[i] for i in range(len(brick_max_f_down))))
        model.setObjective(eq_obj + alpha * sum_brick_max_f_down + beta * sum_f_up)
    else:
        model.setObjective(eq_obj)

    t_solve_start = time.time()
    model.modelSense = GRB.MINIMIZE
    model.update()
    model.optimize()
    t_end = time.time()
    solve_t = t_end - t_solve_start
    total_t = t_end - t_start
  
    heatmap_color = np.zeros((world_dim[0], world_dim[1], world_dim[2], 3))
    for key in lego_structure.keys():
        try:
            int(key)
        except:
            print(f"Warning: Non-integer key {key} found in lego structure. Skipping.")
            continue
        brick = lego_structure[key]
        brick_id = str(brick["brick_id"])
        if(brick["ori"] == 0):
            h = brick_library[brick_id]["height"]
            w = brick_library[brick_id]["width"]
        else:
            w = brick_library[brick_id]["height"]
            h = brick_library[brick_id]["width"]
        brick_x = brick["x"]
        brick_y = brick["y"]
        brick_z = brick["z"] - 1
        min_c = T_
        
        for i in range(brick_x, brick_x + h):
            for j in range(brick_y, brick_y + w):
                force_key = _gen_key(i, j, brick_z)
                if("f_down" in force_dict[force_key]): 
                    for k in range(len(force_dict[force_key]["f_down"])):
                        c = T_ - force_dict[force_key]["f_down"][k].X
                        min_c = min(c, min_c)
        for i in range(brick_x, brick_x + h):
            for j in range(brick_y, brick_y + w):
                if(force_abs_sum_z[int(key) - 1].X > 0 or
                  force_abs_sum_x[int(key) - 1].X > 0 or
                  force_abs_sum_y[int(key) - 1].X > 0 or
                  torque_abs_sum_1[int(key) - 1].X > 0 or
                  torque_abs_sum_2[int(key) - 1].X > 0 or
                  min_c <= 0):
                    heatmap_color[i, j, brick_z, 0] = 1
                    heatmap_color[i, j, brick_z, 1] = 1
                    heatmap_color[i, j, brick_z, 2] = 1
                else:
                    heatmap_color[i, j, brick_z, 0] = 1 - min_c / T_
    if(print_log):
        print("Obj Val:", model.objVal)
        print("Eq obj Val:", eq_obj.X)
        print("Num lego bricks: ", n_bricks)
        print("Total solve time: ", total_t, " Optimization Solve Time: ", solve_t)
    num_vars = model.NumVars
    num_constr = model.NumConstrs
    model.close()
    analysis_score = heatmap_color[:, :, :, 0]
    return analysis_score, num_vars, num_constr, total_t, solve_t
