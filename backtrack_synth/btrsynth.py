#!/usr/bin/env python
"""
SCL; 2011 Aug 22, draft
"""

import numpy as np
import tulip.grgameint
import tulip.automaton


def errmsg(m):
    print "ERROR: "+m

def read_world(world_str):
    """Process given string as a World description, return binary matrix.

    The returned uint8 NumPy matrix (nd-array) has 0 or 1 value
    entries, with 0 being unoccupied, 1 occupied.  On error, return
    None.

    Assume lines are separated by '\n'; note that read_worldf ensures
    this before invoking read_world.
    """
    first_line_found = False
    line_counter = 0
    world = np.array([], dtype=np.uint8)  # In case given string is lame.
    for line in world_str.split("\n"):
        line_counter += 1
        if len(line) == 0 or line[0] == "#":
            continue
        if not first_line_found:
            first_line_found = True
            entries = line.split()
            try:
                if len(entries) != 2:
                    raise ValueError
                num_rows = int(entries[0])
                num_cols = int(entries[1])
            except ValueError:
                errmsg("Malformed size spec at line "+str(line_counter))
                return None
            world = np.zeros(shape=(num_rows, num_cols), dtype=np.uint8)
            row_counter = 0
        else:
            if row_counter >= num_rows:
                errmsg("Too many rows! (as of line "+str(line_counter)+")")
                return None
            if line[0] == "-":  # Empty row?
                continue
            for col_str in line.split():
                try:
                    col = int(col_str)
                    if (col < 0) or (col >= num_cols):
                        raise ValueError
                except ValueError:
                    errmsg("Malformed row spec at line "+str(line_counter))
                    return None
                world[row_counter][col] = 1
            row_counter += 1
    return world

def read_worldf(fname):
    """File wrapper for read_world function."""
    wstr_list = []
    with open(fname, "r") as f:
        for line in f:
            wstr_list.append(line)
    return read_world("\n".join(wstr_list))

def pretty_world(W, history=None):
    """Given world matrix W, return pretty-for-printing string.

    If history is not None, it should be a pair consisting of a list
    of locations (signifying a path) and a failed-but-desired end
    location.  These are exactly returned by the function dsim.

    Return None on failure (or lame arguments).
    """
    if W is None:
        return None
    # Fill in W with magic values if history is given.
    # LEGEND:
    #   2 - regularly visited location;
    #   3 - desired but failed location, was occupied;
    #   4 - desired but failed location, was not occupied;
    if history is not None:
        for loc in history[0]:
            if W[loc[0]][loc[1]] != 0 and W[loc[0]][loc[1]] != 2:
                raise ValueError("Mismatch between given history and world at " \
                                     + "("+str(loc[0])+", "+str(loc[1])+")")
            W[loc[0]][loc[1]] = 2
        if W[history[1][0]][history[1][1]] == 0:
            W[history[1][0]][history[1][1]] = 4
        else:
            W[history[1][0]][history[1][1]] = 3
    out_str = "-"*(W.shape[1]+2) + "\n"
    for i in range(W.shape[0]):
        out_str += "|"
        for j in range(W.shape[1]):
            if W[i][j] == 0:
                out_str += " "
            elif W[i][j] == 1:
                out_str += "*"
            elif W[i][j] == 2:
                out_str += "."
            elif W[i][j] == 3:
                out_str += "X"
            elif W[i][j] == 4:
                out_str += "?"
            else:
                raise ValueError("Unrecognized world W encoding.")
        out_str += "|\n"
        #out_str += "|" + ''.join([k*"*"+(1-k)*" " for k in W[i]]) + "|\n"
    out_str += "-"*(W.shape[1]+2) + "\n"
    return out_str

def LTL_world(W, var_prefix="obs"):
    """Convert world matrix W into an LTL formula describing transitions.
    
    The variables are named according to var_prefix_R_C, where
    var_prefix is given, R is the row, and column the cell
    (0-indexed).

    Return the formula string on success; None if failure.
    """
    out_str = ""
    first_subformula = True
    # Safety, transitions
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if W[i][j] == 1:
                continue  # Cannot start from an occupied cell.
            if not first_subformula:
                out_str += " &\n\t"
            out_str += "[]("+var_prefix+"_"+str(i)+"_"+str(j)+" -> next("
            out_str += var_prefix+"_"+str(i)+"_"+str(j)
            if i > 0 and W[i-1][j] == 0:
                out_str += " | " + var_prefix+"_"+str(i-1)+"_"+str(j)
            if j > 0 and W[i][j-1] == 0:
                out_str += " | " + var_prefix+"_"+str(i)+"_"+str(j-1)
            if i < W.shape[0]-1 and W[i+1][j] == 0:
                out_str += " | " + var_prefix+"_"+str(i+1)+"_"+str(j)
            if j < W.shape[1]-1 and W[i][j+1] == 0:
                out_str += " | " + var_prefix+"_"+str(i)+"_"+str(j+1)
            out_str += "))"
            first_subformula = False
    
    # Safety, static
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if W[i][j] == 1:
                if not first_subformula:
                    out_str += " &\n\t"
                out_str += "[](!(" + var_prefix+"_"+str(i)+"_"+str(j) + "))"
                first_subformula = False

    # Safety, mutex
    first_subformula = True
    out_str += "\n& []("
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if W[i][j] == 1:
                continue
            if not first_subformula:
                out_str += " | "
            out_str += "(" + var_prefix+"_"+str(i)+"_"+str(j)
            for i_sub in range(W.shape[0]):
                for j_sub in range(W.shape[1]):
                    if (W[i_sub][j_sub] == 1) or (i == i_sub and j == j_sub):
                        continue
                    out_str += " & (!" + var_prefix+"_"+str(i_sub)+"_"+str(j_sub) + ")"
            out_str += ")"
            first_subformula = False
    out_str += ")"
            
    return out_str


def gen_navobs_soln(init_list, goal_list, W, num_obs, env_goal_list,
                    var_prefix="X", fname_prefix="tempsyn", env_prefix="Y",
                    disjunction_goals=False, env_disjunction_goals=False):
    """Generate solution as in gen_dsoln but now with dynamic obstacles.

    This is a limited extension to the problem considered in
    gen_dsoln. Here we introduce a finite number (num_obs) of
    obstacles that navigate in the map W, thus requiring a reactive
    controller to safely avoid them while visiting the goal locations
    infinitely often.

    env_goal_list is a list of locations that must be visited by the
    environment obstacles infinitely often.  This reduces the
    likelihood that a permissible env strategy is to corner the
    vehicle forever.  If env_goal_list has length greater than one,
    these locations can be combined in a conjunctive or disjunctive
    form, by setting env_disjunction_goals.  See comments in gen_dsoln.

    Return instance of tulip.automaton.Automaton on success;
    None if error.
    """
    # Argument error checking
    if (len(init_list) == 0) or (num_obs < 0):
        return None

    # Handle degenerate case of no obstacles (thus, deterministic problem).
    if num_obs < 1:
        return gen_dsoln(init_list=init_list, goal_list=goal_list, W=W,
                         var_prefix=var_prefix, fname_prefix=fname_prefix,
                         disjunction_goals=disjunction_goals)

    ########################################
    # Environment prep
    env_str = [LTL_world(W, var_prefix="e."+env_prefix+"_"+str(k)) \
                   for k in range(num_obs)]

    env_goal_str = ""
    if not env_disjunction_goals:
        for obs_ind in range(num_obs):
            for loc in env_goal_list:
                if len(env_goal_str) > 0:
                    env_goal_str += " & "
                env_goal_str += "[]<>(" + "e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    else:
        for obs_ind in range(num_obs):
            if len(env_goal_str) > 0:
                env_goal_str += " & "
            for loc in env_goal_list:
                if len(env_goal_str) == 0:
                    env_goal_str += "[]<>( e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1])
                env_goal_str += " | e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1])
            env_goal_str += " )"
    
    ########################################
    # Sys prep
    safety_str = LTL_world(W, var_prefix="s."+var_prefix)

    init_str = ""
    for loc in init_list:
        if len(init_str) > 0:
            init_str += " | "
        init_str += "(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    goal_str = ""
    if not disjunction_goals:
        for loc in goal_list:
            if len(goal_str) > 0:
                goal_str += " & "
            goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    else:
        for loc in goal_list:
            if len(goal_str) == 0:
                goal_str += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_str += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_str += " )"

    ########################################
    # Interaction: avoid collisions
    coll_str = ""
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if len(coll_str) == 0:
                coll_str += "[]( "
            else:
                coll_str += " & "
            coll_str += "!(" + "s."+var_prefix+"_"+str(i)+"_"+str(j)
            for obs_ind in range(num_obs):
                coll_str += " & e."+env_prefix+"_"+str(obs_ind)+"_"+str(i)+"_"+str(j)
            coll_str += ")"
    coll_str += ")"
            

    ########################################
    # Create SMV file
    with open(fname_prefix+".smv", "w") as f:
        # Some parts of this code are copied from tulip/rhtlp.py
        f.write("MODULE main \n")
        f.write("\tVAR\n")
        f.write("\t\te : env();\n")
        f.write("\t\ts : sys();\n\n")
        f.write("MODULE sys \n")
        f.write("\tVAR\n")
        for i in range(W.shape[0]):
            for j in range(W.shape[1]):
                f.write("\t\t" + var_prefix+"_"+str(i)+"_"+str(j) + " : boolean;\n")
        f.write("MODULE env \n")
        f.write("\tVAR\n")
        for obs_ind in range(num_obs):
            for i in range(W.shape[0]):
                for j in range(W.shape[1]):
                    f.write("\t\t" + env_prefix+"_"+str(obs_ind)+"_"+str(i)+"_"+str(j) + " : boolean;\n")

    # Create SPC file
    with open(fname_prefix+".spc", "w") as f:
        f.write("LTLSPEC\n")
        #f.write("("+init_str+") & \n" + env_goal_str + " & \n")
        f.write(env_goal_str)
        for obs_ind in range(num_obs):
            f.write(" &\n" + env_str[obs_ind])
        f.write("\n;\n\nLTLSPEC\n")
        f.write(goal_str + " & \n" + safety_str)
        f.write(" & \n" + coll_str)
        f.write("\n;")
    
    # Try JTLV synthesis
    tulip.grgameint.solveGame(smv_file=fname_prefix+".smv",
                              spc_file=fname_prefix+".spc",
                              init_option=1, file_exist_option="r")

    return tulip.automaton.Automaton(fname_prefix+".aut")


def navobs_sim():
    """Sister to dsim, but now for solutions from gen_navobs_soln.
    """
    pass


def gen_dsoln(init_list, goal_list, W, var_prefix="X", fname_prefix="tempsyn",
              disjunction_goals=False):
    """Generate deterministic solution, given initial and goal states.

    init_list is a list of pairs (row, col), signifying locations in
    the world matrix W from which the system can be initialized.
    Similarly for goal_list, but locations to visit infinitely often.

    If disjunction_goals is True, then the given goal positions appear
    in a single []<>... formula, combined in a disjunction (i.e. "or"
    statements).  The default (disjunction_goals = False) is for each
    goal position to occur infinitely often, hence one []<>... formula
    per goal.
    
    Return instance of tulip.automaton.Automaton on success;
    None if error.
    """
    if len(init_list) == 0:
        return None

    safety_str = LTL_world(W, var_prefix="s."+var_prefix)

    init_str = ""
    for loc in init_list:
        if len(init_str) > 0:
            init_str += " | "
        init_str += "(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    goal_str = ""
    if not disjunction_goals:
        for loc in goal_list:
            if len(goal_str) > 0:
                goal_str += " & "
            goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    else:
        for loc in goal_list:
            if len(goal_str) == 0:
                goal_str += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_str += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_str += " )"

    # Create SMV file
    with open(fname_prefix+".smv", "w") as f:
        # Some parts of this code are copied from tulip/rhtlp.py
        f.write("MODULE main \n")
        f.write("\tVAR\n")
        f.write("\t\te : env();\n")
        f.write("\t\ts : sys();\n\n")
        f.write("MODULE sys \n")
        f.write("\tVAR\n")
        for i in range(W.shape[0]):
            for j in range(W.shape[1]):
                f.write("\t\t" + var_prefix+"_"+str(i)+"_"+str(j) + " : boolean;\n")
        f.write("MODULE env \n")
        f.write("\tVAR\n")

    # Create SPC file
    with open(fname_prefix+".spc", "w") as f:
        f.write("LTLSPEC\n;\n\nLTLSPEC\n")
        f.write("("+init_str+") & \n" + goal_str + " & \n" + safety_str)
        f.write("\n;")
    
    # Try JTLV synthesis
    tulip.grgameint.solveGame(smv_file=fname_prefix+".smv",
                              spc_file=fname_prefix+".spc",
                              init_option=1, file_exist_option="r")

    return tulip.automaton.Automaton(fname_prefix+".aut")


def dsim(init, aut, W_actual, var_prefix="X"):
    """Simulate application of controller (automaton) on actual world.

    The thrust is that a solution was synthesized for a world W that
    is not correct, and so you would like to execute that controller
    on the true world W_actual and see when it breaks.

    var_prefix is the name prefix used for locative variables,
    e.g. what was using when calling gen_dsoln (their default settings
    should match).

    Return history trace, ending at location (i.e. position of
    vehicle) of failure, and intended next location (which implies an
    action, since we are treating deterministic problem setting).

    If initial location is not even possible, then return initial
    location and None (rather than intended action).
    """
    # Handle initialization as a special case.
    if W_actual[init[0]][init[1]] == 1:
        return init, None
    
    # Main simulation execution
    i, j = init
    loc_var = var_prefix+"_"+str(i)+"_"+str(j)
    next_node = aut.findAllAutPartState({loc_var : 1})
    if len(next_node) == 0:
        return init, None
    next_node = next_node[0]
    history = [(i,j),]  # Initialize trace
    while True:
        next_node = aut.findNextAutState(next_node, env_state={},
                                         deterministic_env=True)
        next_loc = None
        for (k, v) in next_node.state.items():
            if v != 1:  # We are only interested in True variables
                continue
            k_parts = k.split("_")
            if len(k_parts) < 3:  # Conforms to locative naming scheme?
                continue
            if "_".join(k_parts[:-2]) == var_prefix:
                next_loc = extract_coord(k)
                break
        if next_loc is None:
            raise ValueError("Given automaton is incomplete; reached deadend.")
        if W_actual[next_loc[0]][next_loc[1]] == 1:
            return history, next_loc
        history.append(next_loc)


def btsim_d(init, aut, W_actual, var_prefix="X"):
    """Backtrack/patching algorithm, applied to deterministic problem.

    Note that this case is elementary and, being non-adversarial, may
    be better addressed by other methods (e.g., graph search or D*).
    Nonetheless it provides a decent base case for testing the idea.

    Returns an updated (to reflected the corrected controller)
    instance of tulip.automaton.Automaton and the known world map at
    time of completion.  Note that the ``known world'' may not match
    the given W_actual, because some parts of the world may never be
    visited (hence, uncertainty not corrected).
    """
    pass


def extract_coord(var_name):
    """Assuming prefix_R_C format, extract and return (row,column) pair.

    If error, return None or throw exception.
    """
    if not isinstance(var_name, str):
        raise TypeError("extract_coord: invalid argument type; must be string.")
    name_frags = var_name.split("_")
    if len(name_frags) < 3:
        return None
    try:
        col = int(name_frags[-1])
        row = int(name_frags[-2])
    except ValueError:
        return None
    return (row, col)


########################################
# UNIT TESTS
########################################

def extract_coord_test():
    assert(extract_coord("test_3_0") == (3, 0))
    assert(extract_coord("test3_0") is None)
