#!/usr/bin/env python
"""
Backtracking TL synthesis implementation for gridworlds.

Scott C. Livingston,
2011, 2012.
"""

from automaton import BTAutomaton, BTAutomatonNode
from gridworld import *

import itertools
import copy
import numpy as np
import tulip.grgameint


def create_nominal(W, env_init_list, soln_str, restrict_radius=1,
                   var_prefix="Y", env_prefix="X"):
    """Create nominal automaton.

    Generate nominal controller assuming environment can move, but not
    anywhere that inteferes with nominal path.

    nominal path should be sequence of coordinates, beginning with an
    initial position all the way through loop closure. The point not
    at the end of the path that should be attached to it has line
    ending with "*". e.g.,

    0 0
    0 1
    0 2
    0 3 *
    1 3
    2 3
    1 3
    0 3
   
    soln_str should contain the above path data. N.B., end-of-line
    delimiter should be '\n'.
    
    Return instance of btsynth.BTAutomaton
    """
    # Parse nominal path string
    nom_path = []
    loop_marker = None
    line_counter = -1
    for line in soln_str.split("\n"):
        line_counter += 1
        parts = line.split()
        if len(parts) < 2:
            continue
        if len(parts) > 2:
            loop_marker = len(nom_path)
        try:
            nom_path.append((int(parts[0]), int(parts[1])))
        except ValueError:
            print "ERROR: malformed line in nominal path data, at line "+str(line_counter)
            print line
            exit(-1)  # Be aggressive; force quit
    if loop_marker is None:
        print "ERROR: no loop closure marker found in nominal path data."
        exit(-1)  # Be aggressive; force quit

    # Build list of system variables
    pos_indices = [k for k in itertools.product(range(W.shape[0]), range(W.shape[1]))]
    sys_vars = []
    for k in pos_indices:
        sys_vars.append(var_prefix+"_"+str(k[0])+"_"+str(k[1]))
    sys_vars_nowhere = dict([(var, 0) for var in sys_vars])

    # Build node without environment
    aut = BTAutomaton()
    last_id = 0
    for step in nom_path:
        node = BTAutomatonNode(id=last_id,
                               state=sys_vars_nowhere.copy(),
                               transition=[last_id+1])
        node.state[var_prefix+"_"+str(step[0])+"_"+str(step[1])] = 1
        aut.states.append(node)
        last_id += 1
    aut.states[-1].transition = [loop_marker]

    # Augment for all environment variables
    for env_ind in range(len(env_init_list)):
        env_center = env_init_list[env_ind]
        env_vars = []
        nowhere_present = False
        for i in range(env_center[0]-restrict_radius, env_center[0]+restrict_radius+1):
            for j in range(env_center[1]-restrict_radius, env_center[1]+restrict_radius+1):
                if i < 0 or j < 0 or i > W.shape[0]-1 or j > W.shape[1]-1:
                    nowhere_present = True
                else:
                    env_vars.append(env_prefix+"_"+str(env_ind)+"_"+str(i)+"_"+str(j))
        if nowhere_present:
            for node in aut.states:
                node.state[env_prefix+"_"+str(env_ind)+"_n_n"] = 1
            env_vars.append(env_prefix+"_"+str(env_ind)+"_n_n")
            aut.fleshOutGridState(nominal_vars=env_vars,
                                  special_var=env_prefix+"_"+str(env_ind)+"_n_n")
        else:
            for node in aut.states:
                node.state[env_vars[0]] = 1
            aut.fleshOutGridState(nominal_vars=env_vars,
                                  special_var=env_vars[0])

    return aut


def gen_navobs_soln(init_list, goal_list, W, num_obs,
                    env_init_list, env_goal_list=None,
                    goals_disjunct=None,
                    restrict_radius=1,
                    var_prefix="Y", env_prefix="X",
                    fname_prefix="tempsyn"):
    """Generate solution as in gen_dsoln but now with dynamic obstacles.

    This is a limited extension to the problem considered in
    gen_dsoln. Here we introduce a finite number (num_obs) of
    obstacles that navigate in restricted regions of the map W, thus
    requiring a reactive controller to safely avoid them while
    visiting the goal locations infinitely often.

    env_init_list is the center position for each (env-controlled)
    obstacle. Thus it must be that len(env_init_list) = num_obs

    If env_goal_list is None (default), then regard the obstacle
    initial positions as their goal positions.

    N.B., we do not verify that env_goals are within restrict_radius
    of initial env obstacle positions.

    restrict_radius determines the domains of obstacles; cf. notes in
    function LTL_world.

    Return instance of btsynth.BTAutomaton on success;
    None if not realizable, or an error occurs.
    """
    # Argument error checking
    if (len(init_list) == 0) or (num_obs < 0):
        return None

    # Handle default env obstacle goal case
    if env_goal_list is None:
        env_goal_list = env_init_list

    # Handle degenerate case of no obstacles (thus, deterministic problem).
    if num_obs < 1:
        return gen_dsoln(init_list=init_list, goal_list=goal_list, W=W,
                         goals_disjunct=goals_disjunct,
                         var_prefix=var_prefix, fname_prefix=fname_prefix)

    ########################################
    # Environment prep
    obs_bounds = []
    env_str = []
    for k in range(num_obs):
        center_loc = env_goal_list[k]
        row_low = center_loc[0]-restrict_radius
        nowhere = [False,  # corresponds to row_low
                   False,  # row_high
                   False,  # col_low
                   False]  # col_high
        row_low = center_loc[0]-restrict_radius
        if row_low < 0:
            row_low = 0
            nowhere[0] = True
        row_high = center_loc[0]+restrict_radius
        if row_high > W.shape[0]-1:
            row_high = W.shape[0]-1
            nowhere[1] = True
        col_low = center_loc[1]-restrict_radius
        if col_low < 0:
            col_low = 0
            nowhere[2] = True
        col_high = center_loc[1]+restrict_radius
        if col_high > W.shape[1]-1:
            col_high = W.shape[1]-1
            nowhere[3] = True
        obs_bounds.append((row_low, row_high, col_low, col_high, nowhere))
        env_str.append(LTL_world(W, var_prefix="e."+env_prefix+"_"+str(k),
                                 center_loc=center_loc,
                                 restrict_radius=restrict_radius))

    # Environment progress: always eventually obstacle returns to
    # initial position.
    env_goal_str = []
    for k in range(num_obs):
        center_loc = env_goal_list[k]
        if (center_loc[0] >= 0 and center_loc[0] <= W.shape[0]-1
            and center_loc[1] >= 0 and center_loc[1] <= W.shape[1]-1):
            env_goal_str.append("[]<>(e."+env_prefix+"_"+str(k)+"_"+str(center_loc[0])+"_"+str(center_loc[1])+")")
        else:
            env_goal_str.append("[]<>(e."+env_prefix+"_"+str(k)+"_n_n)")
    if len(env_goal_str) == 0:
        env_goal_str = ""
    else:
        env_goal_str = " & ".join(env_goal_str) + " &\n"

    env_init_str = ""
    if (env_init_list is not None) and len(env_init_list) > 0:
        for obs_ind in range(num_obs):
            loc = env_init_list[obs_ind]
            if len(env_init_str) > 0:
                env_init_str += " | "
            if (loc[0] < row_low or loc[0] > row_high
                or loc[1] < col_low or loc[1] > col_high):
                env_init_str += "(" + "e."+env_prefix+"_"+str(obs_ind)+"_n_n)"
            else:
                env_init_str += "(" + "e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    
    ########################################
    # Sys prep
    safety_str = LTL_world(W, var_prefix="s."+var_prefix)

    init_str = ""
    for loc in init_list:
        if len(init_str) > 0:
            init_str += " | "
        init_str += "(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    goal_str = ""
    for loc in goal_list:
        if len(goal_str) > 0:
            goal_str += " & "
        goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    
    if (goals_disjunct is not None) and len(goals_disjunct) > 0:
        goal_dstr = ""
        for loc in goals_disjunct:
            if len(goal_dstr) == 0:
                if len(goal_str) == 0:
                    goal_dstr += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
                else:
                    goal_dstr += " & []<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_dstr += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_dstr += " )"
        goal_str += goal_dstr

    ########################################
    # Interaction: avoid collisions
    coll_str = ""
    for k in range(num_obs):
        for i in range(obs_bounds[k][0], obs_bounds[k][1]+1):
            for j in range(obs_bounds[k][2], obs_bounds[k][3]+1):
                if len(coll_str) == 0:
                    coll_str += "[]( "
                else:
                    coll_str += " & "
                coll_str += "!(" + "s."+var_prefix+"_"+str(i)+"_"+str(j)
                coll_str += " & e."+env_prefix+"_"+str(k)+"_"+str(i)+"_"+str(j)
                coll_str += ")"
    if len(coll_str) > 0:
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
        for k in range(num_obs):
            if (obs_bounds[k][4][0] or obs_bounds[k][4][1]
                or obs_bounds[k][4][2] or obs_bounds[k][4][3]):
                f.write("\t\t" + env_prefix+"_"+str(k)+"_n_n" + " : boolean;\n")
            for i in range(obs_bounds[k][0], obs_bounds[k][1]+1):
                for j in range(obs_bounds[k][2], obs_bounds[k][3]+1):
                    f.write("\t\t" + env_prefix+"_"+str(k)+"_"+str(i)+"_"+str(j) + " : boolean;\n")

    # Create SPC file
    with open(fname_prefix+".spc", "w") as f:
        f.write("LTLSPEC\n")
        if len(env_init_str) > 0:
            f.write("("+env_init_str+") & \n")
        f.write(env_goal_str)
        first_obs_flag = True
        for k in range(num_obs):
            if first_obs_flag:
                f.write(env_str[k])
                first_obs_flag = False
            else:
                f.write(" &\n" + env_str[k])
        f.write("\n;\n\nLTLSPEC\n")
        f.write("("+init_str+") & \n" + goal_str + " & \n" + safety_str)
        if len(coll_str) > 0:
            f.write(" & \n" + coll_str)
        f.write("\n;")
    
    # Try JTLV synthesis
    realizable = tulip.grgameint.solveGame(smv_file=fname_prefix+".smv",
                                           spc_file=fname_prefix+".spc",
                                           init_option=1, file_exist_option="r",
                                           heap_size="-Xmx2048m")

    if not realizable:
        return None
    else:
        return BTAutomaton(fname_prefix+".aut")


def navobs_sim(init, aut, W_actual, num_obs, var_prefix="Y", env_prefix="X",
               num_it=100):
    """Sister to dsim, but now for solutions from gen_navobs_soln.

    If the world is fully known a priori, then tulip.grsim suffices
    for generating a simulation.  navobs_sim simulates uncertainty and
    aborts when given controller breaks.

    Env locative variable prefixes are expected to take the form
    prefix_N_R_C, where R, C are (row, column) as usual, and N is the
    number of the obstacle.

    num_obs could be determined from analysing the automaton... future work.

    Same return values as in dsim, but also list of obstacle positions
    at time of failure.
    """
    # Handle initialization as a special case.
    if W_actual[init[0]][init[1]] == 1:
        return init, None

    # Based on given env_prefix, extract all environment variable names.
    env_vars = []
    for k in aut.states[0].state:
        if k.startswith(env_prefix):
            env_vars.append(k)
    # To avoid re-computing list comprehension at every step...
    fake_env_state = dict([(k,0) for k in env_vars])
    
    # Main simulation execution
    i, j = init
    loc_var = var_prefix+"_"+str(i)+"_"+str(j)
    next_node = aut.findAllAutPartState({loc_var : 1})
    if len(next_node) == 0:
        return init, None
    next_node = next_node[0]
    history = [(i,j),]  # Initialize trace
    it_counter = 0
    while it_counter < num_it:
        it_counter += 1
        this_node = next_node
        next_node_id = aut.execNextAutState(next_node.id,
                                            env_state=fake_env_state,
                                            randNext=True)
        next_node = aut.getAutState(next_node_id)
        next_loc = extract_autcoord(next_node, var_prefix=var_prefix)
        if next_loc is None:
            raise ValueError("Given automaton is incomplete; reached deadend.")
        if len(next_loc) > 1:
            raise ValueError("Given automaton invalid; more than one locative prop true, despite mutual exclusion.")
        next_loc = next_loc[0]
        if W_actual[next_loc[0]][next_loc[1]] == 1:
            obs_poses = []
            for obs_ind in range(num_obs):
                obs_poses.append(extract_autcoord(this_node,
                                                  var_prefix=env_prefix+"_"+str(obs_ind))[0])
            return history, next_loc, obs_poses
        history.append(next_loc)
    obs_poses = []
    for obs_ind in range(num_obs):
        obs_poses.append(extract_autcoord(next_node,
                                          var_prefix=env_prefix+"_"+str(obs_ind))[0])
    return history, True, obs_poses


def gen_dsoln(init_list, goal_list, W, goals_disjunct=None,
              var_prefix="Y", fname_prefix="tempsyn"):
    """Generate deterministic solution, given initial and goal states.

    init_list is a list of pairs (row, col), signifying locations in
    the world matrix W from which the system can be initialized.
    Similarly for goal_list, but locations to visit infinitely often.

    If goals_disjunct is not None, then it must be a list of (row,
    col) pairs specifying goals to be combined disjunctively in a
    single []<>... formula.  The default (None) does nothing.

    Return instance of btsynth.BTAutomaton on success;
    None if not realizable, or an error occurs.
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
    for loc in goal_list:
        if len(goal_str) > 0:
            goal_str += " & "
        goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    if goals_disjunct is not None:
        goal_dstr = ""
        for loc in goals_disjunct:
            if len(goal_dstr) == 0:
                if len(goal_str) == 0:
                    goal_dstr += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
                else:
                    goal_dstr += " & []<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_dstr += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_dstr += " )"
        goal_str += goal_dstr

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
    realizable = tulip.grgameint.solveGame(smv_file=fname_prefix+".smv",
                                           spc_file=fname_prefix+".spc",
                                           init_option=1, file_exist_option="r")

    if not realizable:
        return None
    else:
        return BTAutomaton(fname_prefix+".aut")


def dsim(init, aut, W_actual, var_prefix="Y", num_it=100):
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

    num_it is like a watchdog timer. Only num_it iterations will
    execute before quitting (regardless of failure occurring).

    If initial location is not even possible, then return initial
    location and None (rather than intended action).

    If quit because max number of iterations reached, history is
    returned and True (rather than an intended location).
    """
    # Handle initialization as a special case.
    if W_actual[init[0]][init[1]] == 1:
        return init, None

    # Special case of initial location possible and zero iterations run.
    if num_it == 0:
        return init, True
    
    # Main simulation execution
    i, j = init
    loc_var = var_prefix+"_"+str(i)+"_"+str(j)
    next_node = aut.findAllAutPartState({loc_var : 1})
    if len(next_node) == 0:
        return init, None
    next_node = next_node[0]
    history = [(i,j),]  # Initialize trace
    it_counter = 0
    while it_counter < num_it:
        it_counter += 1
        next_node_id = aut.execNextAutState(next_node.id, env_state={})
        next_node = aut.getAutState(next_node_id)
        # N.B., execNextAutState raises an exception on error.
        next_loc = extract_autcoord(next_node, var_prefix=var_prefix)
        if next_loc is None:
            raise ValueError("Given automaton is incomplete; reached deadend.")
        if len(next_loc) > 1:
            raise ValueError("Given automaton invalid; more than one locative prop true, despite mutual exclusion.")
        next_loc = next_loc[0]
        if W_actual[next_loc[0]][next_loc[1]] == 1:
            return history, next_loc
        history.append(next_loc)
    return history, True


def cond_anynot(memory):
    if len(memory) == 0:
        raise ValueError("Cannot apply transition-conditional on empty memory.")
    if 0 in memory.values():
        return True
    else:
        return False

def cond_all(memory):
    if len(memory) == 0:
        raise ValueError("Cannot apply transition-conditional on empty memory.")
    if 0 in memory.values():
        return False
    else:
        return True

def rule_clearall(aut, memory, prev_node_id, node_id, this_input):
    """Clear all memory values, regardless."""
    ##print "DEBUG: automaton memory cleared by node "+str(node_id)
    return dict([(k, 0) for k in memory.keys()])

def rule_setmatch(aut, memory, prev_node_id, node_id, this_input):
    """Set memory variables nonzero depending on edge-node labeling.

    For each memory variable name that is also an environment or
    system variable, if the present node (from which the rule was
    invoked) is labeled (or its incoming edge is labeled) with this
    variable being nonzero (i.e. True as a Boolean variable), then set
    that memory to 1.

    N.B., this rule acts one-way, i.e. it can only *set* memory
    variables, not clear them.
    """
    node = aut.getAutState(node_id)
    if node == -1:
        raise Exception("FATAL: rule called with invalid node ID.")
    for k in memory.keys():
        if node.state.has_key(k) and node.state[k] != 0:
            ##print "DEBUG: automaton memory \""+str(k)+"\" set by node "+str(node_id)
            memory[k] = 1
    return memory


def btsim_d(init, goal_list, aut, W_actual, num_steps=100, var_prefix="Y"):
    """Backtrack/patching algorithm, applied to deterministic problem.

    This case is elementary and, being non-adversarial, may be better
    addressed by other methods (e.g., graph search or D*).
    Nonetheless it provides a decent base case for testing the idea.

    num_steps is the number of simulation steps to run; this count is
    across corrections.  That is, the basic execution sequence is
      1. sim to fault or num_steps reached;
      2. if fault, run backtrack/patch algorithm to improve automaton;
        goto step 1 (after updating aut)
      3. else (total step count num_steps reached; simulation without
        fault), quit.
    
    (num_steps is not the same as num_it in the function dsim.)

    Returns an updated (to reflect the corrected controller)
    instance of btsynth.BTAutomaton and the known world map at
    time of completion.  Note that the ``known world'' may not match
    the given W_actual, because some parts of the world may never be
    visited (hence, uncertainty not corrected).

    If patching is impossible or seems as hard as the original
    (overall) problem, then return (None, None).
    """
    step_count = 0
    while True:
        if step_count == num_steps:
            return aut, None

        # Loop invariants
        if num_steps-step_count < 0:
            raise ValueError("overstepped bt dsim loop.")
        
        # Sim
        history, intent = dsim(init, aut, W_actual, var_prefix=var_prefix,
                               num_it=num_steps-step_count)
        if intent is True:
            return aut, None
        step_count += len(history)

        # Detect special case
        if intent in goal_list:
            return None, None

        # Patch (terminology follows that of the paper)
        gamma = 1  # radius
        delta = 1  # increment
        iteration_count = 0
        while True:
            iteration_count += 1
            radius = gamma + (iteration_count-1)*delta
            nbhd_inclusion = []  # Use Manhattan distance as metric
            for i in range(intent[0]-radius, intent[0]+radius+1):
                for j in range(intent[1]-radius, intent[1]+radius+1):
                    if i >= 0 and i < W_actual.shape[0] \
                            and j >= 0 and j < W_actual.shape[1]:
                        nbhd_inclusion.append((i, j))
            if len(nbhd_inclusion) == 0:
                raise ValueError("gamma radius is too small; neighborhood is empty.")
            patch_goal_list = []
            for v in nbhd_inclusion:
                if v in goal_list:
                    patch_goal_list.append(v)
            fail_loc_var = var_prefix+"_"+str(intent[0])+"_"+str(intent[1])
            
            # Set of nodes in M corresponding to abstract nbhd.
            Reg = aut.computeGridReg(nbhd=nbhd_inclusion, var_prefix=var_prefix)
            S0 = aut.getAutInit()
            Init = set([node.id for node in S0]) & set(Reg)
            Entry = aut.findEntry(Reg)
            Exit = aut.findExit(Reg)
            
            W_patch, offset = subworld(W_actual, nbhd_inclusion)
            # Shift coordinates to be w.r.t. W_patch
            for ind in range(len(patch_goal_list)):
                patch_goal_list[ind] = (patch_goal_list[ind][0]-offset[0],
                                        patch_goal_list[ind][1]-offset[1])
            
            patch_auts = []
            fail_flag = False
            for l in Init|set(Entry):
                init_loc = extract_autcoord(aut.getAutState(l), var_prefix=var_prefix)[0]
                init_loc = (init_loc[0]-offset[0], init_loc[1]-offset[1])
                local_goals_IDs = list(aut.computeReach(l, Reg) & set(Exit))
                local_goals = []
                for goal_ID in local_goals_IDs:
                    local_goals.append(extract_autcoord(aut.getAutState(goal_ID),
                                                        var_prefix=var_prefix)[0])
                    local_goals[-1] = (local_goals[-1][0]-offset[0],
                                       local_goals[-1][1]-offset[1])
                aut_patch = gen_dsoln(init_list=[init_loc], goal_list=patch_goal_list,
                                      W=W_patch, goals_disjunct=local_goals,
                                      var_prefix=var_prefix, fname_prefix="tempsyn-"+str(l))
                if aut_patch is not None:
                    patch_auts.append((aut_patch, l, local_goals_IDs))
                else:
                    fail_flag = True
                    break
            if not fail_flag:
                break

        # Merge (in several steps)

        # Set rule to clearing mem cells for nodes in the original M
        for node in aut.states:
            node.addNodeRule(rule_clearall)

        # Adjust map coordinates from local (patch-centric) to global,
        # and expand set of variables of the patch to include all
        # those of the (original) global problem.
        patch_id_maps = []
        full_state = aut.states[0].state.keys()  # Pick out full variable list
        for aut_ind in range(len(patch_auts)):
            Ml = patch_auts[aut_ind][0]
            for node in Ml.states:
                prev_keys = node.state.keys()
                (i, j) = extract_autcoord(node, var_prefix=var_prefix)[0]
                node.state = dict([(k, 0) for k in full_state])
                node.state[var_prefix+"_"+str(i+offset[0])+"_"+str(j+offset[1])] = 1
                node.addNodeRule(rule_setmatch)
            patch_id_maps.append(aut.importChildAut(Ml))

        # Undo offset of the part of sys goal list addressed in patch
        for k in range(len(patch_goal_list)):
            patch_goal_list[k] = (patch_goal_list[k][0]+offset[0],
                                  patch_goal_list[k][1]+offset[1])

        # Add memory for these goals
        aut.memInit([var_prefix+"_"+str(i)+"_"+str(j) for (i, j) in patch_goal_list])

        # Attach entry and exit points
        for aut_ind in range(len(patch_auts)):
            l = patch_auts[aut_ind][1]
            Ml = patch_auts[aut_ind][0]
            local_goals_IDs = patch_auts[aut_ind][2]
            entry_node = aut.getAutState(l)
            match_list = Ml.findAllAutState(entry_node.state)
            if len(match_list) == 0:
                raise Exception("FATAL")
            # Shortcut, given we are only addressing deterministic
            # (non-adversarial) problem in this example.
            entry_node.transition = [patch_id_maps[aut_ind][match_list[0].transition[0]]]

            match_flag = False
            for local_goal_ID in local_goals_IDs:
                goal_node = aut.getAutState(local_goal_ID)
                match_list = Ml.findAllAutState(goal_node.state)
                if len(match_list) > 0:
                    match_flag = True
                for match_node in match_list:
                    if len(aut.getMem()) > 0:
                        for k in range(len(aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond)):
                            if aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond[k] is None:
                                aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond[k] = cond_anynot
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([cond_all for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition.extend(goal_node.transition)
                    else:
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond = [None for k in aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition]
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([None for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition = goal_node.transition[:]
                    
                    
            if not match_flag:
                raise Exception("FATAL")
            
        # Delete blocked nodes and dependent edges
        kill_list = []
        for ind in range(len(aut.states)):
            if extract_autcoord(aut.states[ind], var_prefix=var_prefix)[0] == intent:
                kill_list.append(aut.states[ind].id)
        for kill_id in kill_list:
            aut.removeNode(kill_id)
        aut.packIDs()
        
        # Pick-off invalid initial nodes
        aut.removeFalseInits(S0)
        aut.packIDs()


def btsim_navobs(init, goal_list, aut, W_actual,
                 env_init_list, restrict_radius=1,
                 num_obs=None,
                 num_steps=100,
                 var_prefix="Y", env_prefix="X"):
    """Sister to btsim_d, but now for solutions from gen_navobs_soln.
    
    if num_obs is None, set it to len(env_init_list); this is a
    temporary hack till I clean up the code.

    Cf. doc for navobs_sim and gen_navobs_soln.
    """
    if num_obs is None:
        num_obs = len(env_init_list)
    # We do not (yet) allow env obstacle init/goals to differ by user choice
    env_goal_list = env_init_list[:]
    step_count = 0
    while True:
        if step_count == num_steps:
            return aut, None

        # Loop invariants
        if num_steps-step_count < 0:
            raise ValueError("overstepped bt sim_navobs loop.")
        
        # Sim
        history, intent, obs_poses = navobs_sim(init, aut, W_actual,
                                                num_obs=num_obs,
                                                var_prefix=var_prefix,
                                                env_prefix=env_prefix,
                                                num_it=num_steps-step_count)
        if intent is True:
            return aut, None
        step_count += len(history)

        # Detect special case
        if intent in goal_list:
            return None, None

        # Patch (terminology follows that of the paper)
        gamma = 1  # radius, increment
        radius = 0
        while True:
            radius += gamma
            print "r_inc = "+str(radius)
            nbhd_inclusion = []  # Use Manhattan distance as metric
            for i in range(intent[0]-radius, intent[0]+radius+1):
                for j in range(intent[1]-radius, intent[1]+radius+1):
                    if i >= 0 and i < W_actual.shape[0] \
                            and j >= 0 and j < W_actual.shape[1]:
                        nbhd_inclusion.append((i, j))
            if len(nbhd_inclusion) == 0:
                raise ValueError("gamma radius is too small; neighborhood is empty.")
            patch_goal_list = []
            patch_env_goals = []
            for v in nbhd_inclusion:
                if v in goal_list:
                    patch_goal_list.append(v)
                if v in env_goal_list:
                    patch_env_goals.append((env_goal_list.index(v), v))
            fail_loc_var = var_prefix+"_"+str(intent[0])+"_"+str(intent[1])
            
            # Re-sort env obstacle goals
            patch_env_goal_list = env_init_list[:]
            for env_g in patch_env_goals:
                patch_env_goal_list[env_g[0]] = env_g[1]
            
            # Set of nodes in M corresponding to abstract nbhd.
            Reg = aut.computeGridReg(nbhd=nbhd_inclusion, var_prefix=var_prefix)
            S0 = aut.getAutInit()
            Init = set([node.id for node in S0]) & set(Reg)
            Entry = aut.findEntry(Reg)
            Exit = aut.findExit(Reg)
            if len(Reg) == aut.size():
                raise Exception("FATAL: reduced to global problem, i.e. S = Reg.")
            
            W_patch, offset = subworld(W_actual, nbhd_inclusion)
            # Shift coordinates to be w.r.t. W_patch
            for ind in range(len(patch_goal_list)):
                patch_goal_list[ind] = (patch_goal_list[ind][0]-offset[0],
                                        patch_goal_list[ind][1]-offset[1])
            for ind in range(len(patch_env_goal_list)):
                patch_env_goal_list[ind] = (patch_env_goal_list[ind][0]-offset[0],
                                            patch_env_goal_list[ind][1]-offset[1])

            patch_auts = []
            fail_flag = False
            for l in Init|set(Entry):
                init_loc = extract_autcoord(aut.getAutState(l), var_prefix=var_prefix)[0]
                init_loc = (init_loc[0]-offset[0], init_loc[1]-offset[1])
                local_env_init = env_init_list[:]
                for obs in range(num_obs):
                    local_env_init[obs] = extract_autcoord(aut.getAutState(l),
                                                           var_prefix=env_prefix+"_"+str(obs))[0]
                    local_env_init[obs] = (local_env_init[obs][0]-offset[0],
                                           local_env_init[obs][1]-offset[1])
                if len(Exit) == 0:
                    # Special case where it suffices to remain local
                    # forever (all system goals in here, etc.).
                    local_goals_IDs = []  
                else:
                    local_goals_IDs = list(aut.computeReach(l, Reg) & set(Exit))
                local_goals = []
                for goal_ID in local_goals_IDs:
                    local_goals.append(extract_autcoord(aut.getAutState(goal_ID),
                                                        var_prefix=var_prefix)[0])
                    local_goals[-1] = (local_goals[-1][0]-offset[0],
                                       local_goals[-1][1]-offset[1])
                local_goals = list(set(local_goals))  # Remove redundancy
                aut_patch = gen_navobs_soln(init_list=[init_loc], goal_list=patch_goal_list,
                                            W=W_patch, num_obs=num_obs,
                                            env_init_list=local_env_init,
                                            env_goal_list=patch_env_goal_list,
                                            restrict_radius=restrict_radius,
                                            goals_disjunct=local_goals,
                                            var_prefix=var_prefix, env_prefix=env_prefix,
                                            fname_prefix="tempsyn-"+str(l))
                if aut_patch is not None:
                    patch_auts.append((aut_patch, l, local_goals_IDs))
                else:
                    fail_flag = True
                    break
            if not fail_flag:
                break

        # Merge (in several steps)

        for aut_ind in range(len(patch_auts)):
            patch_auts[aut_ind][0].trimDeadStates()

        # Set rule to clearing mem cells for nodes in the original M
        for node in aut.states:
            node.addNodeRule(rule_clearall)

        # Adjust map coordinates from local (patch-centric) to global,
        # and expand set of variables of the patch to include all
        # those of the (original) global problem.
        patch_id_maps = []
        env_vars_list = []
        env_nowhere_vars = []
        for obs in range(num_obs):
            # Pick out full list of env variables, for each obstacle
            env_vars_list.append(prefix_filt(aut.states[0].state,
                                             prefix=env_prefix+"_"+str(obs)))
            env_vars_list[-1] = env_vars_list[-1].keys()
            env_nowhere_vars.append(env_prefix+"_"+str(obs)+"_n_n")
        # Pick out full list of sys variables
        sys_vars = prefix_filt(aut.states[0].state, prefix=var_prefix)  
        for aut_ind in range(len(patch_auts)):
            Ml = patch_auts[aut_ind][0]
            for node in Ml.states:
                temp_state = copy.copy(node.state)
                node.state = {}
                for (k,v) in temp_state.items():
                    ex_result = extract_coord(k)
                    if ((ex_result is None)
                        or (ex_result[1] == -1 and ex_result[2] == -1)):
                        # not spatially-dependent variable; ignore
                        node.state[k] = v
                    else:
                        node.state[ex_result[0]+"_"+str(ex_result[1]+offset[0])+"_"+str(ex_result[2]+offset[1])] = v
                for k in sys_vars.keys():
                    if not node.state.has_key(k):
                        node.state[k] = 0
            for obs in range(num_obs):
                Ml.fleshOutGridState(env_vars_list[obs],
                                     special_var=env_nowhere_vars[obs])
            for node in Ml.states:
                node.addNodeRule(rule_setmatch)

            patch_id_maps.append(aut.importChildAut(Ml,
                                                    tags={"color": (np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256), 0.5),
                                                          "cluster_id": aut_ind}))

        # Undo offset of the part of sys goal list addressed in patch
        for k in range(len(patch_goal_list)):
            patch_goal_list[k] = (patch_goal_list[k][0]+offset[0],
                                  patch_goal_list[k][1]+offset[1])

        # Add memory for these goals
        aut.memInit([var_prefix+"_"+str(i)+"_"+str(j) for (i, j) in patch_goal_list])

        # Attach entry and exit points
        for aut_ind in range(len(patch_auts)):
            l = patch_auts[aut_ind][1]
            Ml = patch_auts[aut_ind][0]
            local_goals_IDs = patch_auts[aut_ind][2]
            entry_node = aut.getAutState(l)
            sys_state = prefix_filt(entry_node.state, prefix=var_prefix)
            match_list = Ml.findAllAutPartState(sys_state)
            if len(match_list) == 0:
                raise Exception("FATAL")
            for match in match_list:
                for k in range(len(entry_node.transition)):
                    next_node = aut.getAutState(entry_node.transition[k])
                    env_state = prefix_filt(next_node.state, prefix=env_prefix)
                    # Find environment (outward edge) labels that are consistent.
                    # Note that multiple matches leads to multiple overwrites, silently!
                    for j in range(len(match.transition)):
                        match_next = Ml.getAutState(match.transition[j])
                        if set(env_state.items()).issubset(set(match_next.state.items())):
                            entry_node.transition[k] = patch_id_maps[aut_ind][match.transition[j]]
                            entry_node.cond[k] = None

            if len(local_goals_IDs) == 0:
                # Special case where it suffices to remain local
                # forever (all system goals in here, etc.).
                match_flag = True
            for local_goal_ID in local_goals_IDs:
                goal_node = aut.getAutState(local_goal_ID)
                sys_state = prefix_filt(goal_node.state, prefix=var_prefix)
                match_list = Ml.findAllAutPartState(sys_state)
                if len(match_list) > 0:
                    match_flag = True
                for match_node in match_list:
                    if len(aut.getMem()) > 0:
                        for k in range(len(aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond)):
                            if aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond[k] is None:
                                aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond[k] = cond_anynot
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([cond_all for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition.extend(goal_node.transition)
                    else:
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond = [None for k in aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition]
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([None for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition = goal_node.transition[:]
                    
                    
            if not match_flag:
                raise Exception("FATAL")
            
        # Delete blocked nodes and dependent edges
        kill_list = []
        for ind in range(len(aut.states)):
            if extract_autcoord(aut.states[ind], var_prefix=var_prefix)[0] == intent:
                kill_list.append(aut.states[ind].id)
        for kill_id in kill_list:
            aut.removeNode(kill_id)
        aut.packIDs()
        
        # Pick-off invalid initial nodes, and other clean-up
        aut.removeFalseInits(S0)
        aut.packIDs()
        aut.cleanDuplicateTrans()


def to_formula(aut_node):
    """Take the given automaton node and return formula equivalent to state.

    NOTE: ...this function, or one like it, may be general enough to
    place in the tulip.automaton module directly.

    ASSUMES ALL VARIABLES APPEARING IN GIVEN NODE ARE BOOLEAN.

    Returns string type.
    """
    out_str = ""
    for (k, v) in aut_node.state.items():
        if len(out_str) > 0:
            out_str += " & "
        if v > 0:
            out_str += "("
        else:
            out_str += "(!"
        out_str += k+")"
    return out_str


def prefix_filt(d, prefix):
    """return all items in dictionary d with key with given prefix."""
    match_list = []
    for k in d.keys():
        if isinstance(k, str):
            if k.startswith(prefix):
                match_list.append(k)
    return dict([(k, d[k]) for k in match_list])
