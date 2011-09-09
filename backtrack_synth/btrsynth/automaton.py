#!/usr/bin/env python
"""
Extend Automaton class from TuLiP.

SCL; 2011 Sep, draft
"""

import btrsynth as bts

import random
import copy
import tulip.automaton


class BTAutomatonNode(tulip.automaton.AutomatonState):
    """btrsynth-related extension of TuLiP automaton nodes.

    Adds support for transition selection based on memory contents.
    See details below.

    As in the class tulip.AutomatonState, each node here has a
    transition attribute.  Each transition also has a corresponding
    callable (or None) that makes it ``enabled'' or ``unenabled''
    depending on the contents of memory.  These callables are
    maintained in the attribute ``cond'', which is kept in order with
    ``transition'' such that the k-th element of self.transition
    corresponds to the k-th element of self.cond

    If the entry in self.cond for a transition is None, then it is
    automatically ``enabled''.  By default, all
    transition-conditionals are set to None.

    Let conf_func be an element of self.cond.  Then conf_func is
    called as conf_func(memory), where ``memory'' is the contents of
    the automaton finite memory.  conf_func should return True if the
    edge is enabled, and False otherwise.

    Use BTAutomatonNode(tulip_autnode=node), where node is an instance
    of TuLiP AutomatonState class, to copy an existing object.
    """
    def __init__(self, id=-1, state={}, transition=[],
                 rule=None, cond=[],
                 tulip_autnode=None,):
        if tulip_autnode is not None:
            self.id = tulip_autnode.id
            self.state = copy.copy(tulip_autnode.state)
            self.transition = copy.copy(tulip_autnode.transition)
        else:
            tulip.automaton.AutomatonState.__init__(self, id, state, transition)
        if not callable(rule):
            self.rule = None
        else:
            self.rule = rule
        if (cond is None) or len(cond) < len(self.transition):
            self.cond = [None for k in self.transition]
        else:
            self.cond = copy.copy(cond)

    def copy(self):
        """Copy self."""
        return BTAutomatonNode(id=self.id,
                               state=copy.copy(self.state),
                               transition=copy.copy(self.transition),
                               rule=self.rule,
                               cond=copy.copy(self.cond))

    def addNodeRule(self, rule):
        """If this node is reached during execution, then apply the rule.

        Once again, note that executions are managed externally, so
        this rule has to be triggered by a corresponding method of the
        parent object, an instance of class BTAutomaton,
        e.g. execNextAutState.

        Called as rule(aut, memory, prev_node_id, node_id, this_input), where
            - aut is the owning BTAutomaton instance,

            - memory is the finite memory of the automaton,

            - node_id is the ID of the node from which this method is
              being called,

            - prev_node_id is the ID of the node preceding this one in
              the current execution path,

            - this_input is the environment action x that lead to this
              node. Specifically, this_input is a dictionary declaring
              states of environment-controlled variables, e.g. as
              given in the arguments to method execNextAutState.

        ``rule'' is then expected to return the new memory contents
        (actually, a new dictionary), or raise an exception on error.

        The one case in which ``rule'' can be not callable is as
        None. A ``rule'' of None means ``do nothing'' (i.e. leave
        automaton memory unchanged).

        If given rule is not callable (and not None), then do not
        change current rule routine, and return False. Else, return True.
        """
        if callable(rule) or (rule is None):
            self.rule = rule
            return True
        else:
            return False


class BTAutomaton(tulip.automaton.Automaton):
    """btrsynth-related extension of TuLiP Automaton.

    Use BTAutomaton(tulip_aut=aut), where aut is an instance of the
    TuLiP Automaton class, to copy an existing object.

    Includes finite (but arbitrarily large) memory and support for
    transitions conditioned on memory contents. Details are below.

    Memory in the automaton takes the form of a dictionary with keys
    being the names (type string) of memory variables, and values
    being integers. Because any execution using the automaton as a
    strategy is managed externally (e.g., using methods like
    findNextAutState, or peeking at the transition attribute of node
    instances), manipulations of memory or conditional transitioning
    is achieved using various access or triggering methods.  See
    docstrings of relevant methods --here and in BTAutomatonNode--
    for usage details.

    To support conditionally taking an edge depending on memory
    contents, all node transition lists in BTAutomaton objects are
    coupled with transition-conditional lists, which contain callables
    (or None) that determine whether the corresponding transition is
    enabled. See doc for class BTAutomatonNode for details

    WARNING: a tutorial presentation of usage is missing, and much of
    the documentation assumes (significant) background knowledge, in
    particular w.r.t. my draft paper.

    Note that nodes in this class are BTAutomatonNode objects.
    """
    def __init__(self, states_or_file=[], varnames=[], verbose=0,
                 tulip_aut=None):
        if tulip_aut is not None:
            tulip.automaton.Automaton.__init__(self)
            self.states = copy.copy(tulip_aut.states)
        else:
            tulip.automaton.Automaton.__init__(self, states_or_file=states_or_file,
                                               varnames=varnames, verbose=verbose)
        self.recastBTAutNodes()
        self.memory = None  
        # None indicates memory uninitialized; thus behaviorally
        # equivalent to automaton without memory.

    def recastBTAutNodes(self):
        """Cast any nodes of class tulip.AutomatonState into BTAutomatonNode.
        
        Catch discrepencies between transition and cond lengths. If
        cond has length zero, fill it out with None. Else raise exception.
        """
        for k in range(len(self.states)):
            if not isinstance(self.states[k], BTAutomatonNode):
                self.states[k] = BTAutomatonNode(tulip_autnode=self.states[k])
            if len(self.states[k].cond) < len(self.states[k].transition):
                if len(self.states[k].cond) == 0:
                    self.states[k].cond = [None for i in self.states[k].transition]
                else:
                    raise ValueError("FATAL: mismatch between cond and transition lists.")

    def trimDeadStates(self):
        """Replace corresponding method from TuLiP Automaton."""
        while True:
            kill_list = []
            for k in range(len(self.states)):
                if len(self.states[k].transition) == 0:
                    kill_list.append(self.states[k].id)
            if len(kill_list) == 0:
                break
            for k_ID in kill_list:
                self.removeNode(k_ID)
        self.packIDs()

    def trimUnconnectedStates(self):
        """Wrap method from TuLiP Automaton to get BTAutomatonNode nodes.
        """
        raise Exception("METHOD NOT SUPPORTED")
        tulip.automaton.Automaton.trimUnconnectedStates(self)
        self.recastBTAutNodes()

    def addAutState(self, aut_state):
        """Replace corresponding method from TuLiP Automaton class."""
        if isinstance(aut_state, BTAutomatonNode):
            self.states.append(aut_state)
        elif isinstance(aut_state, tulip.automaton.AutomatonState):
            self.states.append(BTAutomatonNode(tulip_autnode=aut_state))
        else:
            raise TypeError("given object should be instance of tulip.automaton.AutomatonState or btrsynth.automaton.BTAutomatonNode")

    def addAutNode(self, node):
        """Alias addAutState, to make up for confusing tulip.automaton naming.
        """
        self.addAutState(node)

    def removeNode(self, node_id):
        """Remove node with given ID and all dependent transitions.

        Does *not* re-map IDs.

        Raise exception on failure, else return nothing.
        """
        match_flag = False
        for ind in range(len(self.states)):
            if self.states[ind].id == node_id:
                match_flag = True
                break
        if not match_flag:
            raise TypeError("given node ID not found in automaton.")
        for prev_node in self.getAutInSet(node_id):
            prev_ind = prev_node.transition.index(node_id)
            prev_node.transition.remove(node_id)
            del prev_node.cond[prev_ind]
        del self.states[ind]

    def packIDs(self):
        """Change all node IDs to reflect position in self.states.

        ...this is stupid and almost enough motivation to overhaul
        TuLiP Automaton directly.
        """
        # Build ID map, old to new
        ID_map = dict()
        for ind in range(len(self.states)):
            ID_map[self.states[ind].id] = ind
            self.states[ind].id = ind
        # Now update transition lists
        for ind in range(len(self.states)):
            self.states[ind].transition = [ID_map[k] for k in self.states[ind].transition]
        # N.B., since we've only change IDs, conditions (in self.cond)
        # on transitions remain unchanged, hence self.conf is untouched.

    def cleanDuplicateTrans(self):
        """These duplicate transitions, i.e multiple entries in
        transition attribute of a node with the same destination ID,
        appear during the merging process.  Code could be added there
        to deal with it, but I find this post-process nicer.
        """
        for node in self.states:
            ref_transition = list(set(node.transition))
            if len(ref_transition) == len(node.transition):
                continue
            for next_ID in ref_transition:
                if node.transition.count(next_ID) > 1:
                    indices = []
                    for k in range(node.transition.count(next_ID)):
                        if len(indices) == 0:
                            indices.append(node.transition.index(next_ID))
                        else:
                            indices.append(node.transition.index(next_ID, indices[-1]+1))
                    del indices[0]  # Save one copy!
                    indices.reverse()
                    for k in indices:
                        del node.transition[k]
                        del node.cond[k]


    def fleshOutGridState(self, nominal_vars, special_var):
        """for each node in which special_var is set, expand valuation
        to include all missing variables from nominal_vars, and have
        each one of these be true while satisfying mutex.

        e.g., if node state has special_var (call it fooM) set (or
        ``true'') and is missing 3 variables (call them foo1, foo2,
        foo3) from nominal_vars, then expand this node into 4 new
        nodes, each of these having precisely one of foo1, foo2, foo3,
        fooM true. Also adjust all incoming edges to original node to
        be the same for all 4 new nodes and make the outgoing edge
        sets of all 4 new nodes the same (verbatim) as the original.

        if node state does *not* have special_var set, but is still
        missing some variables from nominal_vars, then they are added
        and cleared.  No new nodes are created in that case.

        This process is repeated for all nodes in the automaton.

        N.B., original IDs will be lost! (obviously).
        """
        self.packIDs()
        new_ID = -1
        for node in self.states:
            if node.id > new_ID:
                new_ID = node.id
        new_ID += 1  # max + 1; i.e., unique
        num_orig = len(self.states)
        for k in range(num_orig):
            if not self.states[k].state.has_key(special_var):
                raise Exception("FATAL: node "+str(self.states[k].id)+" is missing special_var, \""+special_var+"\"")
            if self.states[k].state[special_var] == 0:
                # Just expand any missing nominal vars
                for nom_var in nominal_vars:
                    if not self.states[k].state.has_key(nom_var):
                        self.states[k].state[nom_var] = 0
            else:
                # Flesh out
                missing_vars = []
                for nom_var in nominal_vars:
                    if not self.states[k].state.has_key(nom_var):
                        missing_vars.append(nom_var)
                if len(missing_vars) == 0:
                    continue  # Vacuous case
                # Fill out missing variables for this particular node
                for miss_var in missing_vars:
                    self.states[k].state[miss_var] = 0
                new_nodes = []
                # Generate new nodes, and append to Automaton
                start_ID = new_ID
                for j in range(len(missing_vars)):
                    new_nodes.append(self.states[k].copy())
                    new_nodes[-1].id = new_ID
                    new_ID += 1
                    new_nodes[-1].state[missing_vars[j]] = 1
                    new_nodes[-1].state[special_var] = 0
                self.states.extend(new_nodes)
                for node in self.states:
                    if self.states[k].id in node.transition:
                        trans_ind = node.transition.index(self.states[k].id)
                        node.transition.extend(range(start_ID, new_ID))
                        node.cond.extend([node.cond[trans_ind] for j in range(start_ID, new_ID)])
        self.packIDs()

    def removeFalseInits(self, S0):
        """Remove all nodes that look like init nodes but are not in S0.

        S0 should be a list of nodes (not IDs!).

        During patching, some nodes may become orphaned, etc. This
        method keeps repeating removal of apparent ``init'' nodes that
        do not appear in S0 until no such nodes are found.

        Does *not* re-map IDs.

        Raise exception on failure, else return nothing.
        """
        while True:
            detected_S0 = self.getAutInit()
            change_flag = False
            for node in detected_S0:
                if node in S0:
                    continue
                self.removeNode(node.id)
                change_flag = True
            if not change_flag:
                break

    def importChildAut(self, aut):
        """Import given automaton into this automaton.

        Specifically, add (copy) every node in aut, giving it a
        new unique ID to avoid conflicts, and adjust all transitions
        accordingly.

        Return the ID map, showing how IDs in given automaton map into
        this one. Raise exception on error.
        """
        if not isinstance(aut, BTAutomaton):
            if isinstance(aut, tulip.automaton.Automaton):
                aut = BTAutomaton(tulip_aut=aut)
            else:
                raise TypeError("an instance of BTAutomaton should be given.")

        max_id = -1
        for node in self.states:
            if node.id > max_id:
                max_id = node.id
        new_node_id = max_id
        id_map = dict()  # Key is original ID, value is corresponding new ID

        # Generate copies of nodes from aut and add them.
        for aut_node in aut.states:
            new_node_id += 1
            node = aut_node.copy()
            node.id = new_node_id
            id_map[aut_node.id] = new_node_id
            self.addAutNode(node)
        
        # Adjust all transitions to use new IDs
        for (k,v) in id_map.items():
            node = self.getAutState(v)
            node.transition = [id_map[i] for i in node.transition]
        return id_map

    def memInit(self, name_list):
        """Initialize memory, with variable names in the given list.

        Contents are set to 0.
        """
        self.memory = dict([(k, 0) for k in name_list])

    def memSet(self, name, new_value):
        """Set value in a memory variable.

        Returns previous value, or None on error.
        """
        if not isinstance(new_value, int) \
                or (self.memory is None) or not self.memory.has_key(name):
            return None
        prev_val = self.memory[name]
        self.memory[name] = new_value
        return prev_val
    
    def memClear(self, name):
        """Clear (set to zero) memory variable.

        Return True on succes, False on failure.
        """
        if self.memSet(name, 0) is None:
            return False
        else:
            return True

    def getMem(self):
        """Return a *copy* of automaton memory."""
        return copy.copy(self.memory)

    def triggerRule(self, prev_node_id, node_id, env_state):
        """Call ``rule'' of a node. Return True on success, else raise exception.
        """
        node = self.getAutState(node_id)
        if node == -1:
            raise Exception("Given node ID not recognized.")
        if not isinstance(node, BTAutomatonNode):
            raise Exception("node "+str(node_id)+" is not an instance of BTAutomatonNode")
        if node.rule is not None:
            try:
                new_memory = node.rule(self, self.getMem(), prev_node_id,
                                       node_id, env_state)
            except:
                print "ERROR: rule of node "+str(node_id)+" failed."
                raise
            self.memory = new_memory
        return True

    def addGroupRule(self, ID_list, rule):
        """Set node rules to ``rule'' for all nodes in ID_list.

        ...similar behavior to addGroupCondition.
        """
        for k in ID_list:
            autnode = self.getAutState(k)
            if (autnode is None) or (not autnode.addNodeRule(rule)):
                return k
        return -1

    def execNextAutState(self, node_id, env_state={},
                         randNext=False):
        """Similar to findNextAutState, but runs rule and cond.

        1. find all possible next nodes, given node_id and (full!)
           environment state env_state;

        2. if more than one next node found, call cond for node_id to
           resolve the conflict;

        3. once next node selected (whether or not cond was called),
           simulate ``taking'' the corresponding transition by
           applying the rule of the next node, given the environment
           state and memory that resulted in the transition.

        If randNext is True, then the keys of env_state are used to
        determine the environment variables (and the values of
        env_state are ignored).  One of the outgoing transitions from
        node_id is randomly (uniform probability) selected, the
        environment state (or ``valuation'') in the next node is found
        and set to env_state, and then this method proceeds as usual.
        The default is randNext = False.
        
        IMPORTANT: we deviate from findNextAutState by taking an
        argument of node ID, and returning an ID. This is a design
        decision to avoid dangers from passing object references
        directly. Actually there are not dangers per se, but the way
        tulip.automaton handles IDs and node references is sloppy,
        thus I tend to write ID-centric code when using
        tulip.automaton.

        QUIRK 1: to avoid halting, transition-conditionals are only
        invoked if more than one transition is possible; i.e., an
        ``unenabled'' edge could be taken if it otherwise has the
        correct label (environment valuation) and is the only such
        edge.

        If an error occurs (e.g. given node_id is invalid), raise
        exception indicating the failure, else return the ID of the
        next node.
        """
        node = self.getAutState(node_id)
        if node == -1:
            raise Exception("Given node ID not recognized.")

        if randNext:
            sample_node_ID = random.choice(node.transition)
            sample_node = self.getAutState(sample_node_ID)
            for k in env_state.keys():
                env_state[k] = sample_node.state[k]
        
        transition = []  # n.b., ID and index into node.transition
        
        for cand_ID_ind in range(len(node.transition)):
            cand_ID = node.transition[cand_ID_ind]
            mismatch = False
            cand_node = self.getAutState(cand_ID)
            for env_var in env_state.keys():
                if not cand_node.state.has_key(env_var):
                    raise Exception("Given environment variable not recognized.")
                if cand_node.state[env_var] != env_state[env_var]:
                    mismatch = True
                    break
            if not mismatch:
                transition.append((cand_ID, cand_ID_ind))
        if len(transition) == 0:
            raise Exception("Given environment state does not have a corresponding outgoing transition from node "+str(node_id))
        if len(transition) > 1:
            next_id = None
            for trans_cand in transition:
                if ((node.cond[trans_cand[1]] is None)
                    or node.cond[trans_cand[1]](self.getMem())):
                    if next_id is not None:
                        raise Exception("FATAL: transition conflict unresolved.")
                    else:
                        next_id = trans_cand[0]
            if next_id is None:
                raise Exception("FATAL: execNextAutState led to halt!")
        else:
            next_id = transition[0][0]
        self.triggerRule(node_id, next_id, env_state)
        return next_id


    def computeGridReg(self, nbhd, var_prefix="Y"):
        """Compute the Reg() for the given neighborhood.

        var_prefix is the prefix used in position variable naming.

        nbhd is a list of positions, the ``abstract neighborhood''
        (for gridworld, a list of (row, column) pairs).

        Combining these two, and assuming the naming scheme
        prefix_R_C, find all nodes with label (or nodes with an
        incoming edge labeled as such) indicating a corresponding
        region in the given neighborhood.

        Return list of node IDs.
        """
        Reg = []
        for k in range(len(self.states)):
            coord = bts.extract_autcoord(self.states[k], var_prefix=var_prefix)
            if coord is not None and len(coord) > 1:
                raise ValueError("more than one position match; error?")
            if (coord is not None) and (coord[0] in nbhd):
                Reg.append(self.states[k].id)
        return Reg

    def findEntry(self, subS):
        """Find all nodes in subS reachable from outside subS in one transition.

        subS should be a list of node IDs.
        Return Entry set (as a list of node IDs).
        """
        Entry = []
        for node_id in subS:
            In = self.getAutInSet(node_id)
            if In is None:
                raise Exception("Failed to compute In edge set.")
            if len(set([k.id for k in In])-set(subS)) > 0:
                Entry.append(node_id)
        return Entry

    def findExit(self, subS):
        """Find all nodes in subS that can leave subS in one transition.

        subS should be a list of node IDs.
        Return Exit set (as a list of node IDs).
        """
        Exit = []
        for node_id in subS:
            node = self.getAutState(node_id)
            if node == -1:
                raise Exception("Failed to find node with ID "+str(node_id))
            if len(set(node.transition)-set(subS)) > 0:
                Exit.append(node_id)
        return Exit

    def computeReach(self, node_id, subS):
        """Restricted reachable set from node_id, where paths in subS.

        Be careful. Reachability computation can be expensive.
        Return set of node IDs.
        """
        if node_id not in subS:
            # Degenerate case
            return []
        node = self.getAutState(node_id)
        if node == -1:
            raise Exception("Failed to find node with ID "+str(node_id))

        # Fix-point approach
        prev_set = set([node_id])
        this_set = prev_set.copy()
        while True:
            for next_node_ID in prev_set:
                next_node = self.getAutState(next_node_ID)
                for out_ID in next_node.transition:
                    if (out_ID in subS) and (out_ID not in prev_set):
                        this_set.add(out_ID)
            if prev_set == this_set:
                return this_set
            else:
                prev_set = this_set.copy()

    def writeDotFileCoordNodes(self, fname, hideZeros=False,
                     distinguishTurns=None, turnOrder=None):
        """Use btrsynth.extract_coord to infer position data.

        Otherwise, output and return behavior is similar to
        writeDotFile method.
        """
        if (distinguishTurns is not None) and (len(distinguishTurns) <= 1):
            # This is a fringe case and seemingly ok to ignore.
            distinguishTurns = None

        output = "digraph A {\n"

        # Prebuild sane state names
        state_labels = dict()
        for state in self.states:
            if distinguishTurns is None:
                state_labels[str(state.id)] = ''
            else:
                # If distinguishTurns is not a dictionary with
                # items of the form string -> list, it should
                # simulate that behavior.
                for agent_name in distinguishTurns.keys():
                    state_labels[str(state.id)+agent_name] = ''
            for (k,v) in state.state.items():
                if (not hideZeros) or (v != 0):
                    if distinguishTurns is None:
                        agent_name = ''
                    else:
                        agent_name = None
                        for agent_candidate in distinguishTurns.keys():
                            if k in distinguishTurns[agent_candidate]:
                                agent_name = agent_candidate
                                break
                        if agent_name is None:
                            print "WARNING: variable \""+k+"\" does not belong to an agent in distinguishedTurns"
                            return False

                    if (v != 0) and (bts.extract_coord(k) is not None):
                        coord = bts.extract_coord(k)
                        # Be aggressive about prefix spacing
                        label_str = "".join(coord[0].split("_"))
                        label_str += " ("+str(coord[1])+", "+str(coord[2])+")"
                    else:
                        label_str = k+": "+str(v)
                    if len(state_labels[str(state.id)+agent_name]) == 0:
                        if len(agent_name) > 0:
                            state_labels[str(state.id)+agent_name] += str(state.id)+"::"+agent_name+";\\n" + label_str
                        else:
                            state_labels[str(state.id)+agent_name] += str(state.id)+";\\n" + label_str
                    else:
                        state_labels[str(state.id)+agent_name] += ", "+label_str
            if distinguishTurns is None:
                if len(state_labels[str(state.id)]) == 0:
                    state_labels[str(state.id)] = str(state.id)+";\\n {}"
            else:
                for agent_name in distinguishTurns.keys():
                    if len(state_labels[str(state.id)+agent_name]) == 0:
                        state_labels[str(state.id)+agent_name] = str(state.id)+"::"+agent_name+";\\n {}"

        if (distinguishTurns is not None) and (turnOrder is None):
            if distinguishTurns is not None:
                turnOrder = distinguishTurns.keys()
        for state in self.states:
            if distinguishTurns is not None:
                output += "    \""+ state_labels[str(state.id)+turnOrder[0]] +"\" -> \"" \
                    + state_labels[str(state.id)+turnOrder[1]] +"\";\n"
                for agent_ind in range(1, len(turnOrder)-1):
                    output += "    \""+ state_labels[str(state.id)+turnOrder[agent_ind]] +"\" -> \"" \
                        + state_labels[str(state.id)+turnOrder[agent_ind+1]] +"\";\n"
            for trans in state.transition:
                if distinguishTurns is None:
                    output += "    \""+ state_labels[str(state.id)] +"\" -> \"" \
                        + state_labels[str(self.states[trans].id)] +"\";\n"
                else:
                    output += "    \""+ state_labels[str(state.id)+turnOrder[-1]] +"\" -> \"" \
                        + state_labels[str(self.states[trans].id)+turnOrder[0]] +"\";\n"

        output += "\n}\n"
        with open(fname, "w") as f:
            f.write(output)
        return True

    def writeDotFileCoord(self, fname, var_prefix="Y", env_prefix="X",
                          hideZeros=True, hideAgentNames=True):
        """Use btrsynth.extract_coord to infer position data.

        Otherwise, output and return behavior is similar to
        writeDotFileEdged method.
        """
        env_vars = []
        for k in self.states[0].state.keys():
            if k.startswith(env_prefix):
                env_vars.append(k)
        sys_vars = []
        for k in self.states[0].state.keys():
            if k.startswith(var_prefix):
                sys_vars.append(k)
        if len(sys_vars) == 0:
            return False

        # Make looping possible
        agents = {"env" : env_vars,
                  "sys" : sys_vars}
        
        output = "digraph A {\n"

        # Prebuild sane state names
        state_labels = dict()
        for state in self.states:
            for agent_name in agents.keys():
                state_labels[str(state.id)+agent_name] = ''
            for (k,v) in state.state.items():
                if (not hideZeros) or (v != 0):
                    agent_name = None
                    for agent_candidate in agents.keys():
                        if k in agents[agent_candidate]:
                            agent_name = agent_candidate
                            break
                    if agent_name is None:
                        import pdb; pdb.set_trace()
                        print "WARNING: variable \""+k+"\" does not belong to an agent in distinguishedTurns"
                        return False

                    if (v != 0) and (bts.extract_coord(k) is not None):
                        coord = bts.extract_coord(k)
                        # Be aggressive about prefix spacing
                        label_str = "".join(coord[0].split("_"))
                        if coord[1:] == (-1, -1):
                            label_str += " (nil)"
                        else:
                            label_str += " ("+str(coord[1])+", "+str(coord[2])+")"
                    else:
                        label_str = k+": "+str(v)
                    if len(state_labels[str(state.id)+agent_name]) == 0:
                        if len(agent_name) > 0 and not hideAgentNames:
                            state_labels[str(state.id)+agent_name] += str(state.id)+"::"+agent_name+";\\n" + label_str
                        else:
                            state_labels[str(state.id)+agent_name] += str(state.id)+";\\n" + label_str
                    else:
                        state_labels[str(state.id)+agent_name] += ", "+label_str
            
            for agent_name in agents.keys():
                if (agent_name == "env") and (len(env_vars) == 0):
                    # Special case: deterministic problem
                    state_labels[str(state.id)+agent_name] = ""
                elif len(state_labels[str(state.id)+agent_name]) == 0:
                    if not hideAgentNames:
                        state_labels[str(state.id)+agent_name] = str(state.id)+"::"+agent_name+";\\n {}"
                    else:
                        state_labels[str(state.id)+agent_name] = str(state.id)+";\\n {}"

        # Initialization point
        output += "    \"\" [shape=circle,style=filled,color=black];\n"
        
        # All nodes and edges
        for state in self.states:
            if len(self.getAutInSet(state.id)) == 0:
                # Treat init nodes specially
                output += "    \"\" -> \"" \
                    + state_labels[str(state.id)+"sys"] +"\" [label=\""
                output += state_labels[str(state.id)+"env"] + "\"];\n"
            for trans in state.transition:
                output += "    \""+ state_labels[str(state.id)+"sys"] +"\" -> \"" \
                    + state_labels[str(self.states[trans].id)+"sys"] +"\" [label=\""
                output += state_labels[str(self.states[trans].id)+"env"] + "\"];\n"

        output += "\n}\n"
        with open(fname, "w") as f:
            f.write(output)
        return True
