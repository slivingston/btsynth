#!/usr/bin/env python
"""
Extend Automaton class from TuLiP.

SCL; 2011 Sep, draft
"""

import btrsynth as bts

import copy
import tulip.automaton


class BTAutomatonNode(tulip.automaton.AutomatonState):
    """btrsynth-related extension of TuLiP automaton nodes.

    Adds support for transition selection based on memory contents.

    Use BTAutomatonNode(tulip_autnode=node), where node is an instance
    of TuLiP AutomatonState class, to copy an existing object.
    """
    def __init__(self, id=-1, state={}, transition=[],
                 tulip_autnode=None):
        if tulip_autnode is not None:
            self.id = tulip_autnode.id
            self.state = copy.copy(tulip_autnode.state)
            self.transition = copy.copy(tulip_autnode.transition)
        else:
            tulip.automaton.AutomatonState.__init__(self, id, state, transition)
        self.rule = None
        self.cond = None

    def copy(self):
        """Copy self."""
        return BTAutomatonNode(id=self.id,
                               state=copy.copy(self.state),
                               transition=copy.copy(self.transition))

    def addNodeRule(self, rule):
        """If this node is reached during execution, then apply the rule.

        Once again, note that executions are managed externally, so
        this rule has to be triggered by a corresponding method of the
        parent object, an instance of class BTAutomaton,
        e.g. execNextAutState.

        Called as rule(memory, prev_node_id, this_input), where
            - memory is the finite memory of the automaton,

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

    def addEdgeCondition(self, cond):
        """If more than one transition takes an input (env state), call cond.

        Called as cond(memory, next_input), where
            - memory is the finite memory of the automaton (a
              dictionary; see notes for class BTAutomaton),

            - next_input is the environment action x that leads to
              more than one possible outward transition (hence,
              conflict). It is a dictionary specifying states of
              environment variables, e.g. as given in the arguments to
              execNextAutState.

        cond is then expected to return the appropriate transition (ID
        of next node), or raise an exception on error.

        N.B., unlike a node ``rule'', a condition cannot be None
        because the purpose of cond is to resolve a conflict (more
        than one possible transition).

        If given cond is not callable, then do not change current
        condition routine, and return False. Else, return True.
        """
        if callable(cond):
            self.cond = cond
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
    docstrings of relevant methods --here and for BTAutomatonNode--
    for usage details.

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
        """
        for k in range(len(self.states)):
            if not isinstance(self.states[k], BTAutomatonNode):
                self.states[k] = BTAutomatonNode(tulip_autnode=self.states[k])

    def trimDeadStates(self):
        """Tweak method from TuLiP Automaton to get BTAutomatonNode nodes.
        """
        tulip.automaton.Automaton.trimDeadStates(self)
        self.recastBTAutNodes()

    def trimUnconnectedStates(self):
        """Tweak method from TuLiP Automaton to get BTAutomatonNode nodes.
        """
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

    def triggerRule(self, node_id, env_state):
        """Call ``rule'' of a node. Return True on success, else raise exception.
        """
        node = self.getAutState(node_id)
        if node == -1:
            raise Exception("Given node ID not recognized.")
        if not isinstance(node, BTAutomatonNode):
            raise Exception("node "+str(node_id)+" is not an instance of BTAutomatonNode")
        if node.rule is not None:
            try:
                new_memory = node.rule(self.getMem(), node_id, env_state)
            except:
                raise Exception("rule of node "+str(node_id)+" failed.")
            self.memory = new_memory
        return True
        

    def addGroupCondition(self, ID_list, cond):
        """Set edge condition to cond for all nodes in ID_list.

        On success return -1, on failure return first ID in ID_list
        where error occurred.

        N.B., changes are incremental, so on error some of the nodes
        have already been updated!
        """
        for k in ID_list:
            autnode = self.getAutState(k)
            if (autnode is None) or (not autnode.addEdgeCondition(cond)):
                return k
        return -1

    def addGroupRule(self, ID_list, rule):
        """Set node rules to ``rule'' for all nodes in ID_list.

        ...similar behavior to addGroupCondition.
        """
        for k in ID_list:
            autnode = self.getAutState(k)
            if (autnode is None) or (not autnode.addNodeRule(rule)):
                return k
        return -1

    def execNextAutState(self, node_id, env_state={}):
        """Similar to findNextAutState, but runs rule and cond.

        1. find all possible next nodes, given node_id and (full!)
           environment state env_state;

        2. if more than one next node found, call cond for node_id to
           resolve the conflict;

        3. once next node selected (whether or not cond was called),
           simulate ``taking'' the corresponding transition by
           applying the rule of the next node, given the environment
           state and memory that resulted in the transition.
        
        IMPORTANT: we deviate from findNextAutState by taking an
        argument of node ID, and returning an ID. This is a design
        decision to avoid dangers from passing object references
        directly. Actually there are not dangers per se, but the way
        tulip.automaton handles IDs and node references is sloppy,
        thus I tend to write ID-centric code when using
        tulip.automaton.
        
        If an error occurs (e.g. given node_id is invalid), raise
        exception indicating the failure, else return the ID of the
        next node.
        """
        node = self.getAutState(node_id)
        if node == -1:
            raise Exception("Given node ID not recognized.")
        
        transition = []
        for cand_id in node.transition:
            mismatch = False
            cand_node = self.getAutState(cand_id)
            for env_var in env_state.keys():
                if not cand_node.state.has_key(env_var):
                    raise Exception("Given environment variable not recognized.")
                if cand_node.state[env_var] != env_state[env_var]:
                    mismatch = True
                    break
            if not mismatch:
                transition.append(cand_id)
        if len(transition) == 0:
            raise Exception("Given environment state does not have a corresponding outgoing transition from node "+str(node_id))
        if len(transition) > 1:
            try:
                next_id = node.cond(self.getMem(), env_state)
            except:
                raise Exception("cond of node "+str(node_id)+" failed.")
        else:
            next_id = transition[0]
        self.triggerRule(next_id, env_state)
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
                raise ValueError("more than position match; error?")
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

    def computeReach(self, node_id, subS, reachable=set()):
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
        this_reachable = set()
        for next_id in set(node.transition):
            if (next_id not in subS) or (next_id in reachable):
                continue
            this_reachable |= self.computeReach(next_id,
                                           subS=subS,
                                           reachable=set([node_id, next_id])|this_reachable.copy())
        return reachable | this_reachable

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
                            printWarning("variable \""+k+"\" does not belong to an agent in distinguishedTurns")
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
                        printWarning("variable \""+k+"\" does not belong to an agent in distinguishedTurns")
                        return False

                    if (v != 0) and (bts.extract_coord(k) is not None):
                        coord = bts.extract_coord(k)
                        # Be aggressive about prefix spacing
                        label_str = "".join(coord[0].split("_"))
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
