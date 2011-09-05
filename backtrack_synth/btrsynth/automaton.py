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
    of TuLiP AutomatonState class, to deep copy an existing object.
    """
    def __init__(self, id=-1, state={}, transition=[],
                 tulip_autnode=None):
        if tulip_autnode is not None:
            self.id = tulip_autnode.id
            self.state = copy.deepcopy(tulip_autnode.state)
            self.transition = copy.deepcopy(tulip_autnode.transition)
        else:
            tulip.automaton.AutomatonState.__init__(self, id, state, transition)
        self.cond = None

    def addEdgeCondition(self, cond):
        """If more than one transition satisfies an input symbol, call cond.

        Calling as cond(memory, node_id, node_Out)

        where memory is the finite memory of the automaton (a
        dictionary; see notes for class BTAutomaton), node_id is the
        ID of the calling node, and node_Out is set of outward
        transitions that are in conflict (hence a subset of self.transition).

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
    TuLiP Automaton class, to deep copy an existing object.

    Includes finite (but arbitrarily large) memory and support for
    transition conditioned on memory contents. Details are below.

    Memory in the automaton takes the form of a dictionary with keys
    being the memory variables names, and values being
    integers. Transition conditionals are provided to resolve
    conflicts between possible edges.  Let v be a node, and let x be
    the next input symbol.  If more than one transition in Out(v) is
    labeled with x, then the conflict dictionary for v is searched for
    key x. If found, the conditional statement is evaluated given
    contents of memory, and the appropriate edge is thus selected.
    See docstrings of relevant methods for usage details.

    Note that nodes in this class are BTAutomatonNode objects.

    Manipulation of memory contents is managed externally using class
    access methods.
    """
    def __init__(self, states_or_file=[], varnames=[], verbose=0,
                 tulip_aut=None):
        if tulip_aut is not None:
            tulip.automaton.Automaton.__init__(self)
            self.states = copy.deepcopy(tulip_aut.states)
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
