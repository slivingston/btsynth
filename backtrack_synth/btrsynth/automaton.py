#!/usr/bin/env python
"""
Extend Automaton class from TuLiP.

SCL; 2011 Sep, draft
"""

import btrsynth as bts

import copy
import tulip.automaton


class BTAutomaton(tulip.automaton.Automaton):
    """(This may be superfluous.)

    Use BTAutomaton(tulip_aut=aut), where aut is an instance of the
    TuLiP Automaton class, to deep copy an existing object.

    ...motivated by the need for some methods specific to my experiments.
    """
    def __init__(self, states_or_file=[], varnames=[], verbose=0,
                 tulip_aut=None):
        if tulip_aut is not None:
            tulip.automaton.Automaton.__init__(self)
            self.states = copy.deepcopy(tulip_aut.states)
        else:
            tulip.automaton.Automaton.__init__(self, states_or_file=states_or_file,
                                               varnames=varnames, verbose=verbose)

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
        if len(env_vars) == 0 or len(sys_vars) == 0:
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
                if len(state_labels[str(state.id)+agent_name]) == 0:
                    if not hideAgentNames:
                        state_labels[str(state.id)+agent_name] = str(state.id)+"::"+agent_name+";\\n {}"
                    else:
                        state_labels[str(state.id)+agent_name] = str(state.id)+";\\n {}"

        for state in self.states:
            for trans in state.transition:
                output += "    \""+ state_labels[str(state.id)+"sys"] +"\" -> \"" \
                    + state_labels[str(self.states[trans].id)+"sys"] +"\" [label=\""
                output += state_labels[str(self.states[trans].id)+"env"] + "\"];\n"

        output += "\n}\n"
        with open(fname, "w") as f:
            f.write(output)
        return True
