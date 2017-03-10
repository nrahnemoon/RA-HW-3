from Queue import LifoQueue
import numpy

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        self.planning_env.InitializePlot( goal_config)
        plan = []
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)
        plan.append(goal_config)
        nodes_stack = LifoQueue()
        start_id=self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        nodes_stack.put(start_id)
        end_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        curr_id=start_id
        seen=[start_id]
        while curr_id!=end_id:
            successors = self.planning_env.GetSuccessors(curr_id)
            for i in range(0,numpy.size(successors,0)):
                # print "succesor"+str(self.planning_env.discrete_env.NodeIdToConfiguration(successors[i]))
                # print "grid"+str(self.planning_env.discrete_env.NodeIdToGridCoord(successors[i]))
                # print "from depth"+str(successors)
                # print "from depth single"+str(successors[i])
                #print seen
                if successors[i] in seen:
                    continue
                else:
                    nodes_stack.put(successors[i]) 
                    # print nodes_stack
            node_id=nodes_stack.get()
            self.nodes[node_id]=curr_id
            self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(curr_id),self.planning_env.discrete_env.NodeIdToConfiguration(node_id) );
            curr_id=node_id
            seen.append(curr_id)
        plan_id=curr_id
        while plan_id != start_id:
            successor_id=self.nodes.get(plan_id) 
            plan.insert(1,self.planning_env.discrete_env.NodeIdToConfiguration(successor_id))
            plan_id=successor_id
            #raw_input('Press any key to begin planning')
        

        return plan
