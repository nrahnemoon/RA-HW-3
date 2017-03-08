from RRTTree import RRTTree
from Queue import Queue
import time

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        start_time = time.time()
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)
        plan.append(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        seen = [ start_id ]
        
        queue = Queue()
        queue.put(start_id)

        tree = RRTTree(self.planning_env, start_config)
        nodeIdToTreeIdDict = {}
        nodeIdToTreeIdDict[start_id] = 0

        while not queue.empty():

            curr_id = queue.get()
            curr_tree_id = nodeIdToTreeIdDict[curr_id]

            for node_id in self.planning_env.GetSuccessors(curr_id):
                
                if not node_id in seen:

                    seen.append(node_id)
                    node_tree_id = tree.AddVertex(self.planning_env.discrete_env.NodeIdToConfiguration(node_id))
                    nodeIdToTreeIdDict[node_id] = node_tree_id
                    tree.AddEdge(curr_tree_id, node_tree_id)
                    # self.planning_env.PlotEdge(curr_tree_id, node_tree_id);
                    queue.put(node_id)

                    if node_id == goal_id:
                        while True:
                            node_tree_id = tree.edges[node_tree_id];
                            node_config = tree.vertices[node_tree_id];
                            if (node_tree_id == tree.GetRootId()):
                                print("--- %s seconds ---" % (time.time() - start_time))
                                print("--- %s path length ---" % len(plan))
                                print("--- %s vertices ---" % len(tree.vertices))
                                return plan
                            else:
                                plan.insert(1, node_config)
        return plan
