import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-0.5, -2.]
        self.upper_limits = [3., 2.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        node=self.discrete_env.NodeIdToGridCoord(node_id)
        for i in range(0,numpy.size(node,0)):
            node_add = list(node)
            node_add[i] = node_add[i]+1
            print "node_add"+str(node_add)
            node_add_id = self.discrete_env.GridCoordToNodeId(node_add)
            if (self.IsInLimits(node_add_id)==True and self.IsInCollision(node_add_id)!=True):
                successors.append(node_add_id)
        for j in range(0,numpy.size(node,0)):
            node_minus = list(node)
            node_minus[j] = node_minus[j]-1
            node_minus_id = self.discrete_env.GridCoordToNodeId(node_minus)

            print "node_minus"+str(node_minus)
            if (self.IsInLimits(node_minus_id)==True and self.IsInCollision(node_minus_id)!=True):
                successors.append(node_minus_id)
        print successors
        return successors

    def ComputeDistance(self, start_id, end_id):

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_node=self.discrete_env.NodeIdToConfiguration(start_id)
        end_node=self.discrete_env.NodeIdToConfiguration(end_id)
        diff_vec = numpy.subtract(end_node,start_node)
        dist = numpy.sqrt(numpy.dot(numpy.transpose(diff_vec),diff_vec))
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = self.ComputeDistance(start_id,goal_id)

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

    def IsInCollision(self, node_id):
        orig_config = self.robot.GetTransform()
        location = self.discrete_env.NodeIdToConfiguration(node_id)
        config =orig_config
        config[:2,3]=location
        env = self.robot.GetEnv()
        with env:
            self.robot.SetTransform(config)           
        collision = env.CheckCollision(self.robot)
        with env:
            self.robot.SetTransform(orig_config)
        return collision

    def IsInLimits(self, node_id):
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        for idx in range(len(config)):
            if config[idx] < self.lower_limits[idx] or config[idx] > self.upper_limits[idx]:
                return False
        return True