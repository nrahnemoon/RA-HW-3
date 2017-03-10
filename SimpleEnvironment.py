import numpy
import pylab as pl
from matplotlib import collections as ml
import scipy
import scipy.spatial
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.env1 = self.robot.GetEnv()

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
            node_add_id = self.discrete_env.GridCoordToNodeId(node_add)
            if (node_add_id !=-1 and self.IsInLimits(node_add_id)==True and self.IsInCollision(node_add_id)!=True):
                successors.append(node_add_id)
        for j in range(0,numpy.size(node,0)):
            node_minus = list(node)
            node_minus[j] = node_minus[j]-1
            node_minus_id = self.discrete_env.GridCoordToNodeId(node_minus)
            if (node_minus_id !=-1 and self.IsInLimits(node_minus_id)==True and self.IsInCollision(node_minus_id)!=True):
                successors.append(node_minus_id)
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        end_grid = self.discrete_env.NodeIdToGridCoord(end_id)


        #dist = numpy.linalg.norm(start_config-end_config)
        dist = scipy.spatial.distance.cityblock(start_grid,end_grid)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        start_grid = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_grid = self.discrete_env.NodeIdToGridCoord(goal_id)
        cost = scipy.spatial.distance.cityblock(start_grid,goal_grid)

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

    def PlotAll(self,plan):
        lines=[]
        for i in range(0,numpy.size(plan,0)-1):
            start = plan[i]
            end = plan[i+1]
            temp = [start,end]
            pl.plot([start[0], end[0]],
                    [start[1], end[1]],
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
        if config == -1:
            return False
        for idx in range(len(config)):
            if config[idx] < self.lower_limits[idx] or config[idx] > self.upper_limits[idx]:
                return False
        return True

    def SetGoalParameters(self, goal_config, p = 0.2):
 
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        #
        # TODO: Generate and return a random configuration
        #
        config[0] = numpy.random.uniform(low=self.lower_limits[0], high=self.upper_limits[0]);
        config[1] = numpy.random.uniform(low=self.lower_limits[1], high=self.upper_limits[1]);
        
        return numpy.array(config)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        origTransform = self.robot.GetTransform()

        steps = 10;
        xSteps = numpy.linspace(start_config[0], end_config[0], (steps + 1));
        ySteps = numpy.linspace(start_config[1], end_config[1], (steps + 1));

        for i in range(steps + 1):

            transform = self.robot.GetTransform()
            transform[0, 3] = xSteps[i]
            transform[1, 3] = ySteps[i]
            self.robot.SetTransform(transform);
            

            for body in self.robot.GetEnv().GetBodies():
                if (body.GetName() != self.robot.GetName() and
                        self.robot.GetEnv().CheckCollision(body, self.robot)):
                    #self.robot.SetTransform(origTransform);
                    if (i == 0):
                        return None;
                    else:
                        return [xSteps[i-1], ySteps[i-1]]

        #self.robot.SetTransform(origTransform)

        return end_config
