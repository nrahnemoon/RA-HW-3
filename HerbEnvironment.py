import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        for idx in range(self.discrete_env.dimension):
            coord[idx] = coord[idx] + 1
            up_node_id = self.discrete_env.GridCoordToNodeId(coord)
            if self.IsInLimits(up_node_id) and not self.IsInCollision(up_node_id):
                successors.append(up_node_id)
            coord[idx] = coord[idx] - 2
            down_node_id = self.discrete_env.GridCoordToNodeId(coord)
            if self.IsInLimits(down_node_id) and not self.IsInCollision(down_node_id):
                successors.append(down_node_id)
            coord[idx] = coord[idx] + 1
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id) 
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        sum = 0;
        for idx in range(len(start_coord)):
            sum += (end_config[idx] - start_config[idx])**2;

        dist = numpy.sqrt(sum)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        for i in range(len(start_config)):
            cost = cost + abs(start_config[i] - goal_config[i])

        return cost

    def IsInLimits(self, node_id):
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        for idx in range(len(config)):
            if config[idx] < self.lower_limits[idx] or config[idx] > self.upper_limits[idx]:
                return False
        return True

    def IsInCollision(self, node_id):
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        orig_config = self.robot.GetActiveDOFValues()
        env = self.robot.GetEnv()

        with env:
            self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices())

        collision = env.CheckCollision(self.robot)

        with env:
            self.robot.SetDOFValues(orig_config, self.robot.GetActiveDOFIndices())

        return collision