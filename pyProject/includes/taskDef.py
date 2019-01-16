import numpy as np
import numpy.ma as ma
from scipy.integrate import quad, dblquad, nquad
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import integrate as intg
import random as rd

class Task(object):
    def __init__(self,AgentsConfig, InitializationConfig, FieldConfig, SimuConfig):
        self.field = Field(AgentsConfig, InitializationConfig, FieldConfig)
        self.dt = SimuConfig['dt']
        self.endTime = SimuConfig['endTime']
        self.fig = plt.figure()

        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        plt.draw()
    def run(self):
        for i in range(200):
            #### calculate velocities ####
            if self.field.isDeadLocked():
                if self.field.isReachedDestin():
                    self.field.setDeadLckToFalse()
                else:
                    pass
                if not(self.field.getIsDeadLck()):
                    self.field.calculateVels()
            else:
                self.field.calculateVels()
            self.field.combVCol()

            #### move agents ####
            self.field.moveAgents(self.dt)
            self.field.calculateErr()
            #### plots ####
            plt.pause(0.01)
            self.ax.scatter(self.field.agents.getPos()[:, 0], self.field.agents.getPos()[:, 1])
            plt.draw()
            # self.field.plotErrMesh()
            # self.field.plotAgentPos()
            # plt.clf()
            print(self.field.getTotalErr())

class Agents(object):
    def __init__(self, AgentsConfig, InitializationConfig):
        self.numberOfAgents = AgentsConfig['numberOfAgents']
        self.MaxValue = AgentsConfig['MaxValue']
        self.r = AgentsConfig['r']
        self.rForCalculCapacity = self.r * 1.2
        self.degreeOfCostFunc = AgentsConfig['degreeOfCostFunc']
        self.InitialResolution = InitializationConfig['resolutionMesh']
        self.Velocities = np.zeros([self.numberOfAgents, 2])
        self.AgentPositions = np.zeros([self.numberOfAgents, 2])
        self.basicFunc = BasicFunctions()
        self.calInitCapacity()

    def moveAgents(self, dt):
        self.AgentPositions = self.Velocities * dt

    def setPos(self, pos):
        self.AgentPositions = pos
    def getPos(self):
        return self.AgentPositions

    def setVelos(self, vel):
        self.Velocities = vel
    def getVelos(self):
        return self.Velocities

    def getNumerOfAgents(self):
        return self.numberOfAgents

    def get_r(self):
        return self.r

    def calInitCapacity(self):
        step = 2 * self.rForCalculCapacity / self.InitialResolution
        X = np.arange(-self.rForCalculCapacity, self.rForCalculCapacity + 1e-5, step)
        Y = np.arange(-self.rForCalculCapacity, self.rForCalculCapacity + 1e-5, step)
        X, Y = np.meshgrid(X, Y)
        capacities, deriv_s = self.calCapacitiesMesh(X, Y)
        self.plotOneCapFunc(X, Y, capacities[0])
        print("totla capacity under the Effective function: " , self.integrateCap())
        return capacities[0], deriv_s

    def plotOneCapFunc(self, X, Y, capacities):
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.plot_surface(X, Y, capacities)
        plt.show()

    def integrateCap(self):
        return dblquad(lambda x,y: self.calCapacitiesPts([0, 0], x,y), -self.rForCalculCapacity,self.rForCalculCapacity,
                       lambda y: -self.rForCalculCapacity,self.rForCalculCapacity)

    def calCapacitiesPts(self, pt, x, y):
        dist = self.basicFunc.distSquarePtToPt(pt, x, y)
        return (dist < self.r * self.r) * self.MaxValue / pow(self.r, 4) * pow(dist - self.r * self.r, 2)

    def calCapacitiesMesh(self, X, Y):
        dists = self.basicFunc.distSquarePtsToMesh(self.AgentPositions, X, Y)
        capacities = [(dist <= self.r * self.r) * self.MaxValue / pow(self.r, 4) * pow(dist - self.r * self.r, 2) for dist in dists]
        deriv_s = [(dist <= self.r * self.r) * 2 * self.MaxValue / pow(self.r, 4) * (dist - self.r * self.r) for dist in dists]
        return capacities, deriv_s

class Field(object):
    def __init__(self, AgentsConfig, InitializationConfig, FieldConfig):
        self.agents = Agents(AgentsConfig, InitializationConfig)
        step_x, step_y = FieldConfig['size_x'] / FieldConfig['resolution'], FieldConfig['size_y'] / FieldConfig['resolution']
        self.xMin, self.xMax = -FieldConfig['size_x'] / 2, FieldConfig['size_x'] / 2
        self.yMin, self.yMax = -FieldConfig['size_y'] / 2, FieldConfig['size_y'] / 2
        self.k_cov = FieldConfig['k_cov']
        self.k_col = FieldConfig['k_col']
        self.k_linear = FieldConfig['k_linear']
        self.R_col = self.agents.get_r() * 1.5
        self.r_col = self.agents.get_r() * 1.2
        # self.frame = plt.figure()
        # self.ax_mesh = self.frame.add_subplot(211, projection='3d')
        # self.ax_pos = self.frame.add_subplot(111)
        # self.ax_pos.grid(True)
        #
        # plt.tight_layout()
        # plt.draw()

        self.x = np.arange(self.xMin, self.xMax + 1e-5, step_x)
        self.y = np.arange(self.yMin, self.yMax + 1e-5, step_y)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.MeshErr = FieldConfig['C_star'] * np.ones(self.X.shape)
        self.totalErr = FieldConfig['C_star'] * (self.xMax - self.xMin) * (self.yMax - self.yMin)
        self.totalErrBuffer = self.totalErr
        self.isDeadLck = False
        self.minDistPt = np.zeros([self.agents.getNumerOfAgents(), 2])
        self.randomInit()
        self.calculateErr()
        self.plotErrMesh()

    def randomInit(self):
        # randomly init the positions near the edge of the field
        dice = np.random.random_integers(1, 4 , [self.agents.numberOfAgents, ])
        pos = np.zeros([self.agents.numberOfAgents, 2])
        for i_dice in range(self.agents.numberOfAgents):
            if dice[i_dice] == 1:
                pos[i_dice] = [self.xMin + (2 * np.random.rand() - 1) * self.agents.r,
                                self.yMin + np.random.rand() * (self.yMax - self.yMin)]
            elif dice[i_dice] == 2:
                pos[i_dice] = [self.xMax + (2 * np.random.rand() - 1) * self.agents.r,
                                self.yMin + np.random.rand() * (self.yMax - self.yMin)]
            elif dice[i_dice] == 3:
                pos[i_dice] = [self.xMin + np.random.rand() * (self.xMax - self.xMin),
                                self.yMin + (2 * np.random.rand() - 1) * self.agents.r]
            elif dice[i_dice] == 4:
                pos[i_dice] = [self.xMin + np.random.rand() * (self.xMax - self.xMin),
                           self.yMax + (2 * np.random.rand() - 1) * self.agents.r]

        self.agents.setPos(pos)
        return True

    def calculateErr(self):
        for i_capacity in self.agents.calCapacitiesMesh(self.X, self.Y)[0]:
            self.MeshErr = self.MeshErr - i_capacity
        self.MeshErr[self.MeshErr < 0] = 0

    def getTotalErr(self):
        self.totalErrBuffer = self.totalErr
        self.totalErr = intg.trapz(intg.trapz(self.MeshErr,self.x),self.y)
        return self.totalErr

    def isDeadLocked(self):
        self.isDeadLck = self.totalErr - self.totalErrBuffer <= 1e-4
        return self.getIsDeadLck()

    def getIsDeadLck(self):
        return self.isDeadLck
    def setDeadLckToFalse(self):
        self.isDeadLck = False
    def findNewDestInDeadLck(self):
        pts = self.agents.getPos()
        dists = np.array(self.agents.basicFunc.distSquarePtsToMesh(pts, self.X, self.Y))
        x = self.X.reshape(-1)
        y = self.X.reshape(-1)
        ds = dists.reshape(self.agents.getNumerOfAgents(), -1)
        err = self.MeshErr.reshape(-1)
        mask = (err <= 0)

        d_arrays = [[ma.array(d, mask=mask)] for d in ds]
        min_dists = [ma.min(d) for d in d_arrays]
        ds_list = [d.tolist() for d in ds]
        x = x.tolist()
        y = y.tolist()
        min_indxs = []
        for i in range(len(ds_list)):
            min_indxs.append(ds_list[i].index(min_dists[i]))
        self.minDistPt = [[x[i_minIndx], y[i_minIndx]] for i_minIndx in min_indxs]

    def isReachedDestin(self):
        return np.sum(np.sum(np.abs(self.agents.getPos() - self.minDistPt), axis=1)\
                      < self.agents.get_r() * self.agents.get_r()) == self.agents.getNumerOfAgents()
    def calVelDeadLck(self):
        self.findNewDestInDeadLck()
        v_tmp = -self.k_linear * (self.agents.getPos() - self.minDistPt)
        self.agents.setVelos(v_tmp)

    # def GiveIntegFuncVel(self, x, y):
    def calculateVels(self):
        diffVec_x, diffVec_y = self.agents.basicFunc.diffVecMesh(self.agents.AgentPositions, self.X, self.Y)
        capacities, derivs = self.agents.calCapacitiesMesh(X=self.X, Y=self.Y)
        dervCostFunc = self.agents.basicFunc.dervCostFunction(self.agents.degreeOfCostFunc, self.MeshErr)
        dists = self.agents.basicFunc.distSquarePtsToMesh(self.agents.AgentPositions, self.X, self.Y)

        ElemToInteg_x = [(dists[i] < self.agents.r * self.agents.r) * derivs[i] * dervCostFunc * diffVec_x[i]
                         for i in range(self.agents.getNumerOfAgents())]
        ElemToInteg_y = [(dists[i] < self.agents.r * self.agents.r) * derivs[i] * dervCostFunc * diffVec_y[i]
                         for i in range(self.agents.getNumerOfAgents())]
        v_x = intg.trapz(intg.trapz(ElemToInteg_x, self.x), self.y)
        v_y = intg.trapz(intg.trapz(ElemToInteg_y, self.x), self.y)
        velocities_new = self.k_cov * np.array([[v_x[i_agent],v_y[i_agent]] for i_agent in range(self.agents.getNumerOfAgents())])
        self.agents.setVelos(velocities_new)
        return



    def calColAvoidVel(self):
        agent_x, agent_y = [[x] for x in self.agents.getPos()[:, 0]], [[y] for y in self.agents.getPos()[:, 1]]

        diff_dist_x, diff_dist_y = np.transpose(agent_x) - agent_x, np.transpose(agent_y) - agent_y
        diff_dist_x_sq, diff_dist_y_sq = diff_dist_x * diff_dist_x, diff_dist_y * diff_dist_y

        diff_dist = np.sqrt(diff_dist_x_sq + diff_dist_y_sq)
        diff_dist_sq = diff_dist * diff_dist

        isInRange = (diff_dist < self.R_col) * (diff_dist > self.r_col)

        coeff = 4 * (self.R_col * self.R_col - self.r_col * self.r_col) * (diff_dist_sq - self.R_col * self.R_col)\
                / pow((diff_dist_sq - self.r_col * self.r_col), 3) * isInRange
        v_x = np.sum(coeff * diff_dist_x)
        v_y = np.sum(coeff * diff_dist_y)

        return -self.k_col * np.array([v_x, v_y])

    def combVCol(self):
        v_tmp = self.agents.getVelos()
        v_tmp += self.calColAvoidVel()
        self.agents.setVelos(v_tmp)
        return

    def plotErrMesh(self):
        # self.ax_mesh.clear()
        # self.ax_mesh.plot_surface(self.X, self.Y, self.MeshErr)
        # self.frame.colorbar(surf, shrink=0.5, aspect=5)
        plt.draw()

    def plotAgentPos(self):
        # self.ax_pos.scatter(self.agents.getPos()[:, 0], self.agents.getPos()[:, 1])
        # plt.draw()
        return

    def moveAgents(self, dt):
        self.agents.moveAgents(dt)

class Results(object):
    def __init__(self):
        self.err_time = []
        self.velocities_time = []
        self.positions_time = []
        self.mesh_diff = []

    def plotResults(self):
        pass
    def saveResults(self, filePath):
        pass
    def loadResults(self, filePath):
        pass

class BasicFunctions(object):
    def __init__(self):
        pass

    def distSquarePtToMesh(self, Pt, X, Y):
        return (Pt[0]-X)*(Pt[0]-X) + (Pt[1]-Y)*(Pt[1]-Y)

    def distSquarePtsToMesh(self, Pts, X, Y):
        res = [self.distSquarePtToMesh(i_Pt, X, Y) for i_Pt in Pts[:]]
        return res

    def distSquarePtToPt(self, Pt, x, y):
        return (Pt[0] - x) * (Pt[0] - x) + (Pt[1] - y) * (Pt[1] - y)

    def dervCostFunction(self, degree, ErrMesh):
        return (ErrMesh > 0) * degree * pow(ErrMesh, degree - 1)

    def diffVecMesh(self, Pts, X, Y):
        return [i_Pt[0] - X for i_Pt in Pts], [i_Pt[1] - Y for i_Pt in Pts]

    def diffVecPt(self, Pts, x, y):
        return [i_Pt[0] - x for i_Pt in Pts], [[i_Pt[1] - y for i_Pt in Pts[:]]]
