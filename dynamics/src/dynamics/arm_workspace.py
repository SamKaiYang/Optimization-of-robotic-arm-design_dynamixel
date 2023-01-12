#constant inputs
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import math
from scipy.spatial import Delaunay
import alphashape
from shapely.ops import cascaded_union, polygonize
import shapely.geometry as geometry

class arm_workspace_plane:
    def __init__(self, symbolic=False, ang_arr1=None, ang_arr=None, link_lengths=None, rse=30):
        self.fig, self.ax = plt.subplots(2,figsize=(5,10))
        self.rse = rse
        self.Ang_arr = ang_arr
        self.Ang_arr_angle1 = ang_arr1
        #angular domain in radians
        self.L_arr = link_lengths
        self.L_arr = np.array([0.335,0.335,0.09])
        self.X_base = 0
        self.Y_base = 0
        self.ln1, self.ln2, self.ln3, self.ln4 = self.ax[0].plot([], [], 'r-', 
                                [], [], 'b-', 
                                [], [], 'y-',  
                                [], [], 'c-', linewidth=3, 
                                animated=False) #animated is associated with blit
        self.xdata1, self.ydata1 = [], []
        self.xdata2, self.ydata2 = [], []
        self.xdata3, self.ydata3 = [], []
        self.xdata4, self.ydata4 = [], []

    def alpha_shape(self, points, alpha, only_outer=True):
        assert points.shape[0] > 3, "Need at least four points"

        def add_edge(edges, i, j):
            if (i, j) in edges or (j, i) in edges:
                # already added
                assert (j, i) in edges, "Can't go twice over same directed edge right?"
                if only_outer:
                    # if both neighboring triangles are in shape, it's not a boundary edge
                    edges.remove((j, i))
                return
            edges.add((i, j))

        tri = Delaunay(points)
        edges = set()
        for ia, ib, ic in tri.vertices:
            pa = points[ia]
            pb = points[ib]
            pc = points[ic]
            a = np.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
            b = np.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
            c = np.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
            s = (a + b + c) / 2.0
            area = np.sqrt(s * (s - a) * (s - b) * (s - c))
            circum_r = a * b * c / (4.0 * area)
            if circum_r < alpha:
                add_edge(edges, ia, ib)
                add_edge(edges, ib, ic)
                add_edge(edges, ic, ia)
        return edges

    def find_edges_with(self, i, edge_set):
        i_first = [j for (x,j) in edge_set if x==i]
        i_second = [j for (j,x) in edge_set if x==i]
        return i_first,i_second

    def stitch_boundaries(self, edges):
        edge_set = edges.copy()
        boundary_lst = []
        while len(edge_set) > 0:
            boundary = []
            edge0 = edge_set.pop()
            boundary.append(edge0)
            last_edge = edge0
            while len(edge_set) > 0:
                i,j = last_edge
                j_first, j_second = self.find_edges_with(j, edge_set)
                if j_first:
                    edge_set.remove((j, j_first[0]))
                    edge_with_j = (j, j_first[0])
                    boundary.append(edge_with_j)
                    last_edge = edge_with_j
                elif j_second:
                    edge_set.remove((j_second[0], j))
                    edge_with_j = (j, j_second[0])  # flip edge rep
                    boundary.append(edge_with_j)
                    last_edge = edge_with_j

                if edge0[0] == last_edge[1]:
                    break

            boundary_lst.append(boundary)
        return boundary_lst

    def Wspace_mod(self, A_arr = None,theta_arr = None,res = None): 
        aone = A_arr[0]
        a2 = A_arr[1]
        a3 = A_arr[2]
        numPt = res
        numPt_3 = numPt * numPt * numPt
        q1 = np.linspace(theta_arr[0],theta_arr[1],numPt)
        
        q2 = np.linspace(theta_arr[2],theta_arr[3],numPt)
        
        q3 = np.linspace(theta_arr[4],theta_arr[5],numPt)
        
        #declaring array size for final storage before push
        x_stor = np.ones((1,numPt_3))
        y_stor = np.ones((1,numPt_3))
        angle_stor = np.ones((numPt_3,3))
        storage_i = 0
        for i in np.arange(0,numPt).reshape(-1):
            for j in np.arange(0,numPt).reshape(-1):
                for k in np.arange(0,numPt).reshape(-1):
                    #X,Y coordinates calculated by the projections of the
        #instantaneous joint positions for respective angles.
                    xcord = aone * math.cos(q1[i]) + a2 * math.cos(q1[i] + q2[j]) + a3 * math.cos(q1[i] + q2[j] + q3[k])
                    ycord = aone * math.sin(q1[i]) + a2 * math.sin(q1[i] + q2[j]) + a3 * math.sin(q1[i] + q2[j] + q3[k])
                    x_stor[0][storage_i] = xcord
                    y_stor[0][storage_i] = ycord
                    angle_stor[storage_i,:] = np.array([q1[i],q2[j],q3[k]])
                    #columns 3,4 and 5.
                    storage_i = storage_i + 1
        xytheta_arr1 = np.stack((np.transpose(x_stor[0]),np.transpose(y_stor[0])),axis = 1)
        xytheta_arr = np.concatenate((xytheta_arr1,angle_stor),axis = 1)
        points = np.vstack([xytheta_arr[:,0],xytheta_arr[:,1]]).T
        edges = self.alpha_shape(points, alpha=0.05)
        k = self.stitch_boundaries(edges)
        self.ax[0].plot(xytheta_arr[:,1],xytheta_arr[:,0],'c.')
        for i, j in edges:
            self.ax[0].plot(points[[i, j], 1], points[[i, j], 0], linewidth=3)

        return xytheta_arr,edges

    def xy_Wspace_mod(self, A_arr = None,theta_arr = None,res = None): 
        aone = A_arr[0]+A_arr[1]+A_arr[2]
        numPt = res
        q1 = np.linspace(theta_arr[0],theta_arr[1],numPt)
        #declaring array size for final storage before push
        x_stor = np.ones((1,numPt))
        y_stor = np.ones((1,numPt))
        angle_stor = np.ones((numPt,3))
        storage_i = 0
        for i in np.arange(0,numPt).reshape(-1):
            xcord = aone * math.cos(q1[i])
            ycord = aone * math.sin(q1[i])
            x_stor[0][storage_i] = xcord
            y_stor[0][storage_i] = ycord
            angle_stor[storage_i,:] = np.array([q1[i]])
            #columns 3,4 and 5.
            storage_i = storage_i + 1
        xytheta_arr1 = np.stack((np.transpose(x_stor[0]),np.transpose(y_stor[0])),axis = 1)
        xytheta_arr = np.concatenate((xytheta_arr1,angle_stor),axis = 1)
        # TODO: fixed the shape issue

        # TODO: fixed the boundary issue
        points = np.vstack([xytheta_arr[:,0],xytheta_arr[:,1]]).T
        edges = self.alpha_shape(points, alpha=0.05)
        k = self.stitch_boundaries(edges)
        self.ax[1].plot(xytheta_arr[:,1],xytheta_arr[:,0],'c.')

        p1 = [xytheta_arr[len(xytheta_arr[:,1])-1,1],xytheta_arr[len(xytheta_arr[:,0])-1,0]]
        p2 = [xytheta_arr[0,1], xytheta_arr[0,0]]
        p3 = [0,0]
        self.ax[1].plot([p1[0],p3[0]],[p1[1],p3[1]],'c', linewidth=3)
        self.ax[1].plot([p2[0],p3[0]],[p2[1],p3[1]],'c', linewidth=3)

        return xytheta_arr,edges
    # function that draws each frame of the animation
    def update(self, i): #i is an int from 0 to frames-1, and keep looping
        self.ln1.set_data(self.xdata1[i], self.ydata1[i])
        self.ln2.set_data(self.xdata2[i], self.ydata2[i])
        self.ln3.set_data(self.xdata3[i], self.ydata3[i])
        self.ln4.set_data(self.xdata4[i], self.ydata4[i])
        return self.ln1, self.ln2, self.ln3, self.ln4

    def plot_Wspace_mod(self, CA,maxmin, animation=False):

        if animation == True:
            self.ln1, self.ln2, self.ln3, self.ln4 = self.ax[0].plot([], [], 'r-', 
                            [], [], 'b-', 
                            [], [], 'y-',  
                            [], [], 'c-', linewidth=3, 
                            animated=False) #animated is associated with blit
            for i, j in maxmin:
                q1 = CA[[i, j], 2]
                q2 = CA[[i, j], 3]
                q3 = CA[[i, j], 4]
                #declaring X,Y coordinates for Joint 1 wrt base
                A1_x_tip = 0 + (self.L_arr[0] * np.cos(q1))
                A1_y_tip = 0 + (self.L_arr[0] * np.sin(q1))
                # L1 = plt.plot(np.array([Y_base,A1_y_tip[0]]),np.array([X_base,A1_x_tip[0]]),'r')
                #declaring X,Y coordinates for Joint 2 wrt base(i.e. joint 1)
                A2_x_tip = A1_x_tip + (self.L_arr[1] * np.cos(q1 + q2))
                A2_y_tip = A1_y_tip + (self.L_arr[1] * np.sin(q1 + q2))
                # L2 = plt.plot(np.array([A1_y_tip,A2_y_tip]),np.array([A1_x_tip,A2_x_tip]),'g')
                #declaring X,Y coordinates for Joint 3 wrt base(i.e. joint 2)
                A3_x_tip = A2_x_tip + (self.L_arr[2] * np.cos(q1 + q2 + q3))
                A3_y_tip = A2_y_tip + (self.L_arr[2] * np.sin(q1 + q2 + q3))
                # L3 = plt.plot(np.array([A2_y_tip,A3_y_tip]),np.array([A2_x_tip,A3_x_tip]),'b')
                #Marking the End Effector separately  
                # E_eff = plt.plot(A3_y_tip[0],A3_x_tip[0],'.')

                self.ydata1.append(np.array([self.X_base,A1_x_tip[0]]))
                self.xdata1.append(np.array([self.Y_base,A1_y_tip[0]]))
                self.ydata2.append(np.array([A1_x_tip,A2_x_tip]))
                self.xdata2.append(np.array([A1_y_tip,A2_y_tip]))
                self.ydata3.append(np.array([A2_x_tip,A3_x_tip]))
                self.xdata3.append(np.array([A2_y_tip,A3_y_tip]))
                self.ydata4.append(A3_x_tip[0])
                self.xdata4.append(A3_y_tip[0])
            # run the animation
            self.ax[0].set_title("Workspace Perimeter Sweep - YZ plane")
            self.ax[1].set_title("Workspace Perimeter Sweep - XY Plane")
            self.ax[0].set_xlabel("Y")
            self.ax[0].set_ylabel("Z")
            self.ax[1].set_xlabel("X")
            self.ax[1].set_ylabel("Y")
            ani = FuncAnimation(self.fig, self.update, frames=len(maxmin) , interval=1, repeat=False)
        plt.show()

if __name__=="__main__":

    Ang_arr = np.array([- math.pi * 125 / 180, math.pi * 85 / 180, -math.pi * 145 / 180, math.pi * 95 / 180, -math.pi * 115 / 180, math.pi * 115 / 180])
    Ang_arr1 = np.array([- math.pi * 170 / 180, math.pi * 170 / 180])
    L_arr = np.array([0.335,0.335,0.045])
    Wspace = arm_workspace_plane(ang_arr1= Ang_arr1, ang_arr=Ang_arr, link_lengths=L_arr)
    bb,aa = Wspace.xy_Wspace_mod(Wspace.L_arr, Wspace.Ang_arr_angle1, 360)
    CA,maxmin = Wspace.Wspace_mod(Wspace.L_arr, Wspace.Ang_arr, res=30)
    Wspace.plot_Wspace_mod(CA,maxmin,False)