"""
this module includes functions for finding global path points.
Global path points are divided into groups using dfs algorithm. Near points are assinged the same group number.
It uses kruskal's algorithm for finding MST with given global path points group.
"""


# to find MST, use Edge class
class Edge:
    def __init__(self,u,v,w):
        self.u=u
        self.v=v
        self.w=w

# for MST, uniform find
def find(a, uf):
    if (uf[a]<0):
        return a
    else :
        return find(uf[a], uf)
# for MST
def merge(a,b,uf):
    a=find(a, uf)
    b=find(b, uf)
    if (a==b):
        return False
    else :
        uf[b]=a
        return True

# for finding MST path when global planning
def dfs_point(visit_point, points, ad, result, num):
    if (not visit_point[num]):
        result.append(points[num])
        visit_point[num] = 1
        for item in ad[num]:
            dfs_point(visit_point, points, ad, result, item)

# find MST using Kruskal's algorithm
def MST(points, get_distance):
    n=len(points)
    ad=[[] for _ in range(n)] # ad contains connected points for each edges
    visit_point=[0 for _ in range(n)]
    edgelist=[]
    uf=[]
    for i in range(n):
        uf.append(-1)
    for i in range(n):
        for j in range(i+1,n):
            edgelist.append(Edge(i, j, get_distance(points[i].coordi, points[j].coordi)))
    # sort edges in an increasing order
    edge_sorted=sorted(edgelist, key = lambda x: x.w)
    edge_path=[]
    cnt=0
    for edge in edge_sorted:
        if merge(edge.u, edge.v, uf):
            cnt += 1
            ad[edge.u].append(edge.v)
            ad[edge.v].append(edge.u)
            edge_path.append(edge)
        if cnt == (n - 1):
            break
    result=[]
    dfs_point(visit_point, points, ad, result, 0) # find global path & store it in result
    return result

