import math

def make_tree(path_, parent, n):
    parent[0] = -1
    mlist = [0]
    path = path_.copy()
    cnt = 0
    visited = [False for k in range(n)]

    while True:
        x = mlist.pop(0)
        print(x)
        for i, item in enumerate(path):
            if visited[i]:
                continue
            if item[0] == x:
                parent[item[1]] = x
                cnt +=1
                mlist.append(item[1])
                visited[i] = True
            elif item[1] == x:
                parent[1] = x
                cnt +=1
                mlist.append(item[0])
                visited[i] = True
        if cnt == n-1:
            break
    return

def isancesstor(x, y, parent):
    t = parent[x]

    while (t != -1):
        if t==y:
            return True
        t = parent[t]

    return False



def ispossible(a, b, parent):
    if isancesstor(a[0], b[1], parent) and isancesstor(b[0], a[1], parent):
        return False
    else:
        return True

def permutation(candidates, Prepermuation, res):
    if len(candidates) == 0: res.append(Prepermuation); return
    else:
        for i in range(len(candidates)):
            permutation(candidates[:i]+candidates[i+1:], Prepermuation + [ candidates[i] ], res)
        return

def solution(n, path, order):
    answer = True
    parent = [-2 for i in range(n)]
    l = len(order)

    make_tree(path, parent, n)
    print(parent)

    for i in range(l):
        for j in range(i+1, l):
            tmp = [order[i], order[j]]
            if not ispossible(tmp[0], tmp[1], parent):
                answer = False
    return answer


#print(solution(9, [[0,1],[0,3],[0,7],[8,1],[3,6],[1,2],[4,7],[7,5]], [[8,5],[6,7],[4,1]]))
print(solution(9, [[0,1],[0,3],[0,7],[8,1],[3,6],[1,2],[4,7],[7,5]], [[4,1],[8,7],[6,5]]))
