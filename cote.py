import sys
from itertools import permutations

def get_score(order):
    global score

    o=0
    tasa = order[o]
    sum = 0
    inning = 0
    while inning<n:
        roo = [0]*3
        out = 0
        while out != 3:
            now = score[inning][tasa]
            if now == 0:
                out +=1
            elif now == 1:
                sum += roo[2]
                roo[2] = roo[1]
                roo[1] = roo[0]
                roo[0] = 1
            elif now == 2:
                sum += roo[2] + roo[1]
                roo[2] = roo[0]
                roo[1] = 1
                roo[0] = 0
            elif now == 3:
                sum += roo[2] + roo[1] + roo[0]
                roo[2] = 1
                roo[1] = 0
                roo[0] = 0
            else:
                sum += roo[2] + roo[1] + roo[0] + 1
                roo[2] = 0
                roo[1] = 0
                roo[0] = 0


            o = (o + 1) % 9
            tasa = order[o]
        inning +=1

    return sum

def permutation(candidates, prepermutation, res):
    if len(candidates) == 0:
        res.append(prepermutation); return
    else:
        for i in range(len(candidates)):
            permutation(candidates[:i]+candidates[i+1:], prepermutation+[candidates[i]], res )

n = int(input())

score = [list(map(int, sys.stdin.readline().split())) for i in range(n)]

#print("score:", score)

tazas = [1,2,3,4,5,6,7,8]
pers= permutations(tazas, 8)
ma = 0
case = [0 for i in range(9)]
for p in pers:
    case = list(p[:3]) + [0] + list(p[3:])
    sc = get_score(case)
    if sc>ma:
        ma = sc

print(ma)









    

