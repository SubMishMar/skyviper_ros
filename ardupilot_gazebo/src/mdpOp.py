# coding: utf-8
# Your code here!
import numpy as np
import pol

def outOfBounds(x,y,Rows,Cols):
    if (x > Rows-1) and (y > Cols-1):
        return 3
    elif (x < 0) and (y < 0):
        return 1
    elif (x > Rows-1) and (y < 0):
        return 4
    elif (x < 0) and (y > Cols-1):
        return 2
    elif (x > Rows-1):
        return 44
    elif (y > Cols-1):
        return 33
    elif (x < 0):
        return 11
    elif (y < 0):
        return 22
    else:
        return 0

def assignClosest(fx,fy,Rows,Cols):
    code = outOfBounds(fx,fy,Rows,Cols)
    if (code == 1):
        return [0,0]
    elif (code == 3):
        return [Rows-1, Cols-1]
    elif (code == 4):
        return [Rows-1, 0]
    elif (code == 2):
        return [0, Cols-1]
    elif (code == 44):
        return [Rows-1, fy]
    elif (code == 33):
        return [fx, Cols-1]
    elif (code == 11):
        return [0, fy]
    elif (code == 22):
        return [fx, 0]
    else:
        return [fx,fy]

def getWayPoint(x,y):
    Rows=len(pol.wpX)
    Cols=len(pol.wpX[0])
    if (pol.followPol == 0):
        fx=int(np.floor(x/pol.resX));
        fy=int(np.floor(y/pol.resY));
        [fx,fy]=assignClosest(fx,fy,Rows,Cols)
        wp=[(pol.startX*pol.resX)+0.25, (pol.startY*pol.resY)+0.25]
        if (fx == pol.startX) and (fy == pol.startY):
            pol.followPol=1
    elif (pol.followPol == 1):
        fx=int(np.floor(x/pol.resX));
        fy=int(np.floor(y/pol.resY));
        [fx,fy]=assignClosest(fx,fy,Rows,Cols)
        wp=[pol.wpX[fx][fy], pol.wpY[fx][fy]]
        if (fx == pol.goalX) and (fy == pol.goalY):
            if (pol.ret == 0):
                pol.followPol=3
            else:
                pol.followPol=2
    elif (pol.followPol == 2):
        fx=int(np.floor(x/pol.resX))
        fy=int(np.floor(y/pol.resY))
        [fx,fy]=assignClosest(fx,fy,Rows,Cols)
        wp=[pol.wpX_ret[fx][fy], pol.wpY_ret[fx][fy]]
        if (fx == pol.startX) and (fy == pol.startY):
            pol.followPol=3
    elif (pol.followPol == 3):
        #fx=int(np.floor(pol.resX/2.0))
        #fy=int(np.floor(pol.resY/2.0))
        #[fx,fy]=assignClosest(fx,fy,Rows,Cols)
        wp= [pol.resX/2.0, pol.resY/2.0]	
    else:
        #fx=int(np.floor(pol.resX/2.0))
        #fy=int(np.floor(pol.resY/2.0))
        #[fx,fy]=assignClosest(fx,fy,Rows,Cols)
        wp= [pol.resX/2.0, pol.resY/2.0]	
    return wp