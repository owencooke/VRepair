import numpy as np

'''
Pythonic version of "interparc" written for MATLAB
Adapted from: https://stackoverflow.com/a/42940234
'''
def interpcurve(N,points):
    #equally spaced in arclength
    N=np.transpose(np.linspace(0,1,N))

    #how many points will be uniformly interpolated?
    nt=N.size

    #number of points on the curve
    n=len(points)
    # points=np.array((pX,pY,pZ)).T
    p1=points[0,:]
    pend=points[-1,:]
    # last_segment= np.linalg.norm(np.subtract(p1,pend))
    # epsilon= 10*np.finfo(float).eps

    # #IF the two end points are not close enough lets close the curve
    # if last_segment > epsilon*np.linalg.norm(np.amax(abs(points),axis=0)):
    #     points=np.vstack((points,p1))
    #     nt = nt + 1
    # else:
    #     print('Contour already closed')

    pt=np.zeros((nt,2))

    #Compute the chordal arclength of each segment.
    chordlen = (np.sum(np.diff(points,axis=0)**2,axis=1))**(1/2)
    #Normalize the arclengths to a unit total
    chordlen = chordlen/np.sum(chordlen)
    #cumulative arclength
    cumarc = np.append(0,np.cumsum(chordlen))

    tbins= np.digitize(N,cumarc) # bin index in which each N is in

    #catch any problems at the ends
    tbins[np.where(np.bitwise_or(tbins<=0, N<=0))] = 1
    tbins[np.where(np.bitwise_or(tbins >= n, N >= 1))] = n - 1      

    s = np.divide((N - cumarc[tbins]),chordlen[tbins-1])
    pt = points[tbins,:] + np.multiply((points[tbins,:] - points[tbins-1,:]),(np.vstack([s]*3)).T)

    return pt, tbins


if __name__ == "__main__":
    m = np.array([
        [1.18123127328309,	47.0664149693081,	-2.26053702886203],
        [2.21117165811010,	47.0664149693081,	-2.33263352371475],
        [2.39737740183979,	47.0664149693081,	-2.34566804652998],
        [4.16299979917466,	47.0664149693081,	-2.40746456102411],
        [6.39737740183979,	47.0664149693081,	-2.48566743617842],
        [8.16299979917466,	47.0664149693081,	-2.54746395067254],
        [10.3973774018398,	47.0664149693081,	-2.39105820036392],
        [11.5750137883302,	47.0664149693081,	-2.30862401269569],
        [12.1629997991747,	47.0664149693081,	-2.26746517137567],
        [12.3609421975938,	47.0664149693081,	-2.26053702886203]
    ])
    print(interpcurve(3, m))