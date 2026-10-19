"""Astra-authored GJK distance using tiny convex-simplex least-squares solves.

A support map over vertices represents their convex hull. No collision engine.
Unconverged cases return zero distance (conservative rejection).
"""
import itertools
import numpy as np


def closest_simplex(points):
    points=np.asarray(points,float);best=None
    for n in range(1,min(4,len(points))+1):
        for ids in itertools.combinations(range(len(points)),n):
            a=points[list(ids)]
            if n==1:w=np.ones(1)
            else:
                mat=np.block([[a@a.T,np.ones((n,1))],[np.ones((1,n)),np.zeros((1,1))]])
                w=np.linalg.lstsq(mat,np.r_[np.zeros(n),1.],rcond=1e-12)[0][:n]
                if np.min(w)<-1e-9 or abs(w.sum()-1)>1e-6:continue
            w=np.maximum(w,0);w/=w.sum();v=w@a;score=float(v@v)
            if best is None or score<best[0]:best=(score,v,a[w>1e-10])
    if best is None:raise ValueError('invalid simplex')
    return best[1],best[2]


def convex_distance(a,b,max_iterations=64):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.ndim!=2 or b.ndim!=2 or a.shape[1]!=3 or b.shape[1]!=3 or not len(a) or not len(b) or not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('finite nonempty 3D vertices required')
    def support(d):return a[np.argmax(a@d)]-b[np.argmin(b@d)]
    d=a.mean(0)-b.mean(0)
    if np.linalg.norm(d)<1e-12:d=np.array([1.,0,0])
    simplex=np.array([support(d)]);v=simplex[0]
    for _ in range(max_iterations):
        norm=float(v@v)
        if norm<1e-14:return 0.
        w=support(-v)
        if norm-float(v@w)<=1e-10*max(1.,norm):return float(np.sqrt(norm))
        if np.min(np.linalg.norm(simplex-w,axis=1))<1e-10:return float(np.sqrt(norm))
        v,simplex=closest_simplex(np.vstack((simplex,w)))
    return 0.


def box_vertices(b):
    c,r,h=b
    return np.array([c+r@(h*np.array(s)) for s in itertools.product((-1,1),repeat=3)])
