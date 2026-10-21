"""Presentation-only adjacent-lane tracking; never changes decision geometry.

Observed outer curves may leave the camera image near the bonnet. Render only
their supported, geometrically valid visible span. Each side confirms and ages
independently. Held curves are reprojected against CURRENT ego boundaries and
expire by source time, rather than the number of sampled/encoded frames.
"""
from dataclasses import dataclass
import math

import numpy as np


def _longest_span(mask):
    indices=np.flatnonzero(mask)
    if not len(indices):
        return None
    groups=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1)
    best=max(groups,key=len)
    return int(best[0]),int(best[-1])


@dataclass
class _Observation:
    y: np.ndarray
    ratio: np.ndarray


@dataclass
class _Side:
    active: _Observation | None = None
    pending: _Observation | None = None
    hits: int = 0
    pending_time: float = 0.
    seen: float = 0.


@dataclass(frozen=True)
class SideVisual:
    polygon: np.ndarray
    boundary: np.ndarray
    held: bool
    age_ms: float
    alpha: float
    presentation_only: bool = True


class AdjacentLaneVisualTracker:
    def __init__(self,fit_curve,hold_seconds=.18,smoothing_seconds=.06):
        self.fit_curve=fit_curve
        self.hold_seconds=hold_seconds
        self.smoothing_seconds=smoothing_seconds
        self.reset()

    def reset(self):
        self.sides=[_Side(),_Side()]
        self.last_time=None
        self.previous_ego=None

    @staticmethod
    def _width(fits,y):
        return np.polyval(fits[1],y)-np.polyval(fits[0],y)

    def _observe(self,points,slot,fits,far,near):
        if len(points)<5 or len(points)/81<=.25 or not np.isfinite(points).all():
            return None,False
        outer=self.fit_curve(points)
        if outer is None:
            return None,False
        top=max(far,float(points[:,1].min()))
        bottom=min(near,float(points[:,1].max()))
        if bottom-top<54:
            return None,False
        y=np.linspace(top,bottom,96)
        outside=np.polyval(outer,y)
        inside=np.polyval(fits[0 if slot==0 else 1],y)
        width=self._width(fits,y)
        gap=inside-outside if slot==0 else outside-inside
        # An observed slot crossing the current inner edge is contradictory
        # evidence, rather than a temporary missed measurement to hold over.
        if np.median(gap)<=0:
            return None,True
        valid=(np.isfinite(outside)&np.isfinite(inside)&np.isfinite(width)&
            (outside>=0)&(outside<=639)&(inside>=0)&(inside<=639)&(width>=10)&
            (gap>=np.maximum(8,width*.35))&(gap<=width*2.2))
        span=_longest_span(valid)
        if span is None or y[span[1]]-y[span[0]]<54:
            return None,False
        top,bottom=y[span[0]],y[span[1]]
        # Trim to observed anchors, not merely the polynomial's extrapolation.
        supported=points[(points[:,1]>=top)&(points[:,1]<=bottom)]
        if len(supported)<5:
            return None,False
        top=max(top,float(supported[:,1].min()))
        bottom=min(bottom,float(supported[:,1].max()))
        if bottom-top<54:
            return None,False
        yy=np.linspace(top,bottom,32)
        xx=np.polyval(outer,yy)
        inner=np.polyval(fits[0 if slot==0 else 1],yy)
        gap=inner-xx if slot==0 else xx-inner
        return _Observation(yy,gap/self._width(fits,yy)),False

    @staticmethod
    def _distance(a,b):
        top=max(a.y[0],b.y[0])
        bottom=min(a.y[-1],b.y[-1])
        if bottom-top<54:
            return float('inf')
        y=np.linspace(top,bottom,16)
        return float(np.percentile(np.abs(np.interp(y,a.y,a.ratio)-np.interp(y,b.y,b.ratio)),90))

    def _advance(self,state,observed,now):
        fresh=False
        if observed is not None:
            if state.active is not None and self._distance(state.active,observed)<.32:
                alpha=1-math.exp(-min(max(now-state.seen,0),.067)/self.smoothing_seconds)
                ratio=observed.ratio.copy()
                overlap=(observed.y>=state.active.y[0])&(observed.y<=state.active.y[-1])
                previous=np.interp(observed.y[overlap],state.active.y,state.active.ratio)
                ratio[overlap]=alpha*ratio[overlap]+(1-alpha)*previous
                state.active=_Observation(observed.y.copy(),ratio)
                state.pending=None
                state.hits=0
                fresh=True
            else:
                consecutive=(state.pending is not None and now-state.pending_time<=.35 and
                    self._distance(state.pending,observed)<.24)
                state.hits=state.hits+1 if consecutive else 1
                state.pending=observed
                state.pending_time=now
                if state.hits>=2:
                    state.active=observed
                    state.pending=None
                    state.hits=0
                    fresh=True
        else:
            state.pending=None
            state.hits=0
        if fresh:
            state.seen=now
        if state.active is not None and now-state.seen>self.hold_seconds+1e-9:
            state.active=None
        return fresh

    def _render(self,state,fresh,side,fits,far,near,now):
        if state.active is None:
            return None
        top=max(far,state.active.y[0])
        bottom=min(near,state.active.y[-1])
        if bottom-top<54:
            return None
        y=np.linspace(top,bottom,64)
        width=self._width(fits,y)
        inner=np.polyval(fits[side],y)
        ratio=np.interp(y,state.active.y,state.active.ratio)
        outside=inner+(-1 if side==0 else 1)*ratio*width
        gap=np.abs(outside-inner)
        valid=(np.isfinite(outside)&np.isfinite(inner)&(outside>=0)&(outside<=639)&
            (inner>=0)&(inner<=639)&(width>=10)&
            (gap>=np.maximum(8,width*.35))&(gap<=width*2.2))
        span=_longest_span(valid)
        if span is None or y[span[1]]-y[span[0]]<54:
            return None
        # Always reproject using this frame's ego curves, including during hold.
        yy=np.linspace(y[span[1]],y[span[0]],32)
        width=self._width(fits,yy)
        inner=np.polyval(fits[side],yy)
        ratio=np.interp(yy,state.active.y,state.active.ratio)
        outside=inner+(-1 if side==0 else 1)*ratio*width
        oc=np.rint(np.column_stack((outside,yy))).astype(np.int32)
        ic=np.rint(np.column_stack((inner,yy))).astype(np.int32)
        polygon=np.vstack((oc,ic[::-1])) if side==0 else np.vstack((ic,oc[::-1]))
        age=max(0,now-state.seen)
        alpha=.24 if fresh else .24*max(.18,1-age/self.hold_seconds)
        return SideVisual(polygon,oc,not fresh,age*1000,alpha)

    def update(self,measured,slots,lane,seconds):
        if not np.isfinite(seconds):
            self.reset()
            return (None,None)
        if self.last_time is not None and (seconds<=self.last_time or seconds-self.last_time>.5):
            self.reset()
        self.last_time=seconds
        if (not measured.valid or measured.partial or lane.held or lane.uncertain or
                lane.reassignment!='NONE'):
            self.reset()
            self.last_time=seconds
            return (None,None)
        fits=[self.fit_curve(slots[1]),self.fit_curve(slots[2])]
        if any(f is None or not np.isfinite(f).all() for f in fits):
            self.reset()
            return (None,None)
        far=max(float(slots[1][:,1].min()),float(slots[2][:,1].min()))
        near=min(float(slots[1][:,1].max()),float(slots[2][:,1].max()),352.8)
        if self.previous_ego is not None:
            old_fits,old_near=self.previous_ego
            probe=min(near,old_near)
            old_width=float(self._width(old_fits,probe))
            new_width=float(self._width(fits,probe))
            shift=abs(sum(np.polyval(f,probe) for f in fits)-sum(np.polyval(f,probe) for f in old_fits))*.5
            if old_width<=0 or not .75*old_width<new_width<1.34*old_width or shift>.20*old_width:
                self.sides=[_Side(),_Side()]
        self.previous_ego=(fits,near)
        visuals=[]
        for side,slot in enumerate((0,3)):
            observation,contradictory=self._observe(slots[slot],slot,fits,far,near)
            if contradictory:
                self.sides[side]=_Side()
            state=self.sides[side]
            fresh=self._advance(state,observation,seconds)
            visuals.append(self._render(state,fresh,side,fits,far,near,seconds))
        return tuple(visuals)
