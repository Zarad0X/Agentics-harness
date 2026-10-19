"""Return-only coverage and bounded nominal placement lookahead.

Nominal grasps are ranking hints, never physical evidence or hard exclusions.
Every failed lookahead stays in the fallback; measured loaded planning remains
authoritative after regrasp. No learned hints are created here.
"""
import copy
import itertools
import numpy as np
import astra_planner as planner
from surface_grasps import mesh_grasps
from collision_world import pose_matrix
from pose_goals import ee_for_payload

DEPTH_GROUPS = ((.014,.025,.04,.06),(.008,),(.004,),(.001,),(.0005,))


def complete_grasps(pose, parts):
    groups = [mesh_grasps(pose, parts, depths) for depths in DEPTH_GROUPS]
    # Preserve the old prefix exactly, append all previously omitted entries.
    return [c for group in groups for c in group[:24]] + [c for group in groups for c in group[24:]]


def above_goal(manifest, goal, ee_goal):
    height=max(1.02,max(o['aabb_max'][2] for o in manifest['layout_validation']['objects'].values())+.08)
    above=ee_goal[:3,3].copy();above[2]+=max(.14,height-goal[2])
    return above


def nominal_placement(api, manifest, arm, target, candidate):
    s=api.state();goal=manifest['objects'][str(target)]['pose']
    ee=np.eye(4);ee[:3,3]=candidate['ee_position_m'];ee[:3,:3]=candidate['ee_rotation_world']
    relative=np.linalg.inv(ee)@pose_matrix(s['objects'][str(target)]['pose'])
    end=ee_for_payload(goal,relative);above=above_goal(manifest,goal,end)
    # Isolate lookahead randomness from the physical forecast stream.
    rng=copy.deepcopy(api.rng);q=candidate['grasp_q']
    world=api.world(s);world.held=(arm,str(target),relative)
    fingers=[float(np.clip(candidate['closing_width_m']/2,0,.04))]*2
    previous=None;checks=0
    for p in np.linspace(above,end[:3,3],int(np.ceil(np.linalg.norm(above-end[:3,3])/.02))+1):
        ik=planner.solve_ik(api.model,arm,p,end[:3,:3],q,rng=rng,max_iters=60,restarts=0)
        if not ik['success']:return dict(feasible=False,reason='nominal_ik',ik=ik,collision_checks=checks)
        q=ik['q']
        # Check both endpoints and the swept joint segment, never endpoints alone.
        path=[q] if previous is None else np.linspace(previous,q,max(2,int(np.ceil(np.max(np.abs(np.asarray(q)-previous))/.025))+1)).tolist()
        for sample in path:
            checks+=1
            if not world.free(arm,sample,target,True,fingers):
                return dict(feasible=False,reason='nominal_collision',collision=world.last_collision,collision_checks=checks)
        previous=np.asarray(q)
    return dict(feasible=True,reason='nominal_insertion_checked',collision_checks=checks,
                limitation='nominal transform and finger width; transit and actual grasp require complete physical previews')


def goal_conditioned(api, manifest, arm, target, candidates):
    # Only the first eight source-feasible grasps incur lookahead. Never discard
    # a candidate because bounded IK or the nominal rigid transform failed.
    prefix=list(itertools.islice(candidates,8));preferred=[];fallback=[]
    for index,c in prefix:
        report=nominal_placement(api,manifest,arm,target,c)
        api.event('return_nominal_placement',object_id=target,index=index,arm=arm,**report)
        (preferred if report['feasible'] else fallback).append((index,c))
    yield from preferred
    yield from fallback
    yield from candidates
