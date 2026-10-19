"""Physical regrasp and payload-frame exact return, via the Astra stack only."""
import numpy as np
import time
import astra_planner as planner
from collision_world import pose_matrix
from surface_grasps import mesh_grasps
from pose_goals import ee_for_payload,pose_errors,insertion_order
from preview_errors import CandidateInfeasible
from return_candidates import complete_grasps,above_goal,goal_conditioned


class ReturnPipeline:
    def __init__(self,api,manifest,config):
        self.a=api;self.manifest=manifest;self.config=config;self.returned=[]
        self.candidates=0;self.previews=0
        self.deadline=None

    def check_budget(self,preview=False):
        if not self.config.defer_intermediate_home:return
        reason=None
        if self.deadline is not None and time.monotonic()>=self.deadline:reason='time_cap'
        if preview and self.previews>=self.config.return_preview_cap:reason='preview_cap'
        if reason:raise CandidateInfeasible('return_continuation_'+reason)

    def start_budget(self):
        self.deadline=time.monotonic()+self.config.return_time_cap_s
        self.a.return_budget(self.deadline)

    def options(self,target,arms=None):
        o=self.a.state()['objects'][str(target)];arm='right' if o['pose'][0]>=0 else 'left'
        for candidate_arm in arms or (arm,'left' if arm=='right' else 'right'):
            stream=self.candidates_for(candidate_arm,target)
            for index,candidate in goal_conditioned(self.a,self.manifest,candidate_arm,target,stream):
                self.check_budget()
                yield candidate_arm,index,candidate

    def final_home(self,target):
        self.a.event('return_final_home_started',object_id=target,state=self.a.state())
        for arm in ('left','right'):
            self.a.home(arm,target)
        self.a.event('return_final_home_completed',object_id=target,state=self.a.state())

    def relative(self,arm,target):
        return np.linalg.inv(pose_matrix(self.a.ee_pose(arm)))@pose_matrix(self.a.state()['objects'][str(target)]['pose'])

    def candidates_for(self,arm,target):
        a=self.a;s=a.state();p=pose_matrix(s['objects'][str(target)]['pose'])
        candidates=complete_grasps(p,a.meshes[str(target)])
        candidates=a.rank(candidates,target)
        for index,c in enumerate(candidates):
            self.check_budget()
            self.candidates+=1
            a.event('return_candidate',object_id=target,index=index,arm=arm,candidate=c)
            q=s['robots'][arm]['q'][:7];p=np.asarray(c['ee_position_m']);r=c['ee_rotation_world']
            ik=planner.solve_ik(a.model,arm,p+[0,0,.1],r,q,rng=a.rng,max_iters=100,restarts=1)
            if not ik['success']:continue
            grasp=planner.solve_ik(a.model,arm,p,r,ik['q'],rng=a.rng,max_iters=100,restarts=1)
            if not grasp['success']:continue
            world=a.world(s)
            if not world.free(arm,ik['q']) or not world.free(arm,grasp['q'],target,False):continue
            yield index,dict(c,pregrasp_q=ik['q'],grasp_q=grasp['q'])

    def place(self,arm,target):
        """Recompute T_EO after lifting and before every insertion waypoint."""
        a=self.a;goal=self.manifest['objects'][str(target)]['pose']
        relative=self.relative(arm,target);ee_goal=ee_for_payload(goal,relative)
        # Transit clears the original tallest geometry and container rim.
        above=above_goal(self.manifest,goal,ee_goal)
        a.event('return_payload_transform',object_id=target,ee_to_object=relative.tolist(),goal_pose=goal,
                ee_goal=ee_goal.tolist(),above=above.tolist())
        if self.config.high_transfer_branch_seed:
            a.plan_to(arm,above,ee_goal[:3,:3],target,relative,branch_seed=True)
        else:
            a.plan_to(arm,above,ee_goal[:3,:3],target,relative)
        # A bounded straight Cartesian insertion. Each segment has fresh IK,
        # sampled collision checks and guarded real physics; no loaded RRT
        # detour through support contacts on the final insertion.
        distance=float(above[2]-ee_goal[2,3])
        for remaining in np.linspace(distance,0,int(np.ceil(distance/.01))+1)[1:]:
            relative=self.relative(arm,target)
            now_goal=ee_for_payload(goal,relative);p=now_goal[:3,3]+[0,0,remaining]
            a.insert(arm,p,now_goal[:3,:3],target,relative)
        before=a.state()['objects'][str(target)]['pose'];pe,re=pose_errors(before,goal)
        if pe>.005 or re>3.:raise CandidateInfeasible('payload_not_at_release_goal:'+str((pe,re)))
        a.release(arm,target)
        a.event('return_release',object_id=target,goal_pose=goal,measured_pose=before,
                measured_pose_phase='before_release',state=a.state())
        # Physical fingers open at the reached pose; collision checks include
        # the measured released payload while allowing only target fingers.
        a.retreat(arm,target)
        if not self.config.defer_intermediate_home:a.home(arm,target)
        a.settle(target,500)
        state=a.state();pe,re=pose_errors(state['objects'][str(target)]['pose'],goal)
        # This raw observation precedes validation. Even if retained in a
        # failed preview it cannot grant acceptance or formal ledger credit.
        a.event('returned',object_id=target,goal_pose=goal,position_error_m=pe,rotation_error_deg=re,state=state,
                validation_pending=True,accepted=False)
        if pe>self.config.position_tolerance_m or re>self.config.rotation_tolerance_deg:
            raise CandidateInfeasible('settled_exact_pose_return_failed:'+str((pe,re)))
        obj=state['objects'][str(target)]
        speed=float(np.linalg.norm(obj['linear_velocity']))
        spin=float(np.linalg.norm(obj['angular_velocity']))
        if (not np.isfinite([speed,spin]).all() or speed>self.config.speed_tolerance_m_s
                or spin>self.config.spin_tolerance_rad_s):
            raise CandidateInfeasible('settled_return_velocity_failed:'+str((speed,spin)))

    def run(self,selected):
        if self.config.defer_intermediate_home:self.start_budget()
        try:return self._run(selected)
        except Exception as exc:
            if self.config.defer_intermediate_home:
                self.a.event('return_continuation_failed',reason=type(exc).__name__+':'+str(exc),
                    completed_return_ids=list(self.returned),previews=self.previews,candidates=self.candidates,
                    state=self.a.state(),recovery_attempted=False)
            raise
        finally:
            if self.config.defer_intermediate_home:self.a.return_budget(None)

    def _run(self,selected):
        a=self.a;selected=set(map(str,selected));present=set(self.manifest['objects'])-selected
        order=insertion_order(self.manifest['dependencies'],selected,present)
        a.event('return_order',object_ids=order,dependencies=self.manifest['dependencies'])
        for oid in order:
            target=int(oid);o=a.state()['objects'][oid];arm='right' if o['pose'][0]>=0 else 'left'
            chosen=None
            def preview_place():
                self.place(candidate_arm,target)
                if self.config.defer_intermediate_home and oid==order[-1]:self.final_home(target)
            for candidate_arm,index,candidate in self.options(target):
                # Forecast the COMPLETE physical regrasp, transport, insertion,
                # release and home twice in a separate physics system.
                passed=True
                for trial in (1,2):
                    self.check_budget(preview=True)
                    self.previews+=1
                    if not a.preview_return(candidate_arm,target,candidate,preview_place,index,trial):
                        passed=False;break
                a.experience_record(target,candidate,passed,'return')
                if passed:chosen=candidate;arm=candidate_arm;break
            if chosen is None:raise CandidateInfeasible('no_feasible_physical_return:'+oid)
            a.event('return_started',object_id=target)
            a.grasp(arm,target,chosen);self.place(arm,target)
            self.returned.append(target);a.ledger.put_back(target)
        if self.config.defer_intermediate_home and self.returned:self.final_home(self.returned[-1])
        return self.returned

    def diagnostic_next(self,first_arm,first_target,order,preference):
        """Continue this shadow once. No forecasts, resets, ledger or recovery.

        Two independent full-sequence trials provide the physical diagnostic.
        After a physical candidate failure this trial stops; no changed-state
        candidate retry is presented as the original continuation.
        """
        if len(order)<2 or str(first_target)!=order[0]:raise ValueError('diagnostic_dependency_prefix')
        self.a.event('diagnostic_first_grasp_completed',object_id=first_target,state=self.a.state(),formal_credit=False)
        self.place(first_arm,first_target)
        a=self.a;target=int(order[1]);other='left' if first_arm=='right' else 'right'
        arms=None if preference=='natural' else ((first_arm,other) if preference=='same-first' else (other,first_arm))
        a.event('diagnostic_first_return_completed',object_id=first_target,state=a.state(),formal_credit=False)
        chosen=next(self.options(target,arms),None)
        if chosen is None:raise CandidateInfeasible('return_continuation_no_next_grasp:'+str(target))
        arm,index,candidate=chosen
        a.event('diagnostic_next_selected',object_id=target,arm=arm,index=index,candidate=candidate,
                requested_preference=preference,actual_relation='same' if arm==first_arm else 'switch',
                state=a.state(),formal_credit=False)
        a.grasp(arm,target,candidate)
        a.event('diagnostic_next_grasp_completed',object_id=target,state=a.state(),formal_credit=False)
        self.place(arm,target)
        a.event('diagnostic_second_return_completed',object_id=target,state=a.state(),formal_credit=False)
        self.final_home(target);a.settle(target,500)
        a.event('diagnostic_pair_completed',object_ids=list(map(int,order[:2])),formal_credit=False,state=a.state())
