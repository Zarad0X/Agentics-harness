"""Progress integrity for successful, partial, and pre-decision failed HOME.

Empty receipts are valid only with independently aligned failed lifecycle and
unissued command packets. They can never satisfy completion gates.
"""
import hashlib
from home_functional_v22 import progress_checks as original_checks


def progress_checks(p):
    checks=original_checks(p);trace=p['trace']
    receipts=[e for e in trace if e['kind']=='home_progress_receipt']
    if len(receipts)!=1 or receipts[0]['decisions']!=0:return checks
    r=receipts[0]
    try:
        def one(kind):
            rows=[e for e in trace if e['kind']==kind and e.get('motion_phase')=='home']
            if len(rows)!=1:raise ValueError(kind)
            return rows[0]
        a=one('home_progress_admitted');plan=one('return_motion_plan');phase=one('motion_phase_exit')
        entry=one('motion_phase_entry');motion=one('return_motion_trace');feedback=one('velocity_feedback_receipt')
        packets=motion['samples']+motion['failure_window'];empty=hashlib.sha256(b'').hexdigest()
        valid=(type(r['decisions']) is int and type(r['packet_count']) is int and type(phase['completed_ticks']) is int
            and p['accepted'] is False and r['failed'] is True and r['reference_complete'] is False
            and phase['failed'] is True and motion['failed'] is True and bool(phase['reason'])
            and phase['reason']==motion['reason'] and r['packet_count']==0 and r['samples']==r['tail']==[]
            and r['all_packets_sha256']==empty and r['first_regulation'] is None and r['paused_ticks']==0
            and r['max_nominal_velocity_rad_s']==r['max_nominal_acceleration_rad_s2']==0
            and phase['completed_ticks']==phase['completed_hold_ticks']
            and phase['initial_scene_step']==entry['scene_step']==plan['initial_readback']['scene_step_count']
            and phase['final_scene_step']-phase['initial_scene_step']==phase['completed_ticks']
            and feedback['completed_ticks']==phase['completed_ticks']
            and bool(packets) and packets[-1]['packet_stage']=='exception')
        for e in (r,a,plan,phase,entry,motion,feedback):
            valid=valid and e['arm']==plan['arm'] and e['object_id']==plan['object_id']
        for row in packets:
            valid=valid and ('home_progress_decision' not in row and row['last_issued_index'] is None
                and row['last_issued_q'] is None and row['command_index']==0
                and all(row[k] is False for k in ('command_attempted','command_issued','tick_attempted','tick_completed','physics_tick_issued'))
                and row['monitor_reference_q']==plan['vertices'][0]
                and phase['initial_scene_step']<=row['readback']['scene_step_count']<=phase['final_scene_step'])
        if phase['completed_ticks']==0:
            valid=valid and feedback['packet_count']==feedback['command_attempts']==0 and feedback['all_numeric_sha256']==empty
        checks['retained_command_decision_alignment']=bool(valid)
        checks['retained_progress_decisions']=bool(valid)
        checks['zero_decision_failed_lifecycle']=bool(valid)
    except (KeyError,ValueError,TypeError):checks['zero_decision_failed_lifecycle']=False
    return checks
