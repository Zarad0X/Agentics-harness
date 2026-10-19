"""Explicit independent-scene return diagnostic entry point; never a formal run."""
import argparse
import sys
from pathlib import Path
from return_reconstruction import load_bundle


def main():
    ap=argparse.ArgumentParser(add_help=False)
    ap.add_argument('--bundle',type=Path,required=True)
    ap.add_argument('--candidate-index',type=int)
    ap.add_argument('--max-previews',type=int,default=10)
    ap.add_argument('--legacy-insertion',action='store_true')
    ap.add_argument('--target',type=int,choices=(10,12),default=10)
    ap.add_argument('--fixed-setpoint-diagnostic',choices=('baseline','hold'))
    ap.add_argument('--continuation-diagnostic',choices=('natural','same-first','switch-first'))
    ap.add_argument('--n17-observer',choices=('baseline','observe'))
    args,remaining=ap.parse_known_args()
    if not 1<=args.max_previews<=40 or (args.candidate_index is not None and args.candidate_index<0):
        raise ValueError('diagnostic_budget')
    bundle=load_bundle(args.bundle,args.target)
    if args.continuation_diagnostic and (args.target!=12 or args.candidate_index!=6 or args.max_previews!=2
            or args.legacy_insertion or args.fixed_setpoint_diagnostic):
        raise ValueError('continuation_diagnostic_requires_n12_c6_two_trials')
    sys.argv=[sys.argv[0]]+remaining
    import physics_runner
    return physics_runner.main(dict(bundle=bundle,candidate_index=args.candidate_index,
                                   max_previews=args.max_previews,legacy_insertion=args.legacy_insertion,
                                   fixed_setpoint_diagnostic=args.fixed_setpoint_diagnostic,
                                   continuation_diagnostic=args.continuation_diagnostic,
                                   n17_observer=args.n17_observer))


if __name__=='__main__':raise SystemExit(main())
