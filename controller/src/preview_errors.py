"""Only explicitly identified physical/planning infeasibility may be retried."""
import traceback


class CandidateInfeasible(RuntimeError):
    """A checked IK/path/contact/tracking outcome, not an implementation error."""


def exception_diagnostic(exc):
    from fixed_setpoint_diagnostic import DiagnosticHoldStop,DiagnosticHoldCompleted
    if isinstance(exc,DiagnosticHoldStop):
        outcome='diagnostic_completed' if isinstance(exc,DiagnosticHoldCompleted) else 'diagnostic_guard_failed'
        return dict(failure_type=type(exc).__name__,failure=str(exc),retryable=False,
                    failure_category=outcome,diagnostic_outcome=outcome,
                    failure_traceback=''.join(traceback.format_exception(type(exc),exc,exc.__traceback__)))
    from action_tape import TapeFailure
    from forecast_report_io_v36 import SerializationFailure
    retryable=isinstance(exc,CandidateInfeasible)
    return dict(failure_type=type(exc).__name__,failure=str(exc),retryable=retryable,
                failure_category='infrastructure_serialization' if isinstance(exc,(TapeFailure,SerializationFailure)) else ('candidate_infeasible' if retryable else 'fatal_error'),
                failure_traceback=''.join(traceback.format_exception(type(exc),exc,exc.__traceback__)))
