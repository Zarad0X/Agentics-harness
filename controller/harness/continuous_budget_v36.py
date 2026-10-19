"""Finite future whole-episode allocations; not completion-time estimates."""
TIMEOUT_CAPS = {10: 86400, 15: 129600, 20: 172800}


def validate_timeout(count, timeout):
    """One strict contract for preparation and every manifest consumer."""
    if type(count) is not int or count not in TIMEOUT_CAPS:
        raise ValueError('continuous_timeout_count')
    if type(timeout) is not int or not 60 <= timeout <= TIMEOUT_CAPS[count]:
        raise ValueError('continuous_timeout_integer_60_to_count_cap')
    return timeout
