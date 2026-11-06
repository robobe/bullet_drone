from io import StringIO

import pytest
from loguru import logger

from examples.position_square.diagnostics import PHASE_COLORS, log_transition, log_waypoint
from examples.position_square.mission import Phase


@pytest.mark.parametrize('colorize', [True, False])
def test_phase_and_waypoint_colors_keep_plain_log_content(colorize):
    output = StringIO()
    sink = logger.add(output, format='{message}', colorize=colorize)
    try:
        previous = None
        for phase in Phase:
            log_transition(previous, phase, 1.25)
            previous = phase
        log_waypoint(2, (3., 3., 3.), 10.)
        log_waypoint(2, (3., 3., 3.), 11., settled=True)
    finally:
        logger.remove(sink)
    text = output.getvalue()
    assert set(PHASE_COLORS) == set(Phase)
    assert 'STATE STARTUP -> ' in text
    assert 'STATE TAKEOFF -> ' in text
    assert 'waypoint 2 active' in text and 'waypoint 2 settled' in text
    assert 'target=(3.0, 3.0, 3.0) m | sim=10.000s' in text
    assert ('\x1b[' in text) is colorize
    if colorize:
        assert '\x1b[36mTAKEOFF\x1b[0m' in text
        assert '\x1b[36mwaypoint 2 active\x1b[0m' in text
        assert '\x1b[32mwaypoint 2 settled\x1b[0m' in text
    else:
        assert 'STATE SETTLE -> TAKEOFF' in text
        assert '<cyan>' not in text and '<green>' not in text
