from __future__ import print_function


def background_ready_banner(height=8):
    """Return a conspicuous, terminal-friendly background READY block."""
    marker = ["|"] * max(4, int(height))
    return marker + [
        "[BACKGROUND] Status:READY",
        ">>> BACKGROUND LEARNING COMPLETE - START TEST TARGETS NOW <<<",
    ] + marker
