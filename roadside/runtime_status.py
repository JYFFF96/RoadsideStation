from __future__ import print_function


def background_ready_banner(width=72):
    """Return a conspicuous, terminal-friendly background READY block."""
    separator = "-" * max(36, int(width))
    return [
        separator,
        "[BACKGROUND] Status:READY",
        ">>> BACKGROUND LEARNING COMPLETE - START TEST TARGETS NOW <<<",
        separator,
    ]
