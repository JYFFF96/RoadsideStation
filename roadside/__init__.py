__version__ = "0.6.12"

from .track_stale_cleanup import install_stale_cleanup_patch

install_stale_cleanup_patch()
print("RoadsideStation V0.6.12 Track Stale Cleanup policy active")
