__version__ = "0.6.12.8.2.2.31"

from .track_stale_cleanup import install_stale_cleanup_patch
from .far_geometry_stability import install_far_geometry_stability_patch

install_stale_cleanup_patch()
install_far_geometry_stability_patch()
print("RoadsideStation V0.6.12.8.2.2.31 Town05 Auto Map Selection active")
