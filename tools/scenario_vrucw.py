from __future__ import print_function
import os,sys
script=os.path.join(os.path.dirname(os.path.abspath(__file__)),"spawn_multiclass_targets.py")
os.execv(sys.executable,[sys.executable,script,"--scenario","vrucw"]+sys.argv[1:])
