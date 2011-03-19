XEN_SECURITY_MODULE = "dummy"
from xsm_core import *

import xen.util.xsm.dummy.dummy as xsm_module

xsm_init(xsm_module)
from xen.util.xsm.dummy.dummy import *
del xsm_module

