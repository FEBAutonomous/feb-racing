"""Hold the camera still: the RealSense runs auto exposure and auto white balance by default, and
under indoor lighting that makes the same cone drift between blue and yellow from frame to frame.
A venue profile can fix both (camera_auto_exposure / camera_exposure, camera_auto_wb /
camera_white_balance); this sets them on the camera node over its parameter service, retrying
until the node is up. Nothing is sent when the profile leaves the camera on automatic.
"""
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


def wanted(p):
    """The camera parameters a profile asks for, as (name, value) pairs; empty when all automatic."""
    out = []
    if not p.get("camera_auto_exposure", True):
        out.append(("rgb_camera.enable_auto_exposure", False))
        if int(p.get("camera_exposure", 0)) > 0:
            out.append(("rgb_camera.exposure", int(p["camera_exposure"])))
    if not p.get("camera_auto_wb", True):
        out.append(("rgb_camera.enable_auto_white_balance", False))
        if int(p.get("camera_white_balance", 0)) > 0:
            out.append(("rgb_camera.white_balance", int(p["camera_white_balance"])))
    return out


def _param(name, value):
    v = ParameterValue()
    if isinstance(value, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
    else:
        v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, int(value)
    return Parameter(name=name, value=v)


class CameraLock:
    """Call apply() from a timer; it returns True once the settings are on the camera (or nothing was asked)."""

    def __init__(self, node, p, camera_node="/camera"):
        self.node, self.items = node, wanted(p)
        self.client = node.create_client(SetParameters, camera_node + "/set_parameters") if self.items else None
        self.future, self.done, self.tries = None, not self.items, 0

    def apply(self):
        if self.done:
            return True
        if self.future is not None:
            if self.future.done():
                try:
                    res = self.future.result()
                    ok = all(r.successful for r in res.results)
                except Exception:
                    ok = False
                self.future = None
                if ok:
                    self.done = True
                    self.node.get_logger().info("camera locked: " + ", ".join("%s=%s" % kv for kv in self.items))
                    return True
                self.node.get_logger().warn("camera lock: the camera refused a setting; retrying")
            return False
        if not self.client.service_is_ready():
            self.tries += 1
            if self.tries % 10 == 1:
                self.node.get_logger().info("camera lock: waiting for the camera node")
            return False
        req = SetParameters.Request()
        req.parameters = [_param(n, v) for n, v in self.items]
        self.future = self.client.call_async(req)
        return False
