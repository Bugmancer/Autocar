"""Parameter access shared by controller classes.

Controllers are intentionally usable in pure-Python tests.  A ROS1 node (or
the ``rospy`` module itself) can be passed as the parameter source, while a
small fake object with ``get_param`` is sufficient for unit tests.
"""


def get_param(source, name, default):
    if source is None:
        return default

    getter = getattr(source, "get_param", None)
    if getter is not None:
        try:
            return getter("~" + name, default)
        except TypeError:
            return getter(name, default)

    getter = getattr(source, "get_parameter", None)
    if getter is not None:
        try:
            value = getter(name)
            return getattr(value, "value", value)
        except Exception:
            return default

    return default
