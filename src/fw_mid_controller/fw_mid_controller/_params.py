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
        # ROS1 参数优先取节点私有命名空间；控制器本身不依赖 rospy。
        try:
            return getter("~" + name, default)
        except TypeError:
            return getter(name, default)

    getter = getattr(source, "get_parameter", None)
    if getter is not None:
        # 兼容提供 ROS2 风格参数接口的调用者，解包 Parameter.value。
        try:
            value = getter(name)
            return getattr(value, "value", value)
        except Exception:
            return default

    return default
