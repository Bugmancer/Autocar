"""控制器共用的参数读取适配。

控制器不直接依赖 ROS 运行时：参数源既可以是 ROS1 节点或 ``rospy`` 模块，
也可以是单元测试中仅实现 ``get_param`` 的替身对象。
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
