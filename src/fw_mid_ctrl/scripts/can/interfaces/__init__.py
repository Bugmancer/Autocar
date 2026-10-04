# coding: utf-8

"""
本项目维护 python-can 的 SocketCAN 子集，供 Linux 车机底盘通信使用。
保留第三方插件入口发现机制；未随项目提供的硬件后端不在内建表中注册。
"""

import warnings
from pkg_resources import iter_entry_points


# 接口名称映射到模块和总线类，供 can.interface 按需导入。
BACKENDS = {
    'socketcan':        ('can.interfaces.socketcan',        'SocketcanBus'),
}

BACKENDS.update({
    interface.name: (interface.module_name, interface.attrs[0])
    for interface in iter_entry_points('can.interface')
})

# Old entry point name. May be removed >3.0.
for interface in iter_entry_points('python_can.interface'):
    BACKENDS[interface.name] = (interface.module_name, interface.attrs[0])
    warnings.warn('{} is using the deprecated python_can.interface entry point. '.format(interface.name) +
                  'Please change to can.interface instead.', DeprecationWarning)

VALID_INTERFACES = frozenset(list(BACKENDS.keys()) + ['socketcan_native', 'socketcan_ctypes'])
