#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from math import pi

import numpy as np
from spatialmath import SE3

from roboticstoolbox import DHRobot, RevoluteMDH


class RM65B(DHRobot):
    def __init__(self):
        a = [0, 0, 0.256, 0, 0, 0]
        d = [0.2405, 0, 0, 0.210, 0, 0.144]
        alpha = [0, pi / 2, 0, pi / 2, -pi / 2, pi / 2]
        offset = [0, pi / 2, pi / 2, 0, 0, 0]

        links = []
        for i_ in range(6):
            link = RevoluteMDH(
                d=d[i_],
                a=a[i_],
                alpha=alpha[i_],
                offset=offset[i_],
            )
            links.append(link)

            tool = SE3(0, 0, 0)

        super().__init__(
            links,
            name="rm65b",
            manufacturer="realman robot",
            tool=tool,
        )

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))


class RM65F(DHRobot):
    def __init__(self):
        a = [0, 0, 0.256, 0, 0, 0]
        d = [0.2405, 0, 0, 0.210, 0, 0.1725]
        alpha = [0, pi / 2, 0, pi / 2, -pi / 2, pi / 2]
        offset = [0, pi / 2, pi / 2, 0, 0, 0]

        links = []
        for i_ in range(6):
            link = RevoluteMDH(
                d=d[i_],
                a=a[i_],
                alpha=alpha[i_],
                offset=offset[i_],
            )
            links.append(link)

            tool = SE3(0, 0, 0)

        super().__init__(
            links,
            name="rm65f",
            manufacturer="realman robot",
            tool=tool,
        )

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))


class RM65ZF(DHRobot):
    def __init__(self):
        a = [0, 0, 0.256, 0, 0, 0]
        d = [0.2405, 0, 0, 0.210, 0, 0.152]
        alpha = [0, pi / 2, 0, pi / 2, -pi / 2, pi / 2]
        offset = [0, pi / 2, pi / 2, 0, 0, 0]

        links = []
        for i_ in range(6):
            link = RevoluteMDH(
                d=d[i_],
                a=a[i_],
                alpha=alpha[i_],
                offset=offset[i_],
            )
            links.append(link)

            tool = SE3(0, 0, 0)

        super().__init__(
            links,
            name="rm65zf",
            manufacturer="realman robot",
            tool=tool,
        )

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))


class RM65SF(DHRobot):
    def __init__(self):
        a = [0, 0, 0.256, 0, 0, 0]
        d = [0.2405, 0, 0, 0.210, 0, 0.1725]
        alpha = [0, pi / 2, 0, pi / 2, -pi / 2, pi / 2]
        offset = [0, pi / 2, pi / 2, 0, 0, 0]

        links = []
        for i_ in range(6):
            link = RevoluteMDH(
                d=d[i_],
                a=a[i_],
                alpha=alpha[i_],
                offset=offset[i_],
            )
            links.append(link)

            tool = SE3(0, 0, 0)

        super().__init__(
            links,
            name="rm65sf",
            manufacturer="realman robot",
            tool=tool,
        )

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))
