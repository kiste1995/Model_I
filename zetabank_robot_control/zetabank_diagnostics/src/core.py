#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: Unspecified
#
import py_trees


def init_blackboard():
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.pid = None
    blackboard.client = None
    blackboard.device = None
    return blackboard


def log(msg):
    pass