import conf

def recognizer(cls):
    for k, v in conf.RECOGNIZER.items():
        setattr(cls, k, v)
    return cls

def docking(cls):
    for k, v in conf.DOCKING.items():
        setattr(cls, k, v)
    return cls

def recognizer(cls):
    for k, v in conf.RECOGNIZER.items():
        setattr(cls, k, v)
    return cls

def ros(cls):
    for k, v in conf.ROS.items():
        setattr(cls, k, v)
    return cls
    
def logger(cls):
    for k, v in conf.LOGGER.items():
        setattr(cls, k, v)
    return cls