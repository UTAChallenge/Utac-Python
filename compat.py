import inspect

if not hasattr(inspect, 'getargspec'):
    def getargspec(func):
        return inspect.getfullargspec(func)

    inspect.getargspec = getargspec
