class foo:
    def __init__(self,val):
        self.val = val
    @staticmethod
    def init(val):
        return foo(val)


a = foo.init(99)

print(a.val)