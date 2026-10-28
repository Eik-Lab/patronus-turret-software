class Stat:
    name = "stat"

    def compute(self, coco):
        raise NotImplementedError

    def report(self, result):
        pass

    def plot(self, result, ax):
        return False
