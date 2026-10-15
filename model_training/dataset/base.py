class Stat:
    name = "stat"

    def compute(self, coco):
        """Return a dict of numbers computed from a loaded COCO dict."""
        raise NotImplementedError

    def report(self, result):
        """Print a human-readable summary of a compute() result."""

    def plot(self, result, ax):
        return False
