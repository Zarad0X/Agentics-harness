"""Fair candidate search on a scene that changes only after a committed pick.

Failed forecasts never advance the source scene. Cursors can therefore resume
on the next visit. A successful pick invalidates every remaining cursor because
it may remove an obstruction or change neighboring poses.
"""
from collections import deque


class CandidateSearch:
    def __init__(self, targets, candidate_budget=24, forecast_budget=6):
        targets = list(targets)
        if len(set(targets)) != len(targets):
            raise ValueError('duplicate target')
        if candidate_budget < 1 or forecast_budget < 1:
            raise ValueError('search budgets must be positive')
        self.pending = deque(targets)
        self.cursors = dict.fromkeys(targets, 0)
        self.exhausted = set()
        self.candidate_budget = candidate_budget
        self.forecast_budget = forecast_budget
        self.active = None

    def next_target(self):
        if self.active is not None:
            raise RuntimeError('previous visit not finalized')
        for _ in range(len(self.pending)):
            target = self.pending.popleft()
            if target not in self.exhausted:
                self.active = target
                return target
            self.pending.append(target)
        return None

    def indices(self, total):
        if self.active is None:
            raise RuntimeError('no active visit')
        start = self.cursors[self.active]
        return range(start, min(total, start + self.candidate_budget))

    def advance(self, index):
        if self.active is None or index != self.cursors[self.active]:
            raise RuntimeError('candidate cursor out of order')
        self.cursors[self.active] = index + 1

    def defer(self, total):
        if self.active is None:
            raise RuntimeError('no active visit')
        target = self.active
        if self.cursors[target] >= total:
            self.exhausted.add(target)
        self.pending.append(target)
        self.active = None
        return target in self.exhausted

    def complete(self):
        if self.active is None:
            raise RuntimeError('no active visit')
        del self.cursors[self.active]
        self.active = None
        self.exhausted.clear()
        self.cursors = dict.fromkeys(self.pending, 0)
